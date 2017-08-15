//--------------------------------------------------------------------------------------------------
/**
    Qu3e Physics Engine - C# Version 1.01

	Copyright (c) 2014 Randy Gaul http://www.randygaul.net

	This software is provided 'as-is', without any express or implied
	warranty. In no event will the authors be held liable for any damages
	arising from the use of this software.

	Permission is granted to anyone to use this software for any purpose,
	including commercial applications, and to alter it and redistribute it
	freely, subject to the following restrictions:
	  1. The origin of this software must not be misrepresented; you must not
	     claim that you wrote the original software. If you use this software
	     in a product, an acknowledgment in the product documentation would be
	     appreciated but is not required.
	  2. Altered source versions must be plainly marked as such, and must not
	     be misrepresented as being the original software.
	  3. This notice may not be removed or altered from any source distribution.
*/
//--------------------------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using static Qu3e.Settings;

namespace Qu3e
{
    public class ContactManager
    {
        public ContactManager()
        {
            Broadphase = new BroadPhase(this);
            ContactList = new List<ContactConstraint>();
            ContactListener = null;
        }

        // Add a new contact constraint for a pair of objects
        // unless the contact constraint already exists
        public void AddContact(Box A, Box B)
        {
            Body bodyA = A.body;
            Body bodyB = B.body;
            if (!bodyA.CanCollide(bodyB))
                return;

            // Search for existing matching contact
            // Return if found duplicate to avoid duplicate constraints
            // Mark pre-existing duplicates as active
            foreach (var edge in A.body.ContactList)
            {
                if (edge.other == bodyB)
                {
                    Box shapeA = edge.constraint.A;
                    Box shapeB = edge.constraint.B;

                    // @TODO: Verify this against Box2D; not sure if this is all we need here
                    if ((A == shapeA) && (B == shapeB))
                        return;
                }
            }

            // Create new contact
            ContactConstraint contact = new ContactConstraint()
            {
                A = A,
                B = B,
                bodyA = A.body,
                bodyB = B.body,
                Flags = 0,
                friction = MixFriction(A, B),
                restitution = MixRestitution(A, B),
            };

            contact.manifold.SetPair(A, B);
            
            ContactList.Add(contact);

            // Connect A
            contact.edgeA.constraint = contact;
            contact.edgeA.other = bodyB;
            bodyA.ContactList.Add(contact.edgeA);

            // Connect B
            contact.edgeB.constraint = contact;
            contact.edgeB.other = bodyA;
            bodyB.ContactList.Add(contact.edgeB);

            bodyA.SetToAwake();
            bodyB.SetToAwake();
        }

        // Has broadphase find all contacts and call AddContact on the
        // ContactManager for each pair found
        public void FindNewContacts()
        {
            Broadphase.UpdatePairs();
        }

        // Remove a specific contact
        public void RemoveContact(ContactConstraint contact)
        {
            Body A = contact.bodyA;
            Body B = contact.bodyB;

            // Remove from A

            A.ContactList.Remove(contact.edgeA);

            // Remove from B
            B.ContactList.Remove(contact.edgeB);


            A.SetToAwake();
            B.SetToAwake();

            // Remove contact from the manager
            ContactList.Remove(contact);
        }

        // Remove all contacts from a body
        public void RemoveContactsFromBody(Body body)
        {
            foreach (var edge in body.ContactList)
            {
                RemoveContact(edge.constraint);
            }

        }
        public void RemoveFromBroadphase(Body body)
        {
            foreach (var box in body.Boxes)
            {
                Broadphase.RemoveBox(box);
            }
        }

        // Remove contacts without broadphase overlap
        // Solves contact manifolds
        public void TestCollisions()
        {

            for (int h = 0; h < ContactList.Count; h++)
            {
                var constraint = ContactList[h];
                Box A = constraint.A;
                Box B = constraint.B;
                Body bodyA = A.body;
                Body bodyB = B.body;

                constraint.Flags &= ~ContactFlags.eIsland;

                if (!bodyA.IsAwake() && !bodyB.IsAwake())
                {
                    continue;
                }

                if (!bodyA.CanCollide(bodyB))
                {

                    RemoveContact(constraint);
                    continue;
                }

                // Check if contact should persist
                if (!Broadphase.TestOverlap(A.broadPhaseIndex, B.broadPhaseIndex))
                {
                    RemoveContact(constraint);
                    continue;
                }
                Manifold manifold = constraint.manifold;
                Manifold oldManifold = constraint.manifold;
                Vec3 ot0 = oldManifold.tangentVectors;
                Vec3 ot1 = oldManifold.bitangentVectors;
                constraint.SolveCollision();
                AABB.ComputeBasis(manifold.normal, ref manifold.tangentVectors, ref manifold.bitangentVectors);

                for (int i = 0; i < manifold.contactCount; ++i)
                {
                    Contact c = manifold.contacts[i];
                    c.tangentImpulse = c.bitangentImpulse = c.normalImpulse = 0;
                    byte oldWarmStart = c.warmStarted;
                    c.warmStarted = 0;

                    for (int j = 0; j < oldManifold.contactCount; ++j)
                    {
                        Contact oc = oldManifold.contacts[j];
                        if (c.fp.key == oc.fp.key)
                        {
                            c.normalImpulse = oc.normalImpulse;

                            // Attempt to re-project old friction solutions
                            Vec3 friction = ot0 * oc.tangentImpulse + ot1 * oc.bitangentImpulse;
                            c.tangentImpulse = Vec3.Dot(friction, manifold.tangentVectors);
                            c.bitangentImpulse = Vec3.Dot(friction, manifold.bitangentVectors);
                            c.warmStarted = Math.Max(oldWarmStart, (byte)(oldWarmStart + 1));
                            break;
                        }
                    }
                }

                if (ContactListener != null)
                {
                    var now_colliding = constraint.Flags & ContactFlags.eColliding;
                    var was_colliding = constraint.Flags & ContactFlags.eWasColliding;

                    if (now_colliding > 0 && was_colliding == 0)
                        ContactListener.BeginContact(constraint);

                    else if (now_colliding == 0 && was_colliding > 0)
                        ContactListener.EndContact(constraint);
                }

            }
        }

        public void RenderContacts(Render render)
        {

            foreach (var contact in ContactList)
            {
                Manifold m = contact.manifold;

                if ((contact.Flags & ContactFlags.eColliding) == 0)
                {

                    continue;
                }

                for (int j = 0; j < m.contactCount; ++j)
                {
                    Contact c = m.contacts[j];
                    double blue = (double)(255 - c.warmStarted) / 255.0f;
                    double red = 1.0f - blue;
                    render.SetScale(10.0f, 10.0f, 10.0f);
                    render.SetPenColor(red, blue, blue);
                    render.SetPenPosition(c.position.x, c.position.y, c.position.z);
                    render.Point();

                    if (m.A.body.IsAwake())
                        render.SetPenColor(1.0f, 1.0f, 1.0f);
                    else
                        render.SetPenColor(0.2f, 0.2f, 0.2f);

                    render.SetPenPosition(c.position.x, c.position.y, c.position.z);
                    render.Line(
                        c.position.x + m.normal.x * 0.5f,
                        c.position.y + m.normal.y * 0.5f,
                        c.position.z + m.normal.z * 0.5f
                        );
                }
            }





        }

        internal List<ContactConstraint> ContactList;
        internal BroadPhase Broadphase;
        internal ContactListener ContactListener;
    }
}