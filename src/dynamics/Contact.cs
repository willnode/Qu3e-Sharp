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
using static Qu3e.ContactFlags;
using System.Runtime.InteropServices;

namespace Qu3e
{
    // A shortcut to union in cpp
    [StructLayout(LayoutKind.Explicit)]
    public struct FeaturePair
    {
        [FieldOffset(0)]
        public byte inR;
        [FieldOffset(1)]
        public byte outR;
        [FieldOffset(2)]
        public byte inI;
        [FieldOffset(3)]
        public byte outI;
        [FieldOffset(0)]
        public int key;

    }

    public class Contact
    {
        public Vec3 position;          // World coordinate of contact
        public double penetration;         // Depth of penetration from collision
        public double normalImpulse;           // Accumulated normal impulse
        public double tangentImpulse;   // Accumulated friction impulse
        public double bitangentImpulse;   // Accumulated friction impulse
        public double bias;                    // Restitution + baumgarte
        public double normalMass;              // Normal constraint mass
        public double tangentMass;      // Tangent constraint mass
        public double bitangentMass;      // Tangent constraint mass
        public FeaturePair fp;         // Features on A and B for this contact
        public byte warmStarted;               // Used for debug rendering
    };


    public class Manifold
    {
        public void SetPair(Box a, Box b)
        {
            A = a;
            B = b;

            sensor = A.sensor || B.sensor;
        }

        public Box A;
        public Box B;

        public Vec3 normal;                // From A to B
        public Vec3 tangentVectors; // Tangent vectors
        public Vec3 bitangentVectors; // Tangent vectors
        public Contact[] contacts = new Contact[8];
        public int contactCount;

        public Manifold next;
        public Manifold prev;

        public bool sensor;

        public Manifold ()
        {
            for (int i = 0; i < 8; i++)
            {
                contacts[i] = new Contact();
            }
        }
    };

    public class ContactEdge
    {
        public Body other;
        public ContactConstraint constraint;
    };

    public class ContactConstraint
    {
        public void SolveCollision()
        {
            manifold.contactCount = 0;

            Collide.BoxtoBox(manifold, A, B);

            if (manifold.contactCount > 0)
            {
                if ((Flags & eColliding) > 0)
                    Flags |= eWasColliding;

                else
                    Flags |= eColliding;
            }

            else
            {
                if ((Flags & eColliding)>0)
                {
                    Flags &= ~eColliding;
                    Flags |= eWasColliding;
                }

                else
                    Flags &= ~eWasColliding;
            }
        }

        public Box A, B;
        public Body bodyA, bodyB;

        public ContactEdge edgeA;
        public ContactEdge edgeB;

        public double friction;
        public double restitution;

        public Manifold manifold;

        public ContactFlags Flags;


        public ContactConstraint()
        {
            manifold = new Manifold();
            edgeA = new ContactEdge();
            edgeB = new ContactEdge();

        }
    }



    [Flags]
    public enum ContactFlags
    {
        eColliding = 0x00000001, // Set when contact collides during a step
        eWasColliding = 0x00000002, // Set when two objects stop colliding
        eIsland = 0x00000004, // For internal marking during island forming
    };


}
