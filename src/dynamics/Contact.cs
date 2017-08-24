//--------------------------------------------------------------------------------------------------
/**
    Qu3e Physics Engine v1.01 - Unofficial C# Version with modifications

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

    // in stands for "incoming"
    // out stands for "outgoing"
    // I stands for "incident"
    // R stands for "reference"
    // See D. Gregorius GDC 2015 on creating contacts for more details
    // Each feature pair is used to cache solutions from one physics tick to another. This is
    // called warmstarting, and lets boxes stack and stay stable. Feature pairs identify points
    // of contact over multiple physics ticks. Each feature pair is the junction of an incoming
    // feature and an outgoing feature, usually a result of clipping routines. The exact info
    // stored in the feature pair can be arbitrary as long as the result is a unique ID for a
    // given intersecting configuration.
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
        // World coordinate of contact
        public Vec3 position;
        // Depth of penetration from collision 
        public double penetration;
        // Accumulated normal impulse
        public double normalImpulse;
        // Accumulated friction impulse
        public double[] tangentImpulse = new double[2];
        // Features on A and B for this contact
        public FeaturePair fp;
        // Used for debug rendering
        public byte warmStarted;               
    };


    public class Manifold
    {
        public void SetPair(Shape a, Shape b)
        {
            A = a;
            B = b;

            sensor = A.sensor || B.sensor;
        }

        public Shape A;
        public Shape B;

        public Vec3 normal;                // From A to B
        public Vec3[] tangentVectors = new Vec3[2]; // Tangent vectors
        public Contact[] contacts = new Contact[8];
        public int contactCount;
        
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

            Collide.ComputeCollision(manifold, A, B);

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

        public Shape A, B;
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
