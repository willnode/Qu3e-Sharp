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
    public class ContactState
    {
        public Vec3 ra;                    // Vector from C.O.M to contact position
        public Vec3 rb;                    // Vector from C.O.M to contact position
        public double penetration;         // Depth of penetration from collision
        public double normalImpulse;           // Accumulated normal impulse
        public double tangentImpulse;   // Accumulated friction impulse
        public double bitangentImpulse;   // Accumulated friction impulse
        public double bias;                    // Restitution + baumgarte
        public double normalMass;              // Normal constraint mass
        public double tangentMass;      // Tangent constraint mass
        public double bitangentMass;      // Tangent constraint mass

        // ContactState Heap/Memory Pooling for c#
        static Stack<ContactState> heap = new Stack<ContactState>();
        static public ContactState Allocate() { return heap.Count == 0 ? new ContactState() : heap.Pop(); }
        static public void Free(ContactState instance) { heap.Push(instance); }
    };

    public class ContactConstraintState
    {
        public ContactState[] contacts = new ContactState[8];
        public int contactCount;
        public Vec3 tangentVectors; // Tangent vectors
        public Vec3 bitangentVectors; // Tangent vectors
        public Vec3 normal;                // From A to B
        public Vec3 centerA;
        public Vec3 centerB;
        public Mat3 iA;
        public Mat3 iB;
        public double mA;
        public double mB;
        public double restitution;
        public double friction;
        public int indexA;
        public int indexB;

        // ContactConstaintState Heap/Memory Pooling for c#
        static Stack<ContactConstraintState> heap = new Stack<ContactConstraintState>();
        static public ContactConstraintState Allocate() { return heap.Count == 0 ? new ContactConstraintState() : heap.Pop(); }
        static public void Free(ContactConstraintState instance) { heap.Push(instance); }
    };

    public class ContactSolver
    {
        public void Initialize(Island island)
        {
            Island = island;
           // ContactCount = island.ContactCount;
            Contacts = island.ContactStates;
            Velocities = Island.Velocities;
            EnableFriction = island.EnableFriction;
        }
        public void ShutDown()
        {
            for (int i = 0; i < Contacts.Count; ++i)
            {
                ContactConstraintState c = Contacts[i];
                ContactConstraint cc = Island.Contacts[i];

                for (int j = 0; j < c.contactCount; ++j)
                {
                    Contact oc = cc.manifold.contacts[j];
                    ContactState cs = c.contacts[j];
                    oc.normalImpulse = cs.normalImpulse;
                    oc.tangentImpulse = cs.tangentImpulse;
                    oc.bitangentImpulse = cs.bitangentImpulse;
                }
            }
        }

        static double Invert (double i) {return i == 0 ? 0 : 1 / i; }
        static double Clamp (double a, double b, double t) { if (t < a) return a; if (t > b) return b; return t; }
        public void PreSolve(double dt)
        {
            for (int i = 0; i < Contacts.Count; ++i)
            {
                ContactConstraintState cs = Contacts[i];

                Vec3 vA = Velocities[cs.indexA].v;
                Vec3 wA = Velocities[cs.indexA].w;
                Vec3 vB = Velocities[cs.indexB].v;
                Vec3 wB = Velocities[cs.indexB].w;

                for (int j = 0; j < cs.contactCount; ++j)
                {
                    ContactState c = cs.contacts[j];

                    // Precalculate JM^-1JT for contact and friction constraints
                    Vec3 raCn = Vec3.Cross(c.ra, cs.normal);
                    Vec3 rbCn = Vec3.Cross(c.rb, cs.normal);
                    double nm = cs.mA + cs.mB;

                    nm += Vec3.Dot(raCn, cs.iA * raCn) + Vec3.Dot(rbCn, cs.iB * rbCn);
                    c.normalMass = Invert(nm);


                    {
                        Vec3 raCt = Vec3.Cross(cs.tangentVectors, c.ra);
                        Vec3 rbCt = Vec3.Cross(cs.tangentVectors, c.rb);
                        var tm = nm + Vec3.Dot(raCt, cs.iA * raCt) + Vec3.Dot(rbCt, cs.iB * rbCt);
                        c.tangentMass = Invert(tm);
                    }
                    {
                        Vec3 raCt = Vec3.Cross(cs.bitangentVectors, c.ra);
                        Vec3 rbCt = Vec3.Cross(cs.bitangentVectors, c.rb);
                        var tm = nm + Vec3.Dot(raCt, cs.iA * raCt) + Vec3.Dot(rbCt, cs.iB * rbCt);
                        c.bitangentMass = Invert(tm);
                    }

                    // Precalculate bias factor
                    c.bias = -Q3_BAUMGARTE * (1 / dt) * Math.Min(0, c.penetration + Q3_PENETRATION_SLOP);

                    // Warm start contact
                    Vec3 P = cs.normal * c.normalImpulse;

                    if (EnableFriction)
                    {
                        P += cs.tangentVectors * c.tangentImpulse;
                        P += cs.bitangentVectors * c.bitangentImpulse;
                    }

                    vA -= P * cs.mA;
                    wA -= cs.iA * Vec3.Cross(c.ra, P);

                    vB += P * cs.mB;
                    wB += cs.iB * Vec3.Cross(c.rb, P);

                    // Add in restitution bias
                    double dv = Vec3.Dot(vB + Vec3.Cross(wB, c.rb) - vA - Vec3.Cross(wA, c.ra), cs.normal);

                    if (dv < -1)
                        c.bias += -(cs.restitution) * dv;
                }

                Velocities[cs.indexA] = new VelocityState { v = vA, w = wA };
                Velocities[cs.indexB] = new VelocityState { v = vB, w = wB };
            }
        }
        public void Solve()
        {
            for (int i = 0; i < Contacts.Count; ++i)
            {
                ContactConstraintState cs = Contacts[i];

                Vec3 vA = Velocities[cs.indexA].v;
                Vec3 wA = Velocities[cs.indexA].w;
                Vec3 vB = Velocities[cs.indexB].v;
                Vec3 wB = Velocities[cs.indexB].w;

                for (int j = 0; j < cs.contactCount; ++j)
                {
                    ContactState c = cs.contacts[j];

                    // relative velocity at contact
                    Vec3 dv = vB + Vec3.Cross(wB, c.rb) - vA - Vec3.Cross(wA, c.ra);

                    // Friction
                    if (EnableFriction)
                    {
                        
                        {
                            double lambda = -Vec3.Dot(dv, cs.tangentVectors) * c.tangentMass;

                            // Calculate frictional impulse
                            double maxLambda = cs.friction * c.normalImpulse;

                            // Clamp frictional impulse
                            double oldPT = c.tangentImpulse;
                            c.tangentImpulse = Clamp(-maxLambda, maxLambda, oldPT + lambda);
                            lambda = c.tangentImpulse - oldPT;

                            // Apply friction impulse
                            Vec3 impulse = cs.tangentVectors * lambda;
                            vA -= impulse * cs.mA;
                            wA -= cs.iA * Vec3.Cross(c.ra, impulse);

                            vB += impulse * cs.mB;
                            wB += cs.iB * Vec3.Cross(c.rb, impulse);
                        }
                        {
                            double lambda = -Vec3.Dot(dv, cs.bitangentVectors) * c.bitangentMass;

                            // Calculate frictional impulse
                            double maxLambda = cs.friction * c.normalImpulse;

                            // Clamp frictional impulse
                            double oldPT = c.bitangentImpulse;
                            c.bitangentImpulse = Clamp(-maxLambda, maxLambda, oldPT + lambda);
                            lambda = c.bitangentImpulse - oldPT;

                            // Apply friction impulse
                            Vec3 impulse = cs.bitangentVectors * lambda;
                            vA -= impulse * cs.mA;
                            wA -= cs.iA * Vec3.Cross(c.ra, impulse);

                            vB += impulse * cs.mB;
                            wB += cs.iB * Vec3.Cross(c.rb, impulse);
                        }
                    }

                    // Normal
                    {
                        dv = vB + Vec3.Cross(wB, c.rb) - vA - Vec3.Cross(wA, c.ra);

                        // Normal impulse
                        double vn = Vec3.Dot(dv, cs.normal);

                        // Factor in positional bias to calculate impulse scalar j
                        double lambda = c.normalMass * (-vn + c.bias);

                        // Clamp impulse
                        double tempPN = c.normalImpulse;
                        c.normalImpulse = Math.Max(tempPN + lambda, 0);
                        lambda = c.normalImpulse - tempPN;

                        // Apply impulse
                        Vec3 impulse = cs.normal * lambda;
                        vA -= impulse * cs.mA;
                        wA -= cs.iA * Vec3.Cross(c.ra, impulse);

                        vB += impulse * cs.mB;
                        wB += cs.iB * Vec3.Cross(c.rb, impulse);
                    }
                }

                Velocities[cs.indexA] = new VelocityState { v = vA, w = wA };
                Velocities[cs.indexB] = new VelocityState { v = vB, w = wB };
            }
        }

        public Island Island;
        public List<ContactConstraintState> Contacts;
        public List<VelocityState> Velocities;

        public bool EnableFriction;
    };

}
