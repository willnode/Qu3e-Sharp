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
    public struct VelocityState
    {
        public Vec3 w;
        public Vec3 v;
    };

    public class Island
    {
        public void Solve()
        {
            // Apply gravity
            // Integrate velocities and create state buffers, calculate world inertia
            for (int i = 0; i < Bodies.Count; ++i)
            {
                Body body = Bodies[i];
                VelocityState v = Velocities[i];

                if ((body.Flags & BodyFlags.eDynamic) > 0)
                {
                    body.ApplyLinearForce(Gravity * body.GravityScale);

                    // Calculate world space intertia tensor
                    Mat3 r = body.Tx.rotation;
                    body.InvInertiaWorld = r * body.InvInertiaModel * Mat3.Transpose(r);

                    // Integrate velocity
                    body.LinearVelocity += (body.Force * body.InvMass) * Dt;
                    body.AngularVelocity += (body.InvInertiaWorld * body.Torque) * Dt;

                    // From Box2D!
                    // Apply damping.
                    // ODE: dv/dt + c * v = 0
                    // Solution: v(t) = v0 * exp(-c * t)
                    // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
                    // v2 = exp(-c * dt) * v1
                    // Pade approximation:
                    // v2 = v1 * 1 / (1 + c * dt)
                    body.LinearVelocity *= 1 / (1 + Dt * body.LinearDamping);
                    body.AngularVelocity *= 1 / (1 + Dt * body.AngularDamping);
                }

                Velocities[i] = new VelocityState { v = body.LinearVelocity, w = body.AngularVelocity };
            }

            // Create contact solver, pass in state buffers, create buffers for contacts
            // Initialize velocity constraint for normal + friction and warm start
            ContactSolver.Initialize(this);
            ContactSolver.PreSolve(Dt);

            // Solve contacts
            for (int i = 0; i < Iterations; ++i)
                ContactSolver.Solve();

            ContactSolver.ShutDown();

            // Copy back state buffers
            // Integrate positions
            for (int i = 0; i < Bodies.Count; ++i)
            {
                Body body = Bodies[i];
                VelocityState v = Velocities[i];

                if ((body.Flags & BodyFlags.eStatic) > 0)
                    continue;

                body.LinearVelocity = v.v;
                body.AngularVelocity = v.w;

                // Integrate position
                body.WorldCenter += body.LinearVelocity * Dt;
                body.Q.Integrate(body.AngularVelocity, Dt);
                body.Q = Quaternion.Normalize(body.Q);
                body.Tx.rotation = body.Q.ToMat3();
            }

            if (AllowSleep)
            {
                // Find minimum sleep time of the entire island
                double minSleepTime = double.MaxValue;
                for (int i = 0; i < Bodies.Count; ++i)
                {
                    Body body = Bodies[i];

                    if ((body.Flags & BodyFlags.eStatic) > 0)
                        continue;

                    double sqrLinVel = Vec3.Dot(body.LinearVelocity, body.LinearVelocity);
                    double cbAngVel = Vec3.Dot(body.AngularVelocity, body.AngularVelocity);
                    double linTol = Q3_SLEEP_LINEAR;
                    double angTol = Q3_SLEEP_ANGULAR;

                    if (sqrLinVel > linTol || cbAngVel > angTol)
                    {
                        minSleepTime = 0;
                        body.SleepTime = 0;
                    }

                    else
                    {
                        body.SleepTime += Dt;
                        minSleepTime = Math.Min(minSleepTime, body.SleepTime);
                    }
                }

                // Put entire island to sleep so long as the minimum found sleep time
                // is below the threshold. If the minimum sleep time reaches below the
                // sleeping threshold, the entire island will be reformed next step
                // and sleep test will be tried again.
                if (minSleepTime > Q3_SLEEP_TIME)
                {
                    for (int i = 0; i < Bodies.Count; ++i)
                        Bodies[i].SetToSleep();
                }
            }
        }
        public void Add(Body body)
        {
            body.IslandIndex = Bodies.Count;
            Bodies.Add(body);
            Velocities.Add(new VelocityState());
        }
        public void Add(ContactConstraint contact)
        {
            Contacts.Add(contact);
            ContactStates.Add(ContactConstraintState.Allocate());
        }
        public void Initialize()
        {
            for (int i = 0; i < Contacts.Count; ++i)
            {
                ContactConstraint cc = Contacts[i];

                ContactConstraintState c = ContactStates[i];

                c.centerA = cc.bodyA.WorldCenter;
                c.centerB = cc.bodyB.WorldCenter;
                c.iA = cc.bodyA.InvInertiaWorld;
                c.iB = cc.bodyB.InvInertiaWorld;
                c.mA = cc.bodyA.InvMass;
                c.mB = cc.bodyB.InvMass;
                c.restitution = cc.restitution;
                c.friction = cc.friction;
                c.indexA = cc.bodyA.IslandIndex;
                c.indexB = cc.bodyB.IslandIndex;
                c.normal = cc.manifold.normal;
                c.tangentVectors = cc.manifold.tangentVectors;
                c.bitangentVectors = cc.manifold.bitangentVectors;
                c.contactCount = cc.manifold.contactCount;

                for (int j = 0; j < c.contactCount; ++j)
                {
                    ContactState s = c.contacts[j] = ContactState.Allocate();
                    Contact cp = cc.manifold.contacts[j];
                    s.ra = cp.position - c.centerA;
                    s.rb = cp.position - c.centerB;
                    s.penetration = cp.penetration;
                    s.normalImpulse = cp.normalImpulse;
                    s.tangentImpulse = cp.tangentImpulse;
                    s.bitangentImpulse = cp.bitangentImpulse;
                }
            }
        }

        public void Clear()
        {
            Bodies.Clear();
            Velocities.Clear();
            Contacts.Clear();

            foreach (var state in ContactStates)
            {
                for (int i = 0; i < state.contactCount; i++)
                    ContactState.Free(state.contacts[i]);
                ContactConstraintState.Free(state);
                // Array.Clear(state.contacts, 0, state.contactCount);
            }

            ContactStates.Clear();
        }

        public Island()
        {
            Bodies = new List<Body>();
            Velocities = new List<VelocityState>();
            Contacts = new List<ContactConstraint>();
            ContactStates = new List<ContactConstraintState>();
            ContactSolver = new ContactSolver();
        }

        public List<Body> Bodies;
        public List<VelocityState> Velocities;

        public List<ContactConstraint> Contacts;
        public List<ContactConstraintState> ContactStates;

        public ContactSolver ContactSolver;

        public double Dt;
        public Vec3 Gravity;
        public int Iterations;

        public bool AllowSleep;
        public bool EnableFriction;


    }
}
