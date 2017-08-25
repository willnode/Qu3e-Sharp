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
using static Qu3e.BodyType;
using static Qu3e.BodyFlags;
using System.Text;

namespace Qu3e
{

    public enum BodyType
    {
        eStaticBody,
        eDynamicBody,
        eKinematicBody
    }

    [Flags]
    public enum BodyFlags
    {
        eAwake = 0x001,
        eActive = 0x002,
        eAllowSleep = 0x004,
        eIsland = 0x010,
        eStatic = 0x020,
        eDynamic = 0x040,
        eKinematic = 0x080,
        eLockAxisX = 0x100,
        eLockAxisY = 0x200,
        eLockAxisZ = 0x400,
    }


    public class Body
    {

        // Adds a box to this body. Boxes are all defined in local space
        // of their owning body. Boxes cannot be defined relative to one
        // another. The body will recalculate its mass values. No contacts
        // will be created until the next Scene::Step( ) call.
        public Box AddBox(BoxDef def)
        {
            AABB aabb;
            Box box = new Box()
            {
                local = def.Tx,
                e = def.E,
                body = this,
                friction = def.Friction,
                restitution = def.Restitution,
                density = def.Density,
                sensor = def.Sensor,
            };
            
            Boxes.Add(box);
            box.ComputeAABB(Tx, out aabb);

            CalculateMassData();

            Scene.ContactManager.Broadphase.InsertBox(box, aabb);
            Scene.NewBox = true;

            return box;
        }

        // Removes this box from the body and broadphase. Forces the body
        // to recompute its mass if the body is dynamic. Frees the memory
        // pointed to by the box pointer.
        public void RemoveBox(Box box)
        {
            Assert(box != null);
            Assert(box.body == this);

            bool found = Boxes.Remove(box);

            // This shape was not connected to this body.
            Assert(found);

            // Remove all contacts associated with this shape
            foreach (var edge in ContactList)
            {
                ContactConstraint contact = edge.constraint;

                Box A = contact.A;
                Box B = contact.B;

                if (box == A || box == B)
                    Scene.ContactManager.RemoveContact(contact);

            }

            Scene.ContactManager.Broadphase.RemoveBox(box);

            CalculateMassData();

            // Scene.Heap.Free((void)box);
        }

        // Removes all boxes from this body and the broadphase.
        public void RemoveAllBoxes()
        {
            foreach (var box in Boxes)
            {
                Scene.ContactManager.Broadphase.RemoveBox(box);
            }

            Scene.ContactManager.RemoveContactsFromBody(this);

        }

        public void ApplyLinearForce(Vec3 force)
        {
            Force += force * Mass;

            SetToAwake();
        }
        public void ApplyForceAtWorldPoint(Vec3 force, Vec3 point)
        {
            Force += force * Mass;
            Torque += Vec3.Cross(point - WorldCenter, force);

            SetToAwake();
        }
        public void ApplyLinearImpulse(Vec3 impulse)
        {
            LinearVelocity += impulse * InvMass;

            SetToAwake();
        }
        public void ApplyLinearImpulseAtWorldPoint(Vec3 impulse, Vec3 point)
        {
            LinearVelocity += impulse * InvMass;
            AngularVelocity += InvInertiaWorld * Vec3.Cross(point - WorldCenter, impulse);

            SetToAwake();
        }
        public void ApplyTorque(Vec3 torque)
        {
            Torque += torque;
        }
        public void SetToAwake()
        {
            if ((Flags & eAwake) == 0)
            {
                Flags |= eAwake;
                SleepTime = 0;
            }
        }
        public void SetToSleep()
        {
            Flags &= ~eAwake;
            SleepTime = 0;
            Vec3.Identity(ref LinearVelocity);
            Vec3.Identity(ref AngularVelocity);
            Vec3.Identity(ref Force);
            Vec3.Identity(ref Torque);
        }
        public bool IsAwake()
        {
            return (Flags & eAwake) > 0 ? true : false;
        }

        public double GetGravityScale()
        {
            return GravityScale;
        }
        public void SetGravityScale(double scale)
        {
            GravityScale = scale;
        }
        public Vec3 GetLocalPoint(Vec3 p)
        {
            return Transform.MulT(Tx, p);
        }
        public Vec3 GetLocalVector(Vec3 v)
        {
            return Transform.MulT(Tx.rotation, v);
        }
        public Vec3 GetWorldPoint(Vec3 p)
        {
            return Transform.Mul(Tx, p);
        }
        public Vec3 GetWorldVector(Vec3 v)
        {
            return Transform.Mul(Tx.rotation, v);
        }
        public Vec3 GetLinearVelocity()
        {
            return LinearVelocity;
        }
        public Vec3 GetVelocityAtWorldPoint(Vec3 p)
        {
            Vec3 directionToPoint = p - WorldCenter;
            Vec3 relativeAngularVel = Vec3.Cross(AngularVelocity, directionToPoint);

            return LinearVelocity + relativeAngularVel;
        }
        public void SetLinearVelocity(Vec3 v)
        {
            // Velocity of static bodies cannot be adjusted
            if ((Flags & eStatic) > 0)
                Assert(false);

            if (Vec3.Dot(v, v) > 0)
            {
                SetToAwake();
            }

            LinearVelocity = v;
        }
        public Vec3 GetAngularVelocity()
        {
            return AngularVelocity;
        }
        public void SetAngularVelocity(Vec3 v)
        {
            // Velocity of static bodies cannot be adjusted
            if ((Flags & eStatic) > 0)
                Assert(false);

            if (Vec3.Dot(v, v) > 0)
            {
                SetToAwake();
            }

            AngularVelocity = v;
        }
        public bool CanCollide(Body other)
        {
            if (this == other)
                return false;

            // Every collision must have at least one dynamic body involved
            if ((Flags & eDynamic) == 0 && (other.Flags & eDynamic) == 0)
                return false;

            if ((Layers & other.Layers) == 0)
                return false;

            return true;
        }
        public Transform GetTransform()
        {
            return Tx;
        }
        public BodyFlags GetFlags()
        {
            return Flags;
        }
        public void SetLayers(int layers)
        {
            Layers = layers;
        }
        public int GetLayers()
        {
            return Layers;
        }
        public Quaternion GetQuaternion()
        {
            return Q;
        }
        public object GetUserData()
        {
            return UserData;
        }

        public void SetLinearDamping(double damping)
        {
            LinearDamping = damping;
        }
        public double GetLinearDamping(double damping)
        {
            return LinearDamping;
        }
        public void SetAngularDamping(double damping)
        {
            AngularDamping = damping;
        }
        public double GetAngularDamping(double damping)
        {
            return AngularDamping;
        }

        // Manipulating the transformation of a body manually will result in
        // non-physical behavior. Contacts are updated upon the next call to
        // Scene::Step( ). Parameters are in world space. All body types
        // can be updated.
        public void SetTransform(Vec3 position)
        {
            WorldCenter = position;

            SynchronizeProxies();
        }
        public void SetTransform(Vec3 position, Vec3 axis, double angle)
        {
            WorldCenter = position;
            Q.Set(axis, angle);
            Tx.rotation = Q.ToMat3();

            SynchronizeProxies();
        }

        // Used for debug rendering lines, triangles and basic lighting
        public void Render(Render render)
        {
            bool awake = IsAwake();
            
            foreach (var box in Boxes)
            {
                box.Render(Tx, awake, render);
            }
        }

        
        // Dump this rigid body and its shapes into a log file. The log can be
        // used as C++ code to re-create an initial scene setup.
        public void Dump(StringBuilder file, int index)
        {
            file.AppendFormat( "{{\n");
            file.AppendFormat( "\tq3BodyDef bd;\n");

            switch (Flags & (eStatic | eDynamic | eKinematic))
            {
                case eStatic:
                    file.AppendFormat( "\tbd.bodyType = BodyType( {0} );\n", eStaticBody);
                    break;

                case eDynamic:
                    file.AppendFormat( "\tbd.bodyType = BodyType( {0} );\n", eDynamicBody);
                    break;

                case eKinematic:
                    file.AppendFormat( "\tbd.bodyType = BodyType( {0} );\n", eKinematicBody);
                    break;
            }

            file.AppendFormat( "\tbd.position.Set( ( {0} ), ( {1} ), ( {2} ) );\n", Tx.position.x, Tx.position.y, Tx.position.z);
            Vec3 axis;
            double angle;
            Q.ToAxisAngle(out axis,out angle);
            file.AppendFormat( "\tbd.axis.Set( ( {0} ), ( {1} ), ( {2} ) );\n", axis.x, axis.y, axis.z);
            file.AppendFormat( "\tbd.angle = ( {0} );\n", angle);
            file.AppendFormat( "\tbd.linearVelocity.Set( ( {0} ), ( {1} ), ( {2} ) );\n", LinearVelocity.x, LinearVelocity.y, LinearVelocity.z);
            file.AppendFormat( "\tbd.angularVelocity.Set( ( {0} ), ( {1} ), ( {2} ) );\n", AngularVelocity.x, AngularVelocity.y, AngularVelocity.z);
            file.AppendFormat( "\tbd.gravityScale = ( {0} );\n", GravityScale);
            file.AppendFormat("\tbd.layers = {0} Layers);\n", Layers );
            file.AppendFormat( "\tbd.allowSleep = Bool( {0} );\n", Flags & eAllowSleep);
            file.AppendFormat( "\tbd.awake = Bool( {0} );\n", Flags & eAwake);
            file.AppendFormat( "\tbd.awake = Bool( {0} );\n", Flags & eAwake);
            file.AppendFormat( "\tbd.lockAxisX = Bool( {0} );\n", Flags & eLockAxisX);
            file.AppendFormat( "\tbd.lockAxisY = Bool( {0} );\n", Flags & eLockAxisY);
            file.AppendFormat( "\tbd.lockAxisZ = Bool( {0} );\n", Flags & eLockAxisZ);
            file.AppendFormat( "\tbodies[ {0} ] = scene.CreateBody( bd );\n\n", index);

            foreach (var box in Boxes)
            {
                file.AppendFormat( "\t{{\n");
                file.AppendFormat( "\t\tq3BoxDef sd;\n");
                file.AppendFormat( "\t\tsd.SetFriction( ( {0} ) );\n", box.friction);
                file.AppendFormat( "\t\tsd.SetRestitution( ( {0} ) );\n", box.restitution);
                file.AppendFormat( "\t\tsd.SetDensity( ( {0} ) );\n", box.density);
                var sensor = box.sensor;
                file.AppendFormat( "\t\tsd.SetSensor( Bool( {0} ) );\n", sensor);
                file.AppendFormat( "\t\tq3Transform boxTx;\n");
                Transform boxTx = box.local;
                Vec3 xAxis = boxTx.rotation.ex;
                Vec3 yAxis = boxTx.rotation.ey;
                Vec3 zAxis = boxTx.rotation.ez;
                file.AppendFormat( "\t\tq3Vec3 XAxis( ( {0} ), ( {1} ), ( {2} ) );\n", xAxis.x, xAxis.y, xAxis.z);
                file.AppendFormat( "\t\tq3Vec3 YAxis( ( {0} ), ( {1} ), ( {2} ) );\n", yAxis.x, yAxis.y, yAxis.z);
                file.AppendFormat( "\t\tq3Vec3 ZAxis( ( {0} ), ( {1} ), ( {2} ) );\n", zAxis.x, zAxis.y, zAxis.z);
                file.AppendFormat( "\t\tboxTx.rotation.SetRows( xAxis, yAxis, zAxis );\n");
                file.AppendFormat( "\t\tboxTx.position.Set( ( {0} ), ( {1} ), ( {2} ) );\n", boxTx.position.x, boxTx.position.y, boxTx.position.z);
                file.AppendFormat( "\t\tsd.Set( boxTx, Vec3( ( {0} ), ( {1} ), ( {2} ) ) );\n", box.e.x * 2.0f, box.e.y * 2.0f, box.e.z * 2.0f);
                file.AppendFormat( "\t\tbodies[ {0} ].AddBox( sd );\n", index);
                file.AppendFormat( "\t}}\n");
            }

            file.AppendFormat( "}}\n\n");
        }

        public double GetMass()
        {
            return Mass;
        }
        public double GetInvMass()
        {
            return InvMass;
        }

        internal Mat3 InvInertiaModel;
        internal Mat3 InvInertiaWorld;
        internal double Mass;
        internal double InvMass;
        internal Vec3 LinearVelocity;
        internal Vec3 AngularVelocity;
        internal Vec3 Force;
        internal Vec3 Torque;
        internal Transform Tx;
        internal Quaternion Q;
        internal Vec3 LocalCenter;
        internal Vec3 WorldCenter;
        internal double SleepTime;
        internal double GravityScale;
        internal int Layers;
        internal BodyFlags Flags;

        internal List<Box> Boxes;
        internal object UserData;
        internal Scene Scene;
        //internal Body Next;
        //internal Body Prev;
        internal int IslandIndex;

        internal double LinearDamping;
        internal double AngularDamping;

        internal List<ContactEdge> ContactList;

        public Body(BodyDef def, Scene scene)
        {
            LinearVelocity = def.linearVelocity;
            AngularVelocity = def.angularVelocity;
            Vec3.Identity(ref Force);
            Vec3.Identity(ref Torque);
            Q.Set(Vec3.Normalize(def.axis), def.angle);
            Tx.rotation = Q.ToMat3();
            Tx.position = def.position;
            SleepTime = 0;
            GravityScale = def.gravityScale;
            Layers = def.layers;
            UserData = def.userData;
            Scene = scene;
            Flags = 0;
            LinearDamping = def.linearDamping;
            AngularDamping = def.angularDamping;

            if (def.bodyType == eDynamicBody)
                Flags |= eDynamic;

            else
            {
                if (def.bodyType == eStaticBody)
                {
                    Flags |= eStatic;
                    Vec3.Identity(ref LinearVelocity);
                    Vec3.Identity(ref AngularVelocity);
                    Vec3.Identity(ref Force);
                    Vec3.Identity(ref Torque);
                }

                else if (def.bodyType == eKinematicBody)
                    Flags |= eKinematic;
            }

            if (def.allowSleep)
                Flags |= eAllowSleep;

            if (def.awake)
                Flags |= eAwake;

            if (def.active)
                Flags |= eActive;

            if (def.lockAxisX)
                Flags |= eLockAxisX;

            if (def.lockAxisY)
                Flags |= eLockAxisY;

            if (def.lockAxisZ)
                Flags |= eLockAxisZ;

            Boxes = new List<Box>();
            ContactList = new List<ContactEdge>();
        }

        void CalculateMassData()
        {
            Mat3 inertia = Mat3.Diagonal(0);
            InvInertiaModel = Mat3.Diagonal(0);
            InvInertiaWorld = Mat3.Diagonal(0);
            InvMass = 0;
            Mass = 0;
            double mass = 0;

            if ((Flags & eStatic) > 0 || (Flags & eKinematic) > 0)
            {
                Vec3.Identity(ref LocalCenter);
                WorldCenter = Tx.position;
                return;
            }

            Vec3 lc = new Vec3();
            Vec3.Identity(ref lc);

            foreach (var box in Boxes)
            {
                if (box.density == 0)
                    continue;

                MassData md;
                box.ComputeMass(out md);
                mass += md.mass;
                inertia += md.inertia;
                lc += md.center * md.mass;
            }

            if (mass > 0)
            {
                Mass = mass;
                InvMass = 1 / mass;
                lc *= InvMass;
                inertia -= (Mat3.Identity * Vec3.Dot(lc, lc) - Mat3.OuterProduct(lc, lc)) * mass;
                InvInertiaModel = Mat3.Inverse(inertia);

                if ((Flags & eLockAxisX) > 0)
                    Vec3.Identity(ref InvInertiaModel.ex);

                if ((Flags & eLockAxisY) > 0)
                    Vec3.Identity(ref InvInertiaModel.ey);

                if ((Flags & eLockAxisZ) > 0)
                    Vec3.Identity(ref InvInertiaModel.ez);
            }
            else
            {
                // Force all dynamic bodies to have some mass
                InvMass = 1;
                InvInertiaModel = Mat3.Diagonal(0);
                InvInertiaWorld = Mat3.Diagonal(0);
            }

            LocalCenter = lc;
            WorldCenter = Transform.Mul(Tx, lc);
        }
        internal void SynchronizeProxies()
        {
            BroadPhase broadphase = Scene.ContactManager.Broadphase;

            Tx.position = WorldCenter - Transform.Mul(Tx.rotation, LocalCenter);

            AABB aabb;
            Transform tx = Tx;

            foreach (var box in Boxes)
            {
                box.ComputeAABB(tx, out aabb);
                broadphase.Update(box.broadPhaseIndex, aabb);
            }
        }
    }

    public class BodyDef
    {
        public BodyDef()
        {
            // Set all initial positions/velocties to zero
            Vec3.Identity(ref axis);
            angle = 0;
            Vec3.Identity(ref position);
            Vec3.Identity(ref linearVelocity);
            Vec3.Identity(ref angularVelocity);

            // Usually a gravity scale of 1 is the best
            gravityScale = 1;

            // Common default values
            bodyType = eStaticBody;
            layers = 0x000000001;
            userData = null;
            allowSleep = true;
            awake = true;
            active = true;
            lockAxisX = false;
            lockAxisY = false;
            lockAxisZ = false;

            linearDamping = 0;
            angularDamping = 0.1;
        }

        public Vec3 axis;            // Initial world transformation.
        public double angle;              // Initial world transformation. Radians.
        public Vec3 position;        // Initial world transformation.
        public Vec3 linearVelocity;  // Initial linear velocity in world space.
        public Vec3 angularVelocity; // Initial angular velocity in world space.
        public double gravityScale;       // Convenient scale values for gravity x, y and z directions.
        public int layers;             // Bitmask of collision layers. Bodies matching at least one layer can collide.
        public object userData;         // Use to store application specific data.

        public double linearDamping;
        public double angularDamping;

        // Static, dynamic or kinematic. Dynamic bodies with zero mass are defaulted
        // to a mass of 1. Static bodies never move or integrate, and are very CPU
        // efficient. Static bodies have infinite mass. Kinematic bodies have
        // infinite mass, but *do* integrate and move around. Kinematic bodies do not
        // resolve any collisions.
        public BodyType bodyType;

        public bool allowSleep;    // Sleeping lets a body assume a non-moving state. Greatly reduces CPU usage.
        public bool awake;         // Initial sleep state. True means awake.
        public bool active;        // A body can start out inactive and just sits in memory.
        public bool lockAxisX;     // Locked rotation on the x axis.
        public bool lockAxisY;     // Locked rotation on the y axis.
        public bool lockAxisZ;     // Locked rotation on the z axis.
    };

}
