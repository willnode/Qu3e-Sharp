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
    public struct MassData
    {
        public Mat3 inertia;
        public Vec3 center;
        public double mass;
    }
    public class Box
    {
        public Transform local;
        public Vec3 e; // extent, as in the extent of each OBB axis

        //public Box next;
        public Body body;
        public double friction;
        public double restitution;
        public double density;
        public int broadPhaseIndex;
        public object userData;
        public bool sensor;

        public void SetUserdata(object data)
        {
            userData = data;
        }
        public object GetUserdata()
        {
            return userData;
        }
        public void SetSensor(bool isSensor)
        {
            sensor = isSensor;
        }

        public bool TestPoint(Transform tx, Vec3 p)
        {
            Transform world = Transform.Mul(tx, local);
            Vec3 p0 = Transform.MulT(world, p);

            for (int i = 0; i < 3; ++i)
            {
                double d = p0[i];
                double ei = e[i];

                if (d > ei || d < -ei)
                {
                    return false;
                }
            }

            return true;
        }
        public bool Raycast(Transform tx, RaycastData raycast)
        {
            Transform world = Transform.Mul(tx, local);
            Vec3 d = Transform.MulT(world.rotation, raycast.dir);
            Vec3 p = Transform.MulT(world, raycast.start);
            double epsilon = 1e-8;
            double tmin = 0;
            double tmax = raycast.t;

            // t = (e[ i ] - p.[ i ]) / d[ i ]
            double t0;
            double t1;
            Vec3 n0 = new Vec3();

            for (int i = 0; i < 3; ++i)
            {
                // Check for ray parallel to and outside of AABB
                if (Math.Abs(d[i]) < epsilon)
                {
                    // Detect separating axes
                    if (p[i] < -e[i] || p[i] > e[i])
                    {
                        return false;
                    }
                }

                else
                {
                    double d0 = 1 / d[i];
                    double s = Math.Sign(d[i]);
                    double ei = e[i] * s;
                    Vec3 n = new Vec3(0, 0, 0);
                    n[i] = -s;

                    t0 = -(ei + p[i]) * d0;
                    t1 = (ei - p[i]) * d0;

                    if (t0 > tmin)
                    {
                        n0 = n;
                        tmin = t0;
                    }

                    tmax = Math.Min(tmax, t1);

                    if (tmin > tmax)
                    {
                        return false;
                    }
                }
            }

            raycast.normal = Transform.Mul(world.rotation, n0);
            raycast.toi = tmin;

            return true;
        }

        static Vec3[] kBoxVertices = new Vec3[8]{
                        new Vec3( -1, -1, -1 ),
                        new  Vec3( -1, -1,  1 ),
                        new  Vec3( -1,  1, -1 ),
                        new  Vec3( -1,  1,  1 ),
                        new  Vec3(  1, -1, -1 ),
                        new  Vec3(  1, -1,  1 ),
                        new  Vec3(  1,  1, -1 ),
                        new  Vec3(  1,  1,  1 )
                        };

        public void ComputeAABB(Transform tx, out AABB aabb)
        {
            Transform world = Transform.Mul(tx, local);

            Vec3 min = new Vec3(double.MaxValue, double.MaxValue, double.MaxValue);
            Vec3 max = new Vec3(-double.MaxValue, -double.MaxValue, -double.MaxValue);

            for (int i = 0; i < 8; ++i)
            {
                var v = Transform.Mul(world, Vec3.Mul(kBoxVertices[i], e));
                min = Vec3.Min(min, v);
                max = Vec3.Max(max, v);
            }

            aabb.min = min;
            aabb.max = max;
        }

        public void ComputeMass(out MassData md)
        {
            // Calculate inertia tensor
            double ex2 = 4 * e.x * e.x;
            double ey2 = 4 * e.y * e.y;
            double ez2 = 4 * e.z * e.z;
            double mass = 8 * e.x * e.y * e.z * density;
            double x = 1 / 12.0 * mass * (ey2 + ez2);
            double y = 1 / 12.0 * mass * (ex2 + ez2);
            double z = 1 / 12.0 * mass * (ex2 + ey2);
            Mat3 I = Mat3.Diagonal(x, y, z);

            // Transform tensor to local space
            I = local.rotation * I * Mat3.Transpose(local.rotation);
            I += (Mat3.Identity * Vec3.Dot(local.position, local.position) - Mat3.OuterProduct(local.position, local.position)) * mass;

            md.center = local.position;
            md.inertia = I;
            md.mass = mass;
        }


        //--------------------------------------------------------------------------------------------------
        static int[] kBoxIndices = {
                1 - 1, 7 - 1, 5 - 1,
                1 - 1, 3 - 1, 7 - 1,
                1 - 1, 4 - 1, 3 - 1,
                1 - 1, 2 - 1, 4 - 1,
                3 - 1, 8 - 1, 7 - 1,
                3 - 1, 4 - 1, 8 - 1,
                5 - 1, 7 - 1, 8 - 1,
                5 - 1, 8 - 1, 6 - 1,
                1 - 1, 5 - 1, 6 - 1,
                1 - 1, 6 - 1, 2 - 1,
                2 - 1, 6 - 1, 8 - 1,
                2 - 1, 8 - 1, 4 - 1
            };

        public void Render(Transform tx, bool awake, Render render)
        {
            Transform world = Transform.Mul(tx, local);

            Vec3[] vertices = new Vec3[8]{
                   new Vec3( -e.x, -e.y, -e.z ),
                   new  Vec3( -e.x, -e.y,  e.z ),
                   new  Vec3( -e.x,  e.y, -e.z ),
                  new   Vec3( -e.x,  e.y,  e.z ),
                  new   Vec3(  e.x, -e.y, -e.z ),
            new         Vec3(  e.x, -e.y,  e.z ),
                new     Vec3(  e.x,  e.y, -e.z ),
                    new Vec3(  e.x,  e.y,  e.z )
                };

            for (int i = 0; i < 36; i += 3)
            {
                Vec3 a = Transform.Mul(world, vertices[kBoxIndices[i]]);
                Vec3 b = Transform.Mul(world, vertices[kBoxIndices[i + 1]]);
                Vec3 c = Transform.Mul(world, vertices[kBoxIndices[i + 2]]);

                Vec3 n = Vec3.Normalize(Vec3.Cross(b - a, c - a));

                //render->SetPenColor( 0.2f, 0.4f, 0.7f, 0.5f );
                //render->SetPenPosition( a.x, a.y, a.z );
                //render->Line( b.x, b.y, b.z );
                //render->Line( c.x, c.y, c.z );
                //render->Line( a.x, a.y, a.z );

                render.SetTriNormal(n.x, n.y, n.z);
                render.Triangle(a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
            }
        }
    }

    public class BoxDef
    {

        public BoxDef()
        {
            // Common default values
            Friction = 0.4;
            Restitution = 0.2;
            Density = 1.0;
            Sensor = false;
            Tx = Transform.Identity;
            E = new Vec3(0.5, 0.5, 0.5);
        }

        public void Set(Transform tx, Vec3 extents)
        {
            Tx = tx;
            E = extents * (0.5);
        }

        public void SetFriction(double friction)
        {
            Friction = friction;
        }
        public void SetRestitution(double restitution)
        {
            Restitution = restitution;
        }
        public void SetDensity(double density)
        {
            Density = density;
        }
        public void SetSensor(bool sensor)
        {
            Sensor = sensor;
        }

        internal Transform Tx;
        internal Vec3 E;

        internal double Friction;
        internal double Restitution;
        internal double Density;
        internal bool Sensor;
    };



}
