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

namespace Qu3e
{

    public class Box : Shape
    {

        public Vec3 extent;

        public override void ComputeAABB(Transform tx, out AABB aabb)
        {
            Transform world = Transform.Mul(tx, local);
            aabb = Transform.Mul(world, new AABB() { min = -extent, max = extent });
        }

        public override bool TestPoint(Transform tx, Vec3 p)
        {
            Transform world = Transform.Mul(tx, local);
            Vec3 p0 = Transform.MulT(world, p);

            for (int i = 0; i < 3; ++i)
            {
                if (Math.Abs(p0[i]) > extent[i]) return false;
            }
            return true;
        }

        public override bool Raycast(Transform tx, ref RaycastData raycast)
        {
            Transform world = Transform.Mul(tx, local);
            Vec3 d = Transform.MulT(world.rotation, raycast.dir);
            Vec3 p = Transform.MulT(world, raycast.start);
            double tmin = 0, tmax = raycast.t;

            // t = (e[ i ] - p.[ i ]) / d[ i ]
            double t0, t1;
            Vec3 n0 = new Vec3();

            for (int i = 0; i < 3; ++i)
            {
                // Check for ray parallel to and outside of AABB
                if (Math.Abs(d[i]) < 1e-8)
                {
                    // Detect separating axes
                    if (Math.Abs(p[i]) > extent[i])  return false; 
                }

                else
                {
                    double d0 = 1 / d[i];
                    double s = Math.Sign(d[i]);
                    double ei = extent[i] * s;
                    Vec3 n = new Vec3();
                    n[i] = -s;

                    t0 = -(ei + p[i]) * d0;
                    t1 = (ei - p[i]) * d0;

                    if (t0 > tmin)
                    {
                        n0 = n;
                        tmin = t0;
                    }

                    tmax = Math.Min(tmax, t1);

                    if (tmin > tmax) return false;
                    
                }
            }

            raycast.normal = Transform.Mul(world.rotation, n0);
            raycast.toi = tmin;

            return true;
        }


        public override void ComputeMass(out MassData md)
        {
            // Calculate inertia tensor
            double mass = 8 * extent.x * extent.y * extent.z * density;
            Vec3 e2 = 4 * Vec3.Mul(extent, extent);
            Vec3 e3 = new Vec3(e2.y + e2.z, e2.x + e2.z, e2.x + e2.y);
            Vec3 e4 = 1 / 12.0 * mass * e3;
            Mat3 I = Mat3.Diagonal(e4);

            // Transform tensor to local space
            I = local.rotation * I * Mat3.Transpose(local.rotation);
            I += (Mat3.Identity * Vec3.Dot(local.position, local.position) - Mat3.OuterProduct(local.position, local.position)) * mass;

            md.center = local.position;
            md.inertia = I;
            md.mass = mass;
        }


        //--------------------------------------------------------------------------------------------------

       internal static Vec3[] kBoxVertices = new Vec3[8]{
                        new Vec3( -1, -1, -1 ),
                        new  Vec3( -1, -1,  1 ),
                        new  Vec3( -1,  1, -1 ),
                        new  Vec3( -1,  1,  1 ),
                        new  Vec3(  1, -1, -1 ),
                        new  Vec3(  1, -1,  1 ),
                        new  Vec3(  1,  1, -1 ),
                        new  Vec3(  1,  1,  1 )
                        };

        internal static int[] kBoxIndices = new int[]
        {
            0, 6, 4, 0, 2, 6,
            0, 3, 2, 0, 1, 3,
            2, 7, 6, 2, 3, 7,
            4, 6, 7, 4, 7, 5,
            0, 4, 5, 0, 5, 1,
            1, 5, 7, 1, 7, 3
        };

        public override void Render(Transform tx, bool awake, Render render)
        {
            Transform world = Transform.Mul(tx, local);
            
            for (int i = 0; i < 36; i += 3)
            {
                Vec3 a = Transform.Mul(world, Vec3.Mul(extent, kBoxVertices[kBoxIndices[i]]));
                Vec3 b = Transform.Mul(world, Vec3.Mul(extent, kBoxVertices[kBoxIndices[i + 1]]));
                Vec3 c = Transform.Mul(world, Vec3.Mul(extent, kBoxVertices[kBoxIndices[i + 2]]));

                Vec3 n = Vec3.Normalize(Vec3.Cross(b - a, c - a));

                render.SetPenColor( 0.2, 0.4, 0.7, 0.5 );
                render.SetPenPosition( a.x, a.y, a.z );
                render.Line( b.x, b.y, b.z );
                render.Line( c.x, c.y, c.z );
                render.Line( a.x, a.y, a.z );

                render.SetTriNormal(n.x, n.y, n.z);
                render.Triangle(a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
            }
        }
    }


}
