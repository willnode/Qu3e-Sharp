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
    public class Sphere : Shape
    {
        public double radius;

        public override void ComputeAABB(Transform tx, out AABB aabb)
        {
            Transform world = Transform.Mul(tx, local);
            aabb.min = Vec3.Uniform(-radius) + world.position;
            aabb.max = Vec3.Uniform(radius) + world.position;
        }

        public override void ComputeMass(out MassData md)
        {
            // Calculate inertia tensor
            double mass = 4 / 3.0 * Math.PI * radius * radius * radius * density;
            double i = 2 / 5.0 * mass * radius * radius;
            Mat3 I = Mat3.Diagonal(i, i, i);

            // Transform tensor to local space
            I = local.rotation * I * Mat3.Transpose(local.rotation);
            I += (Mat3.Identity * Vec3.Dot(local.position, local.position) - Mat3.OuterProduct(local.position, local.position)) * mass;

            md.center = local.position;
            md.inertia = I;
            md.mass = mass;
        }

        public override bool Raycast(Transform tx, ref RaycastData raycast)
        {
            Transform world = Transform.Mul(tx, local);
            Vec3 d = Transform.MulT(world.rotation, raycast.dir);
            Vec3 p = Transform.MulT(world, raycast.start);
            double tmax = raycast.t;

            var tca = Vec3.Dot(p, d);
            // If this sphere behind the ray
            if (tca < 0) return false;

            var tcd = Vec3.Dot(p, p) - tca * tca;
            // If this ray completely miss it
            if (tcd > radius * radius) return false;

            // 100% chance of hit. Calculate now.
            var thc = Math.Sqrt(radius * radius - tcd);
            var toi = tca - thc;

            if (toi > tmax) return false;

            raycast.toi = toi;
            raycast.normal = Vec3.Normalize(p + d * toi);
            return true;
        }

        public override bool TestPoint(Transform tx, Vec3 p)
        {
            Transform world = Transform.Mul(tx, local);
            Vec3 p0 = Transform.MulT(world, p);

            return Vec3.Dot(p0, p0) <= radius * radius;
        }

        //--------------------------------------------------------------------------------------------------

        public override void Render(Transform tx, bool awake, Render render)
        {
            throw new NotImplementedException();
        }

    }
}
