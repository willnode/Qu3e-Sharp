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
    public class Capsule : Shape
    {
        // The capsule uses Y as the polar caps
        public double extent;
        public double radius;

        public override void ComputeAABB(Transform tx, out AABB aabb)
        {
            Transform world = Transform.Mul(tx, local);
            Vec3 axis = new Vec3(radius, extent + radius, radius);
            aabb = Transform.Mul(world, new AABB() { min = -axis, max = axis });
        }


        public override void ComputeMass(out MassData md)
        {
            // mass = density * ( volume sphere + volume cylinder )
            double mass = density * radius * radius * Math.PI * ((4 / 3.0 * radius) + (extent * 2));

            // Calculate inertia tensor
            // See www.randygaul.net/wp-content/uploads/2014/08/D.-Gregorius-Capsule-Inertia.pdf
            // The code uses over-simplified version of it

            double r = mass * ((21 / 20.0) * radius * radius + (6 / 4.0) * radius * extent + (4 / 3.0) * extent * extent);
            double h = 13 / 10 * mass * radius * radius;

            Mat3 I = Mat3.Diagonal(r, h, r);

            // Transform tensor to local space
            I = local.rotation * I * Mat3.Transpose(local.rotation);
            I += (Mat3.Identity * Vec3.Dot(local.position, local.position) - Mat3.OuterProduct(local.position, local.position)) * mass;

            md.center = local.position;
            md.inertia = I;
            md.mass = mass;
        }

        public override bool Raycast(Transform tx, ref RaycastData raycast)
        {
            throw new NotImplementedException();
        }

        public override void Render(Transform tx, bool awake, Render render)
        {
            throw new NotImplementedException();
        }

        public override bool TestPoint(Transform tx, Vec3 p)
        {
            Transform world = Transform.Mul(tx, local);
            Vec3 p0 = Transform.MulT(world, p);

            // if point is above/below cylinder
            if (Math.Abs(p0.y) >= extent)
                p0.y -= Sign(p0.y) * extent;
            else
                p0.y = 0;

            return Vec3.LengthSq(p0) <= radius * radius;
        }
    }
}
