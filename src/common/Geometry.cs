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
    //--------------------------------------------------------------------------------------------------
    // q3AABB
    //--------------------------------------------------------------------------------------------------
    public struct AABB
    {
        public Vec3 min;
        public Vec3 max;

        public AABB (Vec3 min, Vec3 max)
        {
            this.min = min;
            this.max = max;
        }
        
        public static bool AABBtoAABB(AABB a, AABB b)
        {
            if (a.max.x < b.min.x || a.min.x > b.max.x)
                return false;

            if (a.max.y < b.min.y || a.min.y > b.max.y)
                return false;

            if (a.max.z < b.min.z || a.min.z > b.max.z)
                return false;

            return true;
        }

        //--------------------------------------------------------------------------------------------------
        public bool Contains(AABB other)
        {
            return
                min.x <= other.min.x &&
                min.y <= other.min.y &&
                min.z <= other.min.z &&
                max.x >= other.max.x &&
                max.y >= other.max.y &&
                max.z >= other.max.z;
        }

        //--------------------------------------------------------------------------------------------------
        public bool Contains(Vec3 point)
        {
            return
                min.x <= point.x &&
                min.y <= point.y &&
                min.z <= point.z &&
                max.x >= point.x &&
                max.y >= point.y &&
                max.z >= point.z;
        }

        //--------------------------------------------------------------------------------------------------
        public double SurfaceArea()
        {
            double x = max.x - min.x;
            double y = max.y - min.y;
            double z = max.z - min.z;

            return 2 * (x * y + x * z + y * z);
        }

        //--------------------------------------------------------------------------------------------------
        public static AABB Combine(AABB a, AABB b)
        {
            AABB c;

            c.min = Vec3.Min(a.min, b.min);
            c.max = Vec3.Max(a.max, b.max);

            return c;
        }

        public Vec3 Clip(Vec3 t)
        {
            Vec3 center = (min + max) * 0.5;
            Vec3 extent = (max - min) * 0.5;

            t = t - center;
            for (int i = 0; i < 3; i++)
            {
                t[i] = Clamp(-extent[i], extent[i], t[i]);
                /*
                double a = Math.Abs(t[i]);
                double e = extent[i];
                if (a > e)
                    t = t * e / a;
                    */
            }
            return t + center;
        }
    };

    public struct HalfSpace
    {
        public HalfSpace(Vec3 normal, double distance)
        {
            this.normal = (normal);
            this.distance = (distance);
        }

        public void Set(Vec3 a, Vec3 b, Vec3 c)
        {
            normal = Vec3.Normalize(Vec3.Cross(b - a, c - a));
            distance = Vec3.Dot(normal, a);
        }

        public void Set(Vec3 n, Vec3 p)
        {
            normal = Vec3.Normalize(n);
            distance = Vec3.Dot(normal, p);
        }

        public Vec3 Origin()
        {
            return normal * distance;
        }

        public double Distance(Vec3 p)
        {
            return Vec3.Dot(normal, p) - distance;
        }

        public Vec3 Projected(Vec3 p)
        {
            return p - normal * Distance(p);
        }

        public Vec3 normal;
        public double distance;
    }

    //--------------------------------------------------------------------------------------------------
    // RaycastData
    //--------------------------------------------------------------------------------------------------
    public struct RaycastData
    {
        // Beginning point of the ray
        public Vec3 start;
        // Direction of the ray (normalized)
        public Vec3 dir;
        // Time specifying ray endpoint
        public double t;

        // Solved time of impact
        public double toi;
        // Surface normal at impact
        public Vec3 normal;	

        public void Set(Vec3 startPoint, Vec3 direction, double endPointTime)
        {
            start = startPoint;
            dir = direction;
            t = endPointTime;
        }

        // Uses toi, start and dir to compute the point at toi. Should
        // only be called after a raycast has been conducted with a
        // return value of true.
        public Vec3 GetImpactPoint()
        {
            return (start + dir * toi);
        }
    }
}