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
    public struct Vec3
    {

        public double x;
        public double y;
        public double z;

        public Vec3(double _x, double _y, double _z)
        {
            x = _x;
            y = _y;
            z = _z;
        }

        public void Set(double _x, double _y, double _z)
        {
            x = _x;
            y = _y;
            z = _z;
        }

        public void SetAll(double a)
        {
            x = a;
            y = a;
            z = a;
        }

        public double this[int i]
        {
            get
            {
                switch (i)
                {
                    case 0: return x;
                    case 1: return y;
                    case 2: return z;
                    default: throw null;
                }
            }
            set
            {
                switch (i)
                {
                    case 0: x = value; break;
                    case 1: y = value; break;
                    case 2: z = value; break;
                    default: throw null;
                }
            }
        }

        static public Vec3 operator -(Vec3 lhs)
        {
            return new Vec3(-lhs.x, -lhs.y, -lhs.z);
        }

        static public Vec3 operator +(Vec3 lhs, Vec3 rhs)
        {
            return new Vec3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
        }
        static public Vec3 operator -(Vec3 lhs, Vec3 rhs)
        {
            return new Vec3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
        }
        static public Vec3 operator *(Vec3 lhs, double f)
        {
            return new Vec3(lhs.x * f, lhs.y * f, lhs.z * f);
        }
        static public Vec3 operator *(double f, Vec3 lhs)
        {
            return new Vec3(lhs.x * f, lhs.y * f, lhs.z * f);
        }
        static public Vec3 operator /(Vec3 lhs, double f)
        {
            return new Vec3(lhs.x / f, lhs.y / f, lhs.z / f);
        }


        //--------------------------------------------------------------------------------------------------
        // Vec3
        //--------------------------------------------------------------------------------------------------
        public static void Identity(ref Vec3 v)
        {
            v.Set(0, 0, 0);
        }

        //--------------------------------------------------------------------------------------------------
        public static Vec3 Mul(Vec3 a, Vec3 b)
        {
            return new Vec3(a.x * b.x, a.y * b.y, a.z * b.z);
        }

        //--------------------------------------------------------------------------------------------------
        public static double Dot(Vec3 a, Vec3 b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        //--------------------------------------------------------------------------------------------------
        public static Vec3 Cross(Vec3 a, Vec3 b)
        {
            return new Vec3(
                (a.y * b.z) - (b.y * a.z),
                (b.x * a.z) - (a.x * b.z),
                (a.x * b.y) - (b.x * a.y)
                );
        }

        //--------------------------------------------------------------------------------------------------
        public static double Length(Vec3 v)
        {
            return Math.Sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        }

        //--------------------------------------------------------------------------------------------------
        public static double LengthSq(Vec3 v)
        {
            return v.x * v.x + v.y * v.y + v.z * v.z;
        }

        //--------------------------------------------------------------------------------------------------
        public static Vec3 Normalize(Vec3 v)
        {
            double l = Length(v);

            if (l != 0)
            {
                double inv = 1 / l;
                return v * inv;
            }

            return v;
        }

        //--------------------------------------------------------------------------------------------------
        public static double Distance(Vec3 a, Vec3 b)
        {
            double xp = a.x - b.x;
            double yp = a.y - b.y;
            double zp = a.z - b.z;

            return Math.Sqrt(xp * xp + yp * yp + zp * zp);
        }

        //--------------------------------------------------------------------------------------------------
        public static double DistanceSq(Vec3 a, Vec3 b)
        {
            double xp = a.x - b.x;
            double yp = a.y - b.y;
            double zp = a.z - b.z;

            return xp * xp + yp * yp + zp * zp;
        }

        //--------------------------------------------------------------------------------------------------
        public static Vec3 Abs(Vec3 v)
        {
            return new Vec3(Math.Abs(v.x), Math.Abs(v.y), Math.Abs(v.z));
        }

        //--------------------------------------------------------------------------------------------------
        public static Vec3 Min(Vec3 a, Vec3 b)
        {
            return new Vec3(Math.Min(a.x, b.x), Math.Min(a.y, b.y), Math.Min(a.z, b.z));
        }

        //--------------------------------------------------------------------------------------------------
        public static Vec3 Max(Vec3 a, Vec3 b)
        {
            return new Vec3(Math.Max(a.x, b.x), Math.Max(a.y, b.y), Math.Max(a.z, b.z));
        }

        //--------------------------------------------------------------------------------------------------
        public static double MinPerElem(Vec3 a)
        {
            return Math.Min(a.x, Math.Min(a.y, a.z));
        }

        //--------------------------------------------------------------------------------------------------
        public static double MaxPerElem(Vec3 a)
        {
            return Math.Max(a.x, Math.Max(a.y, a.z));
        }


    }
}
