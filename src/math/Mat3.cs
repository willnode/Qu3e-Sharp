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
    public struct Mat3
    {
        public Vec3 ex;
        public Vec3 ey;
        public Vec3 ez;


        public Mat3(double a, double b, double c, double d, double e, double f, double g, double h, double i)
        {
            ex = new Vec3(a, b, c);
            ey = new Vec3(d, e, f);
            ez = new Vec3(g, h, i);
        }
        public Mat3(Vec3 _x, Vec3 _y, Vec3 _z)
        {
            ex = (_x);
            ey = (_y);
            ez = (_z);
        }

        public void Set(double a, double b, double c, double d, double e, double f, double g, double h, double i)
        {
            ex.Set(a, b, c);
            ey.Set(d, e, f);
            ez.Set(g, h, i);
        }
        public void Set(Vec3 axis, double angle)
        {
            double s = Math.Sin(angle);
            double c = Math.Cos(angle);
            double x = axis.x;
            double y = axis.y;
            double z = axis.z;
            double xy = x * y;
            double yz = y * z;
            double zx = z * x;
            double t = 1 - c;

            Set(
                x * x * t + c, xy * t + z * s, zx * t - y * s,
                xy * t - z * s, y * y * t + c, yz * t + x * s,
                zx * t + y * s, yz * t - x * s, z * z * t + c
                );
        }

        public void SetRows(Vec3 x, Vec3 y, Vec3 z)
        {
            ex = x;
            ey = y;
            ez = z;
        }


        public Vec3 this[int index]
        {
            get
            {
                switch (index)
                {
                    case 0: return ex;
                    case 1: return ey;
                    case 2: return ez;
                    default:
                        Assert(false);
                        return ex;
                }
            }
            set
            {
                switch (index)
                {
                    case 0: ex = value; break;
                    case 1: ey = value; break;
                    case 2: ez = value; break;
                    default: Assert(false); break;
                }
            }
        }

        public Vec3 Column0()
        {
            return new Vec3(ex.x, ey.x, ez.x);
        }
        public Vec3 Column1()
        {
            return new Vec3(ex.y, ey.y, ez.y);
        }
        public Vec3 Column2()
        {
            return new Vec3(ex.z, ey.z, ez.z);
        }

        static public Vec3 operator *(Mat3 lhs, Vec3 rhs)
        {
            return new Vec3()
            {
                x = lhs.ex.x * rhs.x + lhs.ey.x * rhs.y + lhs.ez.x * rhs.z,
                y = lhs.ex.y * rhs.x + lhs.ey.y * rhs.y + lhs.ez.y * rhs.z,
                z = lhs.ex.z * rhs.x + lhs.ey.z * rhs.y + lhs.ez.z * rhs.z
            };
        }
        static public Mat3 operator *(Mat3 lhs, Mat3 rhs)
        {
            return new Mat3()
            {
                ex = (lhs * rhs.ex),
                ey = (lhs * rhs.ey),
                ez = (lhs * rhs.ez)
            };
        }
        static public Mat3 operator *(Mat3 lhs, double f)
        {
            return new Mat3()
            {
                ex = (lhs.ex * f),
                ey = (lhs.ey * f),
                ez = (lhs.ez * f)
            };
        }
        static public Mat3 operator +(Mat3 lhs, Mat3 rhs)
        {
            return new Mat3()
            {
                ex = (lhs.ex + rhs.ex),
                ey = (lhs.ey + rhs.ey),
                ez = (lhs.ez + rhs.ez)
            };
        }
        static public Mat3 operator -(Mat3 lhs, Mat3 rhs)
        {
            return new Mat3()
            {
                ex = (lhs.ex - rhs.ex),
                ey = (lhs.ey - rhs.ey),
                ez = (lhs.ez - rhs.ez)
            };
        }

        //--------------------------------------------------------------------------------------------------
        public static Mat3 Identity
        {
            get
            {
                return new Mat3(1, 0, 0, 0, 1, 0, 0, 0, 1);
            }
        }

        //--------------------------------------------------------------------------------------------------
        public static Mat3 Rotate(Vec3 x, Vec3 y, Vec3 z)
        {
            return new Mat3(x, y, z);
        }

        //--------------------------------------------------------------------------------------------------
        public static Mat3 Transpose(Mat3 m)
        {
            return new Mat3(
                m.ex.x, m.ey.x, m.ez.x,
                m.ex.y, m.ey.y, m.ez.y,
                m.ex.z, m.ey.z, m.ez.z
                );
        }

        //--------------------------------------------------------------------------------------------------
        public static void Zero(Mat3 m)
        {
            m.Set(0, 0, 0, 0, 0, 0, 0, 0, 0);
        }

        //--------------------------------------------------------------------------------------------------
        public static Mat3 Diagonal(double a)
        {
            return new Mat3(
                a, 0, 0,
                0, a, 0,
                0, 0, a
                );
        }

        //--------------------------------------------------------------------------------------------------
        public static Mat3 Diagonal(double a, double b, double c)
        {
            return new Mat3(
                a, 0, 0,
                0, b, 0,
                0, 0, c
                );
        }

        public static Mat3 Diagonal(Vec3 v)
        {
            return new Mat3(
                v.x, 0, 0,
                0, v.y, 0,
                0, 0, v.z
                );
        }

        //--------------------------------------------------------------------------------------------------
        public static Mat3 OuterProduct(Vec3 u, Vec3 v)
        {
            Vec3 a = v * u.x;
            Vec3 b = v * u.y;
            Vec3 c = v * u.z;

            return new Mat3(
                a.x, a.y, a.z,
                b.x, b.y, b.z,
                c.x, c.y, c.z
                );
        }

        //--------------------------------------------------------------------------------------------------
        public static Mat3 Covariance(Vec3[] points, int numPoints)
        {
            double invNumPoints = 1 / numPoints;
            Vec3 c = new Vec3(0, 0, 0);

            for (int i = 0; i < numPoints; ++i)
                c += points[i];

            c /= (numPoints);

            double m00, m11, m22, m01, m02, m12;
            m00 = m11 = m22 = m01 = m02 = m12 = 0;

            for (int i = 0; i < numPoints; ++i)
            {
                Vec3 p = points[i] - c;

                m00 += p.x * p.x;
                m11 += p.y * p.y;
                m22 += p.z * p.z;
                m01 += p.x * p.y;
                m02 += p.x * p.z;
                m12 += p.y * p.z;
            }

            double m01inv = m01 * invNumPoints;
            double m02inv = m02 * invNumPoints;
            double m12inv = m12 * invNumPoints;

            return new Mat3(
                m00 * invNumPoints, m01inv, m02inv,
                m01inv, m11 * invNumPoints, m12inv,
                m02inv, m12inv, m22 * invNumPoints
                );
        }

        //--------------------------------------------------------------------------------------------------
        public static Mat3 Inverse(Mat3 m)
        {
            Vec3 tmp0, tmp1, tmp2;
            double detinv;

            tmp0 = Vec3.Cross(m.ey, m.ez);
            tmp1 = Vec3.Cross(m.ez, m.ex);
            tmp2 = Vec3.Cross(m.ex, m.ey);

            detinv = 1 / Vec3.Dot(m.ez, tmp2);

            return new Mat3(
                tmp0.x * detinv, tmp1.x * detinv, tmp2.x * detinv,
                tmp0.y * detinv, tmp1.y * detinv, tmp2.y * detinv,
                tmp0.z * detinv, tmp1.z * detinv, tmp2.z * detinv
                );
        }


    }
}
