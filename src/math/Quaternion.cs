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
    public struct Quaternion
    {

        public double x;
        public double y;
        public double z;
        public double w;

        
        public Quaternion(double a, double b, double c, double d)
        {
            x = (a);
            y = (b);
            z = (c);
            w = (d);
        }
        public Quaternion(Vec3 axis, double radians)
        {
            x = 0; y = 0; z = 0; w = 0;
            Set(axis, radians);
        }

         public void Set(Quaternion q)
        {
            x = (q.x);
            y = (q.y);
            z = (q.z);
            w = (q.w);
        }
        public void Set(Vec3 axis, double radians)
        {
            double halfAngle = (0.5) * radians;
            double s = Math.Sin(halfAngle);
            x = s * axis.x;
            y = s * axis.y;
            z = s * axis.z;
            w = Math.Cos(halfAngle);
        }
        public void ToAxisAngle(out Vec3 axis, out double angle)
        {
            Assert(w <= 1);

            angle = 2 * Math.Acos(w);

            double l = Math.Sqrt(1 - w * w);

            if (l == 0)
            {
                axis = new Vec3(0, 0, 0);
            }

            else
            {
                l = 1 / l;
                axis = new Vec3(x * l, y * l, z * l);
            }
        }
        public void Integrate(Vec3 dv, double dt)
        {
            Quaternion q = new Quaternion(dv.x * dt, dv.y * dt, dv.z * dt, 0);

            q *= this;

            x += q.x * 0.5;
            y += q.y * 0.5;
            z += q.z * 0.5;
            w += q.w * 0.5;

            this.Set(Normalize(this));
        }

        static public Quaternion operator *(Quaternion lhs, Quaternion rhs)
        {
            return new Quaternion(
    lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
    lhs.w * rhs.y + lhs.y * rhs.w + lhs.z * rhs.x - lhs.x * rhs.z,
    lhs.w * rhs.z + lhs.z * rhs.w + lhs.x * rhs.y - lhs.y * rhs.x,
    lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z
    );
        }

        public Mat3 ToMat3()
        {
            double qx2 = x + x;
            double qy2 = y + y;
            double qz2 = z + z;
            double qxqx2 = x * qx2;
            double qxqy2 = x * qy2;
            double qxqz2 = x * qz2;
            double qxqw2 = w * qx2;
            double qyqy2 = y * qy2;
            double qyqz2 = y * qz2;
            double qyqw2 = w * qy2;
            double qzqz2 = z * qz2;
            double qzqw2 = w * qz2;

            return new Mat3(
                new Vec3(1 - qyqy2 - qzqz2, qxqy2 + qzqw2, qxqz2 - qyqw2),
                new Vec3(qxqy2 - qzqw2, 1 - qxqx2 - qzqz2, qyqz2 + qxqw2),
                new Vec3(qxqz2 + qyqw2, qyqz2 - qxqw2, 1 - qxqx2 - qyqy2)
                );
        }


        //--------------------------------------------------------------------------------------------------
        public static Quaternion Normalize(Quaternion q)
        {
            double x = q.x;
            double y = q.y;
            double z = q.z;
            double w = q.w;

            double d = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

            if (d == 0)
                w = 1;

            d = 1 / Math.Sqrt(d);

            if (d > 1e-8)
            {
                x *= d;
                y *= d;
                z *= d;
                w *= d;
            }

            return new Quaternion(x, y, z, w);
        }


    }
}
