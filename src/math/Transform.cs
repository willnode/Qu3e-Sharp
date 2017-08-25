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
    public struct Transform
    {
        public Vec3 position;
        public Mat3 rotation;

        public static Vec3 Mul(Transform tx, Vec3 v)
        {
            return (tx.rotation * v + tx.position);
        }

        //--------------------------------------------------------------------------------------------------
        public static Vec3 Mul(Transform tx, Vec3 scale, Vec3 v)
        {
            return (tx.rotation * Vec3.Mul(scale, v) + tx.position);
        }

        //--------------------------------------------------------------------------------------------------
        public static Vec3 Mul(Mat3 r, Vec3 v)
        {
            return r * v;
        }

        //--------------------------------------------------------------------------------------------------
        public static Mat3 Mul(Mat3 r, Mat3 q)
        {
            return r * q;
        }

        //--------------------------------------------------------------------------------------------------
        public static Transform Mul(Transform t, Transform u)
        {
            Transform v = new Transform();
            v.rotation = Mul(t.rotation, u.rotation);
            v.position = Mul(t.rotation, u.position) + t.position;
            return v;
        }

        //--------------------------------------------------------------------------------------------------
        public static HalfSpace Mul(Transform tx, HalfSpace p)
        {
            Vec3 origin = p.Origin();
            origin = Mul(tx, origin);
            Vec3 normal = Mul(tx.rotation, p.normal);

            return new HalfSpace(normal, Vec3.Dot(origin, normal));
        }

        //--------------------------------------------------------------------------------------------------
        public static HalfSpace Mul(Transform tx, Vec3 scale, HalfSpace p)
        {
            Vec3 origin = p.Origin();
            origin = Mul(tx, scale, origin);
            Vec3 normal = Mul(tx.rotation, p.normal);

            return new HalfSpace(normal, Vec3.Dot(origin, normal));
        }

        //--------------------------------------------------------------------------------------------------
        public static Vec3 MulT(Transform tx, Vec3 v)
        {
            return Mat3.Transpose(tx.rotation) * (v - tx.position);
        }

        //--------------------------------------------------------------------------------------------------
        public static Vec3 MulT(Mat3 r, Vec3 v)
        {
            return Mat3.Transpose(r) * v;
        }

        //--------------------------------------------------------------------------------------------------
        public static Mat3 MulT(Mat3 r, Mat3 q)
        {
            return Mat3.Transpose(r) * q;
        }

        //--------------------------------------------------------------------------------------------------
        public static Transform MulT(Transform t, Transform u)
        {
            Transform v = new Transform();
            v.rotation = MulT(t.rotation, u.rotation);
            v.position = MulT(t.rotation, u.position - t.position);
            return v;
        }

        //--------------------------------------------------------------------------------------------------
        public static HalfSpace MulT(Transform tx, HalfSpace p)
        {
            Vec3 origin = p.normal * p.distance;
            origin = MulT(tx, origin);
            Vec3 n = MulT(tx.rotation, p.normal);
            return new HalfSpace(n, Vec3.Dot(origin, n));
        }

        //--------------------------------------------------------------------------------------------------
        public static Transform Identity
        {
            get
            {
                return new Transform() { rotation = Mat3.Identity };
            }
        }

    }
}
