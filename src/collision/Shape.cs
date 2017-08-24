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
    public abstract class Shape
    {

        public Transform local;

        public Body body;
        public double friction;
        public double restitution;
        public double density;
        public int broadPhaseIndex;
        public object userData;
        public bool sensor;

        public abstract bool TestPoint(Transform tx, Vec3 p);

        public abstract bool Raycast(Transform tx, ref RaycastData raycast);

        public abstract void ComputeAABB(Transform tx, out AABB aabb);

        public abstract void ComputeMass(out MassData md);

        public abstract void Render(Transform tx, bool awake, Render render);

        public Transform GetWorldTransform ()
        {
            return Transform.Mul(body.Tx, local);
        }


    }

    public struct MassData
    {
        public Mat3 inertia;
        public Vec3 center;
        public double mass;
    }

    public class ShapeDef
    {

        public ShapeDef()
        {
            // Common default values
            Type = ShapeType.Box;
            Friction = 0.4;
            Restitution = 0.2;
            Density = 1.0;
            Sensor = false;
            Tx = Transform.Identity;
            Size = Vec3.Uniform(1);
        }

        public ShapeType Type;
        public Transform Tx;
        public Vec3 Size;
        
        public double Friction;
        public double Restitution;
        public double Density;
        public bool Sensor;

    };

    public enum ShapeType
    {
        Box = 0,
        Sphere = 1,
        Capsule = 2,
    }

}