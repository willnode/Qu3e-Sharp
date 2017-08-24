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
using static Qu3e.Algorithms;


namespace Qu3e
{
    public static class Collide
    {
        
        //--------------------------------------------------------------------------------------------------
        // Caches for BoxtoBox
        static ClipVertex[] incident = new ClipVertex[4];
        static byte[] clipEdges = new byte[4];
        static ClipVertex[] results = new ClipVertex[8];
        static double[] depths = new double[8];
        // Resources:
        // http://www.randygaul.net/2014/05/22/deriving-obb-to-obb-intersection-sat/
        // https://box2d.googlecode.com/files/GDC2007_ErinCatto.zip
        // https://box2d.googlecode.com/files/Box2D_Lite.zip
        static void BoxToBox(Manifold m, Box a, Box b)
        {
            Transform atx = a.GetWorldTransform();
            Transform btx = b.GetWorldTransform();
            Vec3 eA = a.extent;
            Vec3 eB = b.extent;

            // B's frame input A's space
            Mat3 C = Mat3.Transpose(atx.rotation) * btx.rotation;

            Mat3 absC = new Mat3();
            bool parallel = false;
            double kCosTol = 1e-6;
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    double val = Math.Abs(C[i][j]);
                    var o = absC[i];
                    o[j] = val;
                    absC[i] = o;

                    if (val + kCosTol >= 1)
                        parallel = true;
                }
            }

            // Vector from center A to center B input A's space
            Vec3 t = Transform.MulT(atx.rotation, btx.position - atx.position);

            // Query states
            double s;
            double aMax = -double.MaxValue;
            double bMax = -double.MaxValue;
            double eMax = -double.MaxValue;
            int aAxis = ~0;
            int bAxis = ~0;
            int eAxis = ~0;
            Vec3 nA = new Vec3();
            Vec3 nB = new Vec3();
            Vec3 nE = new Vec3();

            // Face axis checks

            // a's x axis
            s = Math.Abs(t.x) - (eA.x + Vec3.Dot(absC.Column0(), eB));
            if (TrackFaceAxis(ref aAxis, 0, s, ref aMax, atx.rotation.ex, ref nA))
                return;

            // a's y axis
            s = Math.Abs(t.y) - (eA.y + Vec3.Dot(absC.Column1(), eB));
            if (TrackFaceAxis(ref aAxis, 1, s, ref aMax, atx.rotation.ey, ref nA))
                return;

            // a's z axis
            s = Math.Abs(t.z) - (eA.z + Vec3.Dot(absC.Column2(), eB));
            if (TrackFaceAxis(ref aAxis, 2, s, ref aMax, atx.rotation.ez, ref nA))
                return;

            // b's x axis
            s = Math.Abs(Vec3.Dot(t, C.ex)) - (eB.x + Vec3.Dot(absC.ex, eA));
            if (TrackFaceAxis(ref bAxis, 3, s, ref bMax, btx.rotation.ex, ref nB))
                return;

            // b's y axis
            s = Math.Abs(Vec3.Dot(t, C.ey)) - (eB.y + Vec3.Dot(absC.ey, eA));
            if (TrackFaceAxis(ref bAxis, 4, s, ref bMax, btx.rotation.ey, ref nB))
                return;

            // b's z axis
            s = Math.Abs(Vec3.Dot(t, C.ez)) - (eB.z + Vec3.Dot(absC.ez, eA));
            if (TrackFaceAxis(ref bAxis, 5, s, ref bMax, btx.rotation.ez, ref nB))
                return;

            if (!parallel)
            {
                // Edge axis checks
                double rA;
                double rB;

                // Cross( a.x, b.x )
                rA = eA.y * absC[0][2] + eA.z * absC[0][1];
                rB = eB.y * absC[2][0] + eB.z * absC[1][0];
                s = Math.Abs(t.z * C[0][1] - t.y * C[0][2]) - (rA + rB);
                if (TrackEdgeAxis(ref eAxis, 6, s, ref eMax, new Vec3(0, -C[0][2], C[0][1]), ref nE))
                    return;

                // Cross( a.x, b.y )
                rA = eA.y * absC[1][2] + eA.z * absC[1][1];
                rB = eB.x * absC[2][0] + eB.z * absC[0][0];
                s = Math.Abs(t.z * C[1][1] - t.y * C[1][2]) - (rA + rB);
                if (TrackEdgeAxis(ref eAxis, 7, s, ref eMax, new Vec3(0, -C[1][2], C[1][1]), ref nE))
                    return;

                // Cross( a.x, b.z )
                rA = eA.y * absC[2][2] + eA.z * absC[2][1];
                rB = eB.x * absC[1][0] + eB.y * absC[0][0];
                s = Math.Abs(t.z * C[2][1] - t.y * C[2][2]) - (rA + rB);
                if (TrackEdgeAxis(ref eAxis, 8, s, ref eMax, new Vec3(0, -C[2][2], C[2][1]), ref nE))
                    return;

                // Cross( a.y, b.x )
                rA = eA.x * absC[0][2] + eA.z * absC[0][0];
                rB = eB.y * absC[2][1] + eB.z * absC[1][1];
                s = Math.Abs(t.x * C[0][2] - t.z * C[0][0]) - (rA + rB);
                if (TrackEdgeAxis(ref eAxis, 9, s, ref eMax, new Vec3(C[0][2], 0, -C[0][0]), ref nE))
                    return;

                // Cross( a.y, b.y )
                rA = eA.x * absC[1][2] + eA.z * absC[1][0];
                rB = eB.x * absC[2][1] + eB.z * absC[0][1];
                s = Math.Abs(t.x * C[1][2] - t.z * C[1][0]) - (rA + rB);
                if (TrackEdgeAxis(ref eAxis, 10, s, ref eMax, new Vec3(C[1][2], 0, -C[1][0]), ref nE))
                    return;

                // Cross( a.y, b.z )
                rA = eA.x * absC[2][2] + eA.z * absC[2][0];
                rB = eB.x * absC[1][1] + eB.y * absC[0][1];
                s = Math.Abs(t.x * C[2][2] - t.z * C[2][0]) - (rA + rB);
                if (TrackEdgeAxis(ref eAxis, 11, s, ref eMax, new Vec3(C[2][2], 0, -C[2][0]), ref nE))
                    return;

                // Cross( a.z, b.x )
                rA = eA.x * absC[0][1] + eA.y * absC[0][0];
                rB = eB.y * absC[2][2] + eB.z * absC[1][2];
                s = Math.Abs(t.y * C[0][0] - t.x * C[0][1]) - (rA + rB);
                if (TrackEdgeAxis(ref eAxis, 12, s, ref eMax, new Vec3(-C[0][1], C[0][0], 0), ref nE))
                    return;

                // Cross( a.z, b.y )
                rA = eA.x * absC[1][1] + eA.y * absC[1][0];
                rB = eB.x * absC[2][2] + eB.z * absC[0][2];
                s = Math.Abs(t.y * C[1][0] - t.x * C[1][1]) - (rA + rB);
                if (TrackEdgeAxis(ref eAxis, 13, s, ref eMax, new Vec3(-C[1][1], C[1][0], 0), ref nE))
                    return;

                // Cross( a.z, b.z )
                rA = eA.x * absC[2][1] + eA.y * absC[2][0];
                rB = eB.x * absC[1][2] + eB.y * absC[0][2];
                s = Math.Abs(t.y * C[2][0] - t.x * C[2][1]) - (rA + rB);
                if (TrackEdgeAxis(ref eAxis, 14, s, ref eMax, new Vec3(-C[2][1], C[2][0], 0), ref nE))
                    return;
            }

            // Artificial axis bias to improve frame coherence
            const double kRelTol = 0.95, kAbsTol = 0.01;

            int axis;
            double sMax;
            Vec3 n;
            double faceMax = Math.Max(aMax, bMax);

            if (kRelTol * eMax > faceMax + kAbsTol)
            {
                axis = eAxis;
                sMax = eMax;
                n = nE;
            }
            else
            {
                if (kRelTol * bMax > aMax + kAbsTol)
                {
                    axis = bAxis;
                    sMax = bMax;
                    n = nB;
                }

                else
                {
                    axis = aAxis;
                    sMax = aMax;
                    n = nA;
                }
            }

            if (Vec3.Dot(n, btx.position - atx.position) < 0)
                n = -n;



            if (axis < 6)
            {
                Transform rtx, itx;
                Vec3 eR, eI;
                bool flip;

                if (axis < 3)
                {
                    rtx = atx;
                    itx = btx;
                    eR = eA;
                    eI = eB;
                    flip = false;
                }
                else
                {
                    rtx = btx;
                    itx = atx;
                    eR = eB;
                    eI = eA;
                    flip = true;
                    n = -n;
                }

                // Compute reference and incident edge information necessary for clipping

                ComputeIncidentFace(itx, eI, n, incident);
                Mat3 basis;
                Vec3 e;
                ComputeReferenceEdgesAndBasis(eR, rtx, n, axis, clipEdges, out basis, out e);

                // Clip the incident face against the reference face side planes
                int resultNum;
                resultNum = Clip(rtx.position, e, clipEdges, basis, incident, results, depths);

                if (resultNum != 0)
                {
                    m.contactCount = resultNum;
                    m.normal = flip ? -n : n;

                    for (int i = 0; i < resultNum; ++i)
                    {
                        Contact c = m.contacts[i];

                        FeaturePair pair = results[i].f;

                        if (flip)
                        {

                            Swap(ref pair.inI, ref pair.inR);
                            Swap(ref pair.outI, ref pair.outR);
                        }

                        c.fp = results[i].f;
                        c.position = results[i].v;
                        c.penetration = depths[i];
                    }
                }
            }
            else
            {
                n = atx.rotation * n;

                if (Vec3.Dot(n, btx.position - atx.position) < 0)
                    n = -n;

                Vec3 PA, QA, PB, QB, CA, CB;

                SupportEdge(atx, eA, n, out PA, out QA);
                SupportEdge(btx, eB, -n, out PB, out QB);

                EdgesContact(out CA, out CB, PA, QA, PB, QB);

                m.normal = n;
                m.contactCount = 1;

                Contact c = m.contacts[0];
                FeaturePair pair = new FeaturePair();

                pair.key = axis;
                c.fp = pair;
                c.penetration = sMax;
                c.position = (CA + CB) * (0.5);
            }
        }

        //--------------------------------------------------------------------------------------------------
        static void BoxToSphere(Manifold m, Box a, Sphere b)
        {
            Transform atx = a.GetWorldTransform();
            Transform btx = b.GetWorldTransform();

            Vec3 eA = a.extent;
            AABB eAABB = new AABB(-eA, eA);
            double rB = b.radius;

            // Vector from center A to center B relative to A's space
            Vec3 t = Transform.MulT(atx.rotation, btx.position - atx.position);
            // Nearest point to sphere inside box
            Vec3 n = eAABB.Clip(t);
            Vec3 d = t - n;
            double dd = Vec3.LengthSq(d);

            if (dd > rB * rB) return;
            
            m.normal = Transform.Mul(atx.rotation, Vec3.Normalize(d));
            m.contactCount = 1;
            Contact c = m.contacts[0];

            // Box-sphere does never generating any friction
            c.fp = new FeaturePair();
            c.position = Transform.Mul(atx, n);
            c.penetration = rB * rB - dd;
        }

        //--------------------------------------------------------------------------------------------------
        static void SphereToSphere(Manifold m, Sphere a, Sphere b)
        {
            Transform atx = a.GetWorldTransform();
            Transform btx = b.GetWorldTransform();
            double rA = a.radius;
            double rB = b.radius;
            double rT = rA + rB;

            Vec3 d = btx.position - atx.position;
            double dd = Vec3.Dot(d, d);

            if (dd > rT * rT) return;

            double distance = Vec3.Length(d);
            Vec3 normal = new Vec3(0.0f, 1.0f, 0.0f);
            if (distance > 1e-8)
            {
                normal = d / distance;
            }

            m.normal = normal;
            m.contactCount = 1;

            Contact c = m.contacts[0];
            
            // Sphere-sphere does never generating any friction
            c.fp = new FeaturePair();

            c.position = (atx.position + btx.position) * (0.5);
            c.penetration = rT * rT - dd; // Avoid Sqrt for performance
        }

        //--------------------------------------------------------------------------------------------------
        static void BoxToCapsule(Manifold m, Box a, Capsule b)
        {
            Transform atx = a.GetWorldTransform();
            Transform btx = b.GetWorldTransform();
            Vec3 eA = a.extent;
            AABB eAABB = new AABB(-eA, eA);
            double rB = b.radius;
            double eB = b.extent;

            // Vector from pole A to center B relative to A's space
            Vec3 tA = Transform.MulT(atx, Transform.Mul(btx, new Vec3(0, eB, 0)));
            Vec3 tB = Transform.MulT(atx, Transform.Mul(btx, new Vec3(0, -eB, 0)));
            
            Vec3 t, n;
            ConvexEdgeContact(out n, out t, AABBToConvex(eAABB), tA, tB);
            Vec3 d = t - n;
            double dd = Vec3.LengthSq(d);
            
            if (dd > rB * rB) return;

            m.normal = Transform.Mul(atx.rotation, Vec3.Normalize(d));
            m.contactCount = 1;
            Contact c = m.contacts[0];

            // Box-cylinder does never generating any friction
            c.fp = new FeaturePair();
            c.position = Transform.Mul(atx, n);
            c.penetration = rB * rB - dd;
        }

        //--------------------------------------------------------------------------------------------------
        static void SphereToCapsule(Manifold m, Sphere a, Capsule b)
        {
            Transform atx = a.GetWorldTransform();
            Transform btx = b.GetWorldTransform();
            double rA = a.radius;
            double rB = b.radius;
            double rT = rA + rB;
            double eB = b.extent;

            // Vector from center B to center A relative to B's space
            Vec3 t = Transform.MulT(btx, atx.position);

            Vec3 O;
            if (Math.Abs(t.y) >= eB)
            {
                // Collision hit on the hemisphere
                O = new Vec3(0, Sign(t.y), 0) * eB;
            }
            else
            {
                // Collision hit on the cylinder
                O = new Vec3(0, t.y, 0);
            }

            t = t - O;

            double dd = Vec3.LengthSq(t);
            if (dd > rB * rB) return;

            m.contactCount = 1;
            m.normal = Transform.Mul(atx.rotation, Vec3.Normalize(t));
            Contact c = m.contacts[0];

            // Sphere-capsule does never generating any friction
            c.fp = new FeaturePair();
            c.position = Transform.Mul(atx, O + (m.normal) * rB);
            c.penetration = rT * rT - dd; // Avoid Sqrt for performance
        }

        //--------------------------------------------------------------------------------------------------
        static void CapsuleToCapsule(Manifold m, Capsule a, Capsule b)
        {
            Transform atx = a.GetWorldTransform();
            Transform btx = b.GetWorldTransform();
            double rA = a.radius;
            double rB = b.radius;
            double rT = rA + rB;
            double eA = a.extent;
            double eB = b.extent;
            Vec3 oA = new Vec3(0, eA, 0);
            Vec3 oB = new Vec3(0, eB, 0);

            Vec3 A, B;
            EdgesContact(out A, out B, Transform.Mul(atx, oA), Transform.Mul(atx, -oA), 
                   Transform.Mul(btx, oB), Transform.Mul(btx, -oB));

            double dd = Vec3.DistanceSq(A, B);
            if (dd > rT * rT) return;

            {
                //TODO: Parallel, then make friction
            }
            
            m.contactCount = 1;
            m.normal = Vec3.Normalize(B - A);
            Contact c = m.contacts[0];

            // Non parallel Capsule-capsule does never generating any friction
            c.fp = new FeaturePair();
            c.position = (A + B) * 0.5;
            c.penetration = rT - Math.Sqrt(dd); // Avoid Sqrt for performance
        }

        //--------------------------------------------------------------------------------------------------
        public static void ComputeCollision (Manifold m, Shape a, Shape b)
        {
            if (a is Box)
            {
                if (b is Box)
                    BoxToBox(m, a as Box, b as Box);
                else if (b is Sphere)
                    BoxToSphere(m, a as Box, b as Sphere);
                else if (b is Capsule)
                    BoxToCapsule(m, a as Box, b as Capsule);
            }
            else if (a is Sphere)
            {
                if (b is Box)
                    BoxToSphere(m, b as Box, a as Sphere);
                else if (b is Sphere)
                    SphereToSphere(m, a as Sphere, b as Sphere);
                else if (b is Capsule)
                    SphereToCapsule(m, a as Sphere, b as Capsule);
            }
            else if (a is Capsule)
            {
                if (b is Box)
                    BoxToCapsule(m, b as Box, a as Capsule);
                else if (b is Sphere)
                    SphereToCapsule(m, b as Sphere, a as Capsule);
                else if (b is Capsule)
                    CapsuleToCapsule(m, a as Capsule, b as Capsule);
            }
        }

    }
}
