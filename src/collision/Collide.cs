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
    public static class Collide
    {
        public static bool TrackFaceAxis(ref int axis, int n, double s, ref double sMax, Vec3 normal, ref Vec3 axisNormal)
        {
            if (s > 0)
                return true;

            if (s > sMax)
            {
                sMax = s;
                axis = n;
                axisNormal = normal;
            }

            return false;
        }

        //--------------------------------------------------------------------------------------------------
        public static bool TrackEdgeAxis(ref int axis, int n, double s,ref double sMax, Vec3 normal,ref Vec3 axisNormal)
        {
            if (s > 0)
                return true;

            double l = 1 / Vec3.Length(normal);
            s *= l;

            if (s > sMax)
            {
                sMax = s;
                axis = n;
                axisNormal = normal * l;
            }

            return false;
        }

        //--------------------------------------------------------------------------------------------------
        public struct ClipVertex
        {
            /*
            public ClipVertex()
            {
                f.key = ~0;
            }
            */

            public Vec3 v;
            public FeaturePair f;
        };

        //--------------------------------------------------------------------------------------------------
        public static void ComputeReferenceEdgesAndBasis(Vec3 eR, Transform rtx, Vec3 n, int axis, byte[] result, out Mat3 basis, out Vec3 e)
        {
            basis = new Mat3();
            e = new Vec3();

            n = Transform.MulT(rtx.rotation, n);

            if (axis >= 3)
                axis -= 3;

            switch (axis)
            {
                case 0:
                    if (n.x > 0)
                    {
                        result[0] = 1;
                        result[1] = 8;
                        result[2] = 7;
                        result[3] = 9;

                        e.Set(eR.y, eR.z, eR.x);
                        basis.SetRows(rtx.rotation.ey, rtx.rotation.ez, rtx.rotation.ex);
                    }

                    else
                    {
                        result[0] = 11;
                        result[1] = 3;
                        result[2] = 10;
                        result[3] = 5;

                        e.Set(eR.z, eR.y, eR.x);
                        basis.SetRows(rtx.rotation.ez, rtx.rotation.ey, -rtx.rotation.ex);
                    }
                    break;

                case 1:
                    if (n.y > 0)
                    {
                        result[0] = 0;
                        result[1] = 1;
                        result[2] = 2;
                        result[3] = 3;

                        e.Set(eR.z, eR.x, eR.y);
                        basis.SetRows(rtx.rotation.ez, rtx.rotation.ex, rtx.rotation.ey);
                    }

                    else
                    {
                        result[0] = 4;
                        result[1] = 5;
                        result[2] = 6;
                        result[3] = 7;

                        e.Set(eR.z, eR.x, eR.y);
                        basis.SetRows(rtx.rotation.ez, -rtx.rotation.ex, -rtx.rotation.ey);
                    }
                    break;

                case 2:
                    if (n.z > 0)
                    {
                        result[0] = 11;
                        result[1] = 4;
                        result[2] = 8;
                        result[3] = 0;

                        e.Set(eR.y, eR.x, eR.z);
                        basis.SetRows(-rtx.rotation.ey, rtx.rotation.ex, rtx.rotation.ez);
                    }

                    else
                    {
                        result[0] = 6;
                        result[1] = 10;
                        result[2] = 2;
                        result[3] = 9;

                        e.Set(eR.y, eR.x, eR.z);
                        basis.SetRows(-rtx.rotation.ey, -rtx.rotation.ex, -rtx.rotation.ez);
                    }
                    break;
            }
        }

        //--------------------------------------------------------------------------------------------------
        public static void ComputeIncidentFace(Transform itx, Vec3 e, Vec3 n, ClipVertex[] result)
        {
            n = -Transform.MulT(itx.rotation, n);
            Vec3 absN = Vec3.Abs(n);

            if (absN.x > absN.y && absN.x > absN.z)
            {
                if (n.x > 0)
                {
                    result[0].v.Set(e.x, e.y, -e.z);
                    result[1].v.Set(e.x, e.y, e.z);
                    result[2].v.Set(e.x, -e.y, e.z);
                    result[3].v.Set(e.x, -e.y, -e.z);

                    result[0].f.inI = 9;
                    result[0].f.outI = 1;
                    result[1].f.inI = 1;
                    result[1].f.outI = 8;
                    result[2].f.inI = 8;
                    result[2].f.outI = 7;
                    result[3].f.inI = 7;
                    result[3].f.outI = 9;
                }

                else
                {
                    result[0].v.Set(-e.x, -e.y, e.z);
                    result[1].v.Set(-e.x, e.y, e.z);
                    result[2].v.Set(-e.x, e.y, -e.z);
                    result[3].v.Set(-e.x, -e.y, -e.z);

                    result[0].f.inI = 5;
                    result[0].f.outI = 11;
                    result[1].f.inI = 11;
                    result[1].f.outI = 3;
                    result[2].f.inI = 3;
                    result[2].f.outI = 10;
                    result[3].f.inI = 10;
                    result[3].f.outI = 5;
                }
            }

            else if (absN.y > absN.x && absN.y > absN.z)
            {
                if (n.y > 0)
                {
                    result[0].v.Set(-e.x, e.y, e.z);
                    result[1].v.Set(e.x, e.y, e.z);
                    result[2].v.Set(e.x, e.y, -e.z);
                    result[3].v.Set(-e.x, e.y, -e.z);

                    result[0].f.inI = 3;
                    result[0].f.outI = 0;
                    result[1].f.inI = 0;
                    result[1].f.outI = 1;
                    result[2].f.inI = 1;
                    result[2].f.outI = 2;
                    result[3].f.inI = 2;
                    result[3].f.outI = 3;
                }

                else
                {
                    result[0].v.Set(e.x, -e.y, e.z);
                    result[1].v.Set(-e.x, -e.y, e.z);
                    result[2].v.Set(-e.x, -e.y, -e.z);
                    result[3].v.Set(e.x, -e.y, -e.z);

                    result[0].f.inI = 7;
                    result[0].f.outI = 4;
                    result[1].f.inI = 4;
                    result[1].f.outI = 5;
                    result[2].f.inI = 5;
                    result[2].f.outI = 6;
                    result[3].f.inI = 5;
                    result[3].f.outI = 6;
                }
            }

            else
            {
                if (n.z > 0)
                {
                    result[0].v.Set(-e.x, e.y, e.z);
                    result[1].v.Set(-e.x, -e.y, e.z);
                    result[2].v.Set(e.x, -e.y, e.z);
                    result[3].v.Set(e.x, e.y, e.z);

                    result[0].f.inI = 0;
                    result[0].f.outI = 11;
                    result[1].f.inI = 11;
                    result[1].f.outI = 4;
                    result[2].f.inI = 4;
                    result[2].f.outI = 8;
                    result[3].f.inI = 8;
                    result[3].f.outI = 0;
                }

                else
                {
                    result[0].v.Set(e.x, -e.y, -e.z);
                    result[1].v.Set(-e.x, -e.y, -e.z);
                    result[2].v.Set(-e.x, e.y, -e.z);
                    result[3].v.Set(e.x, e.y, -e.z);

                    result[0].f.inI = 9;
                    result[0].f.outI = 6;
                    result[1].f.inI = 6;
                    result[1].f.outI = 10;
                    result[2].f.inI = 10;
                    result[2].f.outI = 2;
                    result[3].f.inI = 2;
                    result[3].f.outI = 9;
                }
            }

            for (int i = 0; i < 4; ++i)
                result[i].v = Transform.Mul(itx, result[i].v);
        }

        //--------------------------------------------------------------------------------------------------

        static bool InFront(double a) { return a < 0; }
        static bool Behind(double a) { return a > 0; }
        static bool On(double a) { return a < 0.005 && a > -0.005; }

        public static int Orthographic(double sign, double e, int axis, byte clipEdge, ClipVertex[] input, int inCount, ClipVertex[] result)
        {
            int resultCount = 0;
            ClipVertex a = input[inCount - 1];

            for (int i = 0; i < inCount; ++i)
            {
                ClipVertex b = input[i];

                double da = sign * a.v[axis] - e;
                double db = sign * b.v[axis] - e;

                ClipVertex cv = new ClipVertex();

                // B
                if (((InFront(da) && InFront(db)) || On(da) || On(db)))
                {

                    Assert(resultCount < 8);
                    result[resultCount++] = b;
                }

                // I
                else if (InFront(da) && Behind(db))
                {
                    cv.f = b.f;
                    cv.v = a.v + (b.v - a.v) * (da / (da - db));
                    cv.f.outR = clipEdge;
                    cv.f.outI = 0;

                    Assert(resultCount < 8);
                    result[resultCount++] = cv;
                }

                // I, B
                else if (Behind(da) && InFront(db))
                {
                    cv.f = a.f;
                    cv.v = a.v + (b.v - a.v) * (da / (da - db));
                    cv.f.inR = clipEdge;
                    cv.f.inI = 0;

                    Assert(resultCount < 8);
                    result[resultCount++] = cv;


                    Assert(resultCount < 8);
                    result[resultCount++] = b;
                }

                a = b;
            }

            return resultCount;
        }

        static ClipVertex[] input = new ClipVertex[8];
        static ClipVertex[] result = new ClipVertex[8];

        //--------------------------------------------------------------------------------------------------
        // Resources (also see q3BoxtoBox's resources):
        // http://www.randygaul.net/2013/10/27/sutherland-hodgman-clipping/
        public static int Clip(Vec3 rPos, Vec3 e, byte[] clipEdges, Mat3 basis, ClipVertex[] incident, ClipVertex[] resultVerts, double[] resultDepths)
        {
            int inCount = 4;
            int resultCount;

            for (int i = 0; i < 4; ++i)
                input[i].v = Transform.MulT(basis, incident[i].v - rPos);

            resultCount = Orthographic(1, e.x, 0, clipEdges[0], input, inCount, result);

            if (resultCount == 0)
                return 0;

            inCount = Orthographic(1, e.y, 1, clipEdges[1], result, resultCount, input);

            if (inCount == 0)
                return 0;

            resultCount = Orthographic(-1, e.x, 0, clipEdges[2], input, inCount, result);

            if (resultCount == 0)
                return 0;

            inCount = Orthographic(-1, e.y, 1, clipEdges[3], result, resultCount, input);

            // Keep incident vertices behind the reference face
            resultCount = 0;
            for (int i = 0; i < inCount; ++i)
            {
                double d = input[i].v.z - e.z;

                if (d <= 0)
                {
                    resultVerts[resultCount].v = Transform.Mul(basis, input[i].v) + rPos;
                    resultVerts[resultCount].f = input[i].f;
                    resultDepths[resultCount++] = d;
                }
            }


            Assert(resultCount <= 8);

            return resultCount;
        }

        //--------------------------------------------------------------------------------------------------
        public static void EdgesContact(out Vec3 CA, out Vec3 CB, Vec3 PA, Vec3 QA, Vec3 PB, Vec3 QB)
        {
            Vec3 DA = QA - PA;
            Vec3 DB = QB - PB;
            Vec3 r = PA - PB;
            double a = Vec3.Dot(DA, DA);
            double e = Vec3.Dot(DB, DB);
            double f = Vec3.Dot(DB, r);
            double c = Vec3.Dot(DA, r);

            double b = Vec3.Dot(DA, DB);
            double denom = a * e - b * b;

            double TA = (b * f - c * e) / denom;
            double TB = (b * TA + f) / e;

            CA = PA + DA * TA;
            CB = PB + DB * TB;
        }

        //--------------------------------------------------------------------------------------------------
        public static void SupportEdge(Transform tx, Vec3 e, Vec3 n, out Vec3 aresult, out Vec3 bresult)
        {
            n = Transform.MulT(tx.rotation, n);
            Vec3 absN = Vec3.Abs(n);
            Vec3 a = new Vec3(), b = new Vec3();

            // x > y
            if (absN.x > absN.y)
            {
                // x > y > z
                if (absN.y > absN.z)
                {
                    a.Set(e.x, e.y, e.z);
                    b.Set(e.x, e.y, -e.z);
                }

                // x > z > y || z > x > y
                else
                {
                    a.Set(e.x, e.y, e.z);
                    b.Set(e.x, -e.y, e.z);
                }
            }

            // y > x
            else
            {
                // y > x > z
                if (absN.x > absN.z)
                {
                    a.Set(e.x, e.y, e.z);
                    b.Set(e.x, e.y, -e.z);
                }

                // z > y > x || y > z > x
                else
                {
                    a.Set(e.x, e.y, e.z);
                    b.Set(-e.x, e.y, e.z);
                }
            }

            double signx = Sign(n.x);
            double signy = Sign(n.y);
            double signz = Sign(n.z);

            a.x *= signx;
            a.y *= signy;
            a.z *= signz;
            b.x *= signx;
            b.y *= signy;
            b.z *= signz;

            aresult = Transform.Mul(tx, a);
            bresult = Transform.Mul(tx, b);
        }

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
        public static void BoxtoBox(Manifold m, Box a, Box b)
        {
            Transform atx = a.body.GetTransform();
            Transform btx = b.body.GetTransform();
            Transform aL = a.local;
            Transform bL = b.local;
            atx = Transform.Mul(atx, aL);
            btx = Transform.Mul(btx, bL);
            Vec3 eA = a.e;
            Vec3 eB = b.e;

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
            double kRelTol = 0.95;
            double kAbsTol = 0.01;
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
                Transform rtx;
                Transform itx;
                Vec3 eR;
                Vec3 eI;
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

                Vec3 PA, QA;
                Vec3 PB, QB;

                SupportEdge(atx, eA, n, out PA, out QA);

                SupportEdge(btx, eB, -n, out PB, out QB);

                Vec3 CA, CB;

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

        public static void ComputeCollision (Manifold m, Shape a, Shape b)
        {
            if (a is Box && b is Box)
            {
                BoxtoBox(m, a as Box, b as Box);
            }
        }

        static void Swap<T>(ref T l, ref T r)
        {
            var tmp = l;
            l = r;
            r = tmp;
        }

        static double Sign (double v) { return v >= 0 ? 1 : -1; }
    }
}
