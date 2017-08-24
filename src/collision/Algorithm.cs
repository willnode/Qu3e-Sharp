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
    // Common geometry algorithms
    internal static class Algorithms
    {
        //--------------------------------------------------------------------------------------------------
        public struct ClipVertex
        {
            public Vec3 v;
            public FeaturePair f;
        };

        // http://box2d.org/2014/02/computing-a-basis/
        public static void ComputeBasis(Vec3 a, ref Vec3 b, ref Vec3 c)
        {
            // Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
            // Then 3*s*s = 1, s = sqrt(1/3) = 0.57735027. This means that at least one component of a
            // unit vector must be greater or equal to 0.57735027. Can use SIMD select operation.

            if (Math.Abs(a.x) >= (0.57735027))
                b.Set(a.y, -a.x, 0);
            else
                b.Set(0, a.z, -a.y);

            b = Vec3.Normalize(b);
            c = Vec3.Cross(a, b);
        }

        internal static bool TrackFaceAxis(ref int axis, int n, double s, ref double sMax, Vec3 normal, ref Vec3 axisNormal)
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
        internal static bool TrackEdgeAxis(ref int axis, int n, double s, ref double sMax, Vec3 normal, ref Vec3 axisNormal)
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
        internal static void ComputeReferenceEdgesAndBasis(Vec3 eR, Transform rtx, Vec3 n, int axis, byte[] result, out Mat3 basis, out Vec3 e)
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
        internal static void ComputeIncidentFace(Transform itx, Vec3 e, Vec3 n, ClipVertex[] result)
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

        internal static int Orthographic(double sign, double e, int axis, byte clipEdge, ClipVertex[] input, int inCount, ClipVertex[] result)
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

        internal static ClipVertex[] input = new ClipVertex[8];
        internal static ClipVertex[] result = new ClipVertex[8];

        //--------------------------------------------------------------------------------------------------
        // Resources (also see q3BoxtoBox's resources):
        // http://www.randygaul.net/2013/10/27/sutherland-hodgman-clipping/
        internal static int Clip(Vec3 rPos, Vec3 e, byte[] clipEdges, Mat3 basis, ClipVertex[] incident, ClipVertex[] resultVerts, double[] resultDepths)
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
        // Get nearest point from two edges
        internal static void EdgesContact(out Vec3 CA, out Vec3 CB, Vec3 PA, Vec3 QA, Vec3 PB, Vec3 QB)
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

            double TA, TB;
            if (denom < 1e-8)
            {
                // Parallel
                TA = 0; TB = 0;
            }
            else
            {
                TA = Clamp(0, 1, (b * f - c * e) / denom);
                TB = Clamp(0, 1, (b * TA + f) / e);
            }

            CA = PA + DA * TA;
            CB = PB + DB * TB;
        }

        //--------------------------------------------------------------------------------------------------
        // Get nearest point along the edge
        internal static Vec3 EdgeVertexContact(Vec3 A, Vec3 B, Vec3 P)
        {
            var AB = B - A;
            var D = Vec3.Dot(P - A, AB) / Vec3.Dot(AB, AB);
            return A + Clamp(0, 1, D) * (AB);
        }

        internal static Vec3 EdgeVertexContact(Vec3 A, Vec3 B, Vec3 P, out int I)
        {
            
            var AB = B - A;
            var D = Vec3.Dot(P - A, AB) / Vec3.Dot(AB, AB);

            I = (D < 0 || D > 1) ? 1 : 2;
            return A + Clamp(0, 1, D) * (AB);
        }



        //--------------------------------------------------------------------------------------------------
        // Get nearest point along the triangle

        internal static Vec3 TriangleVertexContact(Vec3 A, Vec3 B, Vec3 C, Vec3 P)
        {
            int t;
            return TriangleVertexContact(A, B, C, P, out t);
        }

        internal static Vec3 TriangleVertexContact(Vec3 A, Vec3 B, Vec3 C, Vec3 P, out int I)
        {
            var plane = new HalfSpace();

            plane.Set(A, B, C);

            var Q = plane.Projected(P);

            Vec3 AB = B - A, AC = A - C, QA = A - Q, QB = B - Q, QC = C - Q;

            var ABC = Vec3.Cross(AB, AC);

            var u = Vec3.Dot(Vec3.Cross(QC, QB), ABC);
            var v = Vec3.Dot(Vec3.Cross(QA, QC), ABC);
            var w = Vec3.Dot(Vec3.Cross(QB, QA), ABC);

            // var d = Vec3.Dot(ABC, ABC);
            // var uvw = u + v + w;
            // Assert(Math.Abs( uvw / d - 1) < 1e-4);

            if (u >= 0 && v >= 0 && w >= 0)
            {
                // Inside the triangle
                I = 3;
                return Q;
            }
            else if (u >= 0 ^ v >= 0 ^ w >= 0)
            {
                // Corner area
                I = 1;
                if (u >= 0) return A;
                if (v >= 0) return B;
                if (w >= 0) return C;
            }
            else
            {
                // Edge area
                I = 2;
                if (u < 0) return EdgeVertexContact(B, C, Q);
                if (v < 0) return EdgeVertexContact(C, A, Q);
                if (w < 0) return EdgeVertexContact(A, B, Q);
            }
            return Q;
        }

        internal static int VertexIndex(Vec3[] Vs, Vec3 V)
        {
            for (int i = 0; i < Vs.Length; i++)
            {
                if (Vec3.DistanceSq(Vs[i], V) < 1e-4)
                    return i;
            }
            Assert(false);
            return -1;
        }

        //--------------------------------------------------------------------------------------------------
        // Get nearest point inside convex shape
        internal static Vec3 ConvexVertexContact(Vec3[] Vs, Vec3 Q)
        {
            Assert(Vs.Length > 2);

            // Setup the instrument
            int Ti;
            var Vc = new Vec3();
            var Vr = new Vec3();
            var Vb = ListPool<int>.Pop();
            var V = ListPool<VertexHull>.Pop();
            HalfSpace H = new HalfSpace();

            // Get median (don't have to be median, but must be a point inside convex)
            for (int i = 0; i < Vs.Length; i++)
            {
                Vc += Vs[i];
            }
            Vc /= Vs.Length;
            H.Set(Vc - Q, Q);

            // Get list of vertex data
            for (int i = 0; i < Vs.Length; i++)
            {
                V.Add(new VertexHull(Vs[i], i, H.Distance(Vs[i])));
            }

            // -- Points on faces

            do
            {

                int A = -1, B = -1, C = -1;
                double dA = double.MaxValue, dB = double.MaxValue, dC = double.MaxValue;

                // Get three nearest vertices
                for (int i = 0; i < Vs.Length; i++)
                {
                    if (Vb.Contains(i)) continue;

                    var dV = V[i].distance;
                    
                    if (dV < dA) {  Chain(i, ref A, ref B, ref C); Chain(dV, ref dA, ref dB, ref dC); }
                    else if (dV < dB) { Chain(i, ref B, ref C); Chain(dV, ref dB, ref dC); }
                    else if (dV < dC) { C = i; dC = dV; }
                }

                // Get the magic
                Vr = TriangleVertexContact(V[A].vector, V[B].vector, V[C].vector, Q, out Ti);

                // R must be parallel to triangle in order to be 'valid'
                if (Ti == 3)
                    return Vr;
                else
                {
                    // One of its vertex must be invalid.
                    if (dA > dB && dA > dC)
                        Vb.Add(V[A].index);
                    else if (dB > dC)
                        Vb.Add(V[B].index);
                    else
                        Vb.Add(V[C].index);
                }

            } while (Vb.Count + 2 < Vs.Length);

            // -- Points on edges

            Vb.Clear();

            do
            {

                int A = -1, B = -1;
                double dA = double.MaxValue, dB = double.MaxValue;

                // Get two nearest vertices
                for (int i = 0; i < Vs.Length; i++)
                {
                    if (Vb.Contains(i)) continue;

                    var dV = V[i].distance;

                    if (dV < dA) { Chain(i, ref A, ref B); Chain(dV, ref dA, ref dB); }
                    else if (dV < dB) { B = i; dB = dV; }
                }

                // Get the magic
                Vr = EdgeVertexContact(V[A].vector, V[B].vector, Q, out Ti);

                // R must be parallel to triangle in order to be 'valid'
                if (Ti == 2)
                    return Vr;
                else
                {
                    // One of its vertex must be invalid.
                    if (dA > dB)
                        Vb.Add(V[A].index);
                    else
                        Vb.Add(V[B].index);
                }

            } while (Vb.Count + 1 < Vs.Length);

            // -- Point on corner. Just find the nearest point!

            ListPool<int>.Push(Vb);

            {

                int A = -1;
                double dA = double.MaxValue;

                // Get two nearest vertices
                for (int i = 0; i < Vs.Length; i++)
                {
                    var dV = V[i].distance;

                    if (dV < dA) { A = i; dA = dV; }
                }

                // Get the magic
                return V[A].vector;

            } 
        }

        //--------------------------------------------------------------------------------------------------
        // Get nearest point between edge and convex shape
        internal static void ConvexEdgeContact(out Vec3 CV, out Vec3 CP, Vec3[] Vs, Vec3 PA, Vec3 PB)
        {
            Vec3 A = ConvexVertexContact(Vs, PA);
            Vec3 B = ConvexVertexContact(Vs, PB);
            EdgesContact(out CV, out CP, A, B, PA, PB);
        }

        //--------------------------------------------------------------------------------------------------
        internal static void SupportEdge(Transform tx, Vec3 e, Vec3 n, out Vec3 aresult, out Vec3 bresult)
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

        static Vec3[] cubeAABB = new Vec3[8];
        internal static Vec3[] AABBToConvex(AABB aabb)
        {
            var center = (aabb.min + aabb.max) / 2;
            var extent = (aabb.max - aabb.min) / 2;
            for (int i = 0; i < 8; i++)
            {
                cubeAABB[i] = Vec3.Mul(Box.kBoxVertices[i], extent) + center;
            }
            return cubeAABB;
        }

        internal struct VertexHull
        {
            public Vec3 vector;
            public int index;
            public double distance;

            public VertexHull (Vec3 v, int i, double d)
            {
                vector = v;
                index = i;
                distance = d;
            }
        }

    }
}
