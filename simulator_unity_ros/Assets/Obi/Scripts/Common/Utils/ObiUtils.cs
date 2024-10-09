using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace Obi
{

    public static class Constants
    {
        public const int maxVertsPerMesh = 65000;
        public const int maxInstancesPerBatch = 1023;
    }

    public static class ObiUtils
    {

        // Colour alphabet from https://www.aic-color.org/resources/Documents/jaic_v5_06.pdf
        public static readonly Color32[] colorAlphabet = new Color32[26]
        {
            new Color32(240,163,255,255), 
            new Color32(0,117,220,255),
            new Color32(153,63,0,255),
            new Color32(76,0,92,255),
            new Color32(25,25,25,255),
            new Color32(0,92,49,255),
            new Color32(43,206,72,255),
            new Color32(255,204,153,255),
            new Color32(128,128,128,255),
            new Color32(148,255,181,255),
            new Color32(143,124,0,255),
            new Color32(157,204,0,255),
            new Color32(194,0,136,255),
            new Color32(0,51,128,255),
            new Color32(255,164,5,255),
            new Color32(255,168,187,255),
            new Color32(66,102,0,255),
            new Color32(255,0,16,255),
            new Color32(94,241,242,255),
            new Color32(0,153,143,255),
            new Color32(224,255,102,255),
            new Color32(116,10,255,255),
            new Color32(153,0,0,255),
            new Color32(255,255,128,255),  
            new Color32(255,255,0,255),
            new Color32(255,80,5,255)
        };

        public static void DrawArrowGizmo(float bodyLenght, float bodyWidth, float headLenght, float headWidth)
        {

            float halfBodyLenght = bodyLenght * 0.5f;
            float halfBodyWidth = bodyWidth * 0.5f;

            // arrow body:
            Gizmos.DrawLine(new Vector3(halfBodyWidth, 0, -halfBodyLenght), new Vector3(halfBodyWidth, 0, halfBodyLenght));
            Gizmos.DrawLine(new Vector3(-halfBodyWidth, 0, -halfBodyLenght), new Vector3(-halfBodyWidth, 0, halfBodyLenght));
            Gizmos.DrawLine(new Vector3(-halfBodyWidth, 0, -halfBodyLenght), new Vector3(halfBodyWidth, 0, -halfBodyLenght));

            // arrow head:
            Gizmos.DrawLine(new Vector3(halfBodyWidth, 0, halfBodyLenght), new Vector3(headWidth, 0, halfBodyLenght));
            Gizmos.DrawLine(new Vector3(-halfBodyWidth, 0, halfBodyLenght), new Vector3(-headWidth, 0, halfBodyLenght));
            Gizmos.DrawLine(new Vector3(0, 0, halfBodyLenght + headLenght), new Vector3(headWidth, 0, halfBodyLenght));
            Gizmos.DrawLine(new Vector3(0, 0, halfBodyLenght + headLenght), new Vector3(-headWidth, 0, halfBodyLenght));
        }

        public static void DebugDrawCross(Vector3 pos, float size, Color color)
        {
            Debug.DrawLine(pos - Vector3.right * size, pos + Vector3.right * size, color);
            Debug.DrawLine(pos - Vector3.up * size, pos + Vector3.up * size, color);
            Debug.DrawLine(pos - Vector3.forward * size, pos + Vector3.forward * size, color);
        }

        public static void Swap<T>(ref T lhs, ref T rhs)
        {
            T temp = lhs;
            lhs = rhs;
            rhs = temp;
        }

        public static void Swap<T>(this T[] source, int index1, int index2)
        {
            if (source != null && index1 >= 0 && index2 != 0 && index1 < source.Length && index2 < source.Length)
            {
                T temp = source[index1];
                source[index1] = source[index2];
                source[index2] = temp;
            }
        }

        public static void Swap<T>(this IList<T> list, int index1, int index2)
        {
            if (list != null && index1 >= 0 && index2 != 0 && index1 < list.Count && index2 < list.Count)
            {
                T temp = list[index1];
                list[index1] = list[index2];
                list[index2] = temp;
            }
        }

        public static void ShiftLeft<T>(this T[] source, int index, int count, int positions)
        {
            for (int j = 0; j < positions; ++j)
            {
                for (int i = index; i < index + count; ++i)
                    source.Swap(i, i - 1);
                index--;
            }
        }

        public static void ShiftRight<T>(this T[] source, int index, int count, int positions)
        {
            for (int j = 0; j < positions; ++j)
            {
                for (int i = index + count - 1; i >= index; --i)
                    source.Swap(i, i + 1);
                index++;
            }
        }

        public static bool AreValid(this Bounds bounds)
        {
            return !(float.IsNaN(bounds.center.x) || float.IsInfinity(bounds.center.x) ||
                     float.IsNaN(bounds.center.y) || float.IsInfinity(bounds.center.y) ||
                     float.IsNaN(bounds.center.z) || float.IsInfinity(bounds.center.z));
        }

        public static Bounds Transform(this Bounds b, Matrix4x4 m)
        {
            var xa = m.GetColumn(0) * b.min.x;
            var xb = m.GetColumn(0) * b.max.x;

            var ya = m.GetColumn(1) * b.min.y;
            var yb = m.GetColumn(1) * b.max.y;

            var za = m.GetColumn(2) * b.min.z;
            var zb = m.GetColumn(2) * b.max.z;

            Bounds result = new Bounds();
            Vector3 pos = m.GetColumn(3);
            result.SetMinMax(Vector3.Min(xa, xb) + Vector3.Min(ya, yb) + Vector3.Min(za, zb) + pos,
                             Vector3.Max(xa, xb) + Vector3.Max(ya, yb) + Vector3.Max(za, zb) + pos);


            return result;
        }

        public static void Add(Vector3 a, Vector3 b, ref Vector3 result)
        {
            result.x = a.x + b.x;
            result.y = a.y + b.y;
            result.z = a.z + b.z;
        }

        public static float Remap(this float value, float from1, float to1, float from2, float to2)
        {
            return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
        }

        /**
         * Modulo operator that also follows intuition for negative arguments. That is , -1 mod 3 = 2, not -1.
         */
        public static float Mod(float a, float b)
        {
            return a - b * Mathf.Floor(a / b);
        }

        public static Matrix4x4 Add(this Matrix4x4 a, Matrix4x4 other)
        {
            for (int i = 0; i < 16; ++i)
                a[i] += other[i];
            return a;
        }

        public static Matrix4x4 ScalarMultiply(this Matrix4x4 a, float s)
        {
            for (int i = 0; i < 16; ++i)
                a[i] *= s;
            return a;
        }

        public static Vector3 ProjectPointLine(Vector3 point, Vector3 lineStart, Vector3 lineEnd, out float mu, bool clampToSegment = true)
        {
            Vector3 ap = point - lineStart;
            Vector3 ab = lineEnd - lineStart;

            mu = Vector3.Dot(ap, ab) / Vector3.Dot(ab, ab);

            if (clampToSegment)
                mu = Mathf.Clamp01(mu);

            return lineStart + ab * mu;
        }

        public static bool LinePlaneIntersection(Vector3 planePoint, Vector3 planeNormal, Vector3 linePoint, Vector3 lineDirection, out Vector3 point)
        {
            point = linePoint;
            Vector3 lineNormal = lineDirection.normalized;
            float denom = Vector3.Dot(planeNormal, lineNormal);

            if (Mathf.Approximately(denom, 0))
                return false;

            float t = (Vector3.Dot(planeNormal,planePoint) - Vector3.Dot(planeNormal,linePoint)) / denom;
            point = linePoint + lineNormal * t;
            return true;
        }

        public static float InvMassToMass(float invMass)
        {
            return 1.0f / invMass;
        }

        public static float MassToInvMass(float mass)
        {
            return 1.0f / Mathf.Max(mass, 0.00001f);
        }

        public static int PureSign(this float val)
        {
            return ((0 <= val)?1:0) - ((val < 0)?1:0);
        }

        public static void NearestPointOnTri(in Vector3 p1,
                                                in Vector3 p2,
                                                in Vector3 p3,
                                                in Vector3 p,
                                                out Vector3 result)
        {
            float e0x = p2.x - p1.x;
            float e0y = p2.y - p1.y;
            float e0z = p2.z - p1.z;

            float e1x = p3.x - p1.x;
            float e1y = p3.y - p1.y;
            float e1z = p3.z - p1.z;

            float v0x = p1.x - p.x;
            float v0y = p1.y - p.y;
            float v0z = p1.z - p.z;

            float a00 = e0x * e0x + e0y * e0y + e0z * e0z;
            float a01 = e0x * e1x + e0y * e1y + e0z * e1z;
            float a11 = e1x * e1x + e1y * e1y + e1z * e1z;
            float b0 = e0x * v0x + e0y * v0y + e0z * v0z;
            float b1 = e1x * v0x + e1y * v0y + e1z * v0z;

            const float zero = 0;
            const float one = 1;

            float det = a00 * a11 - a01 * a01;
            float t0 = a01 * b1 - a11 * b0;
            float t1 = a01 * b0 - a00 * b1;

            if (t0 + t1 <= det)
            {
                if (t0 < zero)
                {
                    if (t1 < zero)  // region 4
                    {
                        if (b0 < zero)
                        {
                            t1 = zero;
                            if (-b0 >= a00)  // V0
                            {
                                t0 = one;
                            }
                            else  // E01
                            {
                                t0 = -b0 / a00;
                            }
                        }
                        else
                        {
                            t0 = zero;
                            if (b1 >= zero)  // V0
                            {
                                t1 = zero;
                            }
                            else if (-b1 >= a11)  // V2
                            {
                                t1 = one;
                            }
                            else  // E20
                            {
                                t1 = -b1 / a11;
                            }
                        }
                    }
                    else  // region 3
                    {
                        t0 = zero;
                        if (b1 >= zero)  // V0
                        {
                            t1 = zero;
                        }
                        else if (-b1 >= a11)  // V2
                        {
                            t1 = one;
                        }
                        else  // E20
                        {
                            t1 = -b1 / a11;
                        }
                    }
                }
                else if (t1 < zero)  // region 5
                {
                    t1 = zero;
                    if (b0 >= zero)  // V0
                    {
                        t0 = zero;
                    }
                    else if (-b0 >= a00)  // V1
                    {
                        t0 = one;
                    }
                    else  // E01
                    {
                        t0 = -b0 / a00;
                    }
                }
                else  // region 0, interior
                {
                    float invDet = one / det;
                    t0 *= invDet;
                    t1 *= invDet;
                }
            }
            else
            {
                float tmp0, tmp1, numer, denom;

                if (t0 < zero)  // region 2
                {
                    tmp0 = a01 + b0;
                    tmp1 = a11 + b1;
                    if (tmp1 > tmp0)
                    {
                        numer = tmp1 - tmp0;
                        denom = a00 - 2 * a01 + a11;
                        if (numer >= denom)  // V1
                        {
                            t0 = one;
                            t1 = zero;
                        }
                        else  // E12
                        {
                            t0 = numer / denom;
                            t1 = one - t0;
                        }
                    }
                    else
                    {
                        t0 = zero;
                        if (tmp1 <= zero)  // V2
                        {
                            t1 = one;
                        }
                        else if (b1 >= zero)  // V0
                        {
                            t1 = zero;
                        }
                        else  // E20
                        {
                            t1 = -b1 / a11;
                        }
                    }
                }
                else if (t1 < zero)  // region 6
                {
                    tmp0 = a01 + b1;
                    tmp1 = a00 + b0;
                    if (tmp1 > tmp0)
                    {
                        numer = tmp1 - tmp0;
                        denom = a00 - 2 * a01 + a11;
                        if (numer >= denom)  // V2
                        {
                            t1 = one;
                            t0 = zero;
                        }
                        else  // E12
                        {
                            t1 = numer / denom;
                            t0 = one - t1;
                        }
                    }
                    else
                    {
                        t1 = zero;
                        if (tmp1 <= zero)  // V1
                        {
                            t0 = one;
                        }
                        else if (b0 >= zero)  // V0
                        {
                            t0 = zero;
                        }
                        else  // E01
                        {
                            t0 = -b0 / a00;
                        }
                    }
                }
                else  // region 1
                {
                    numer = a11 + b1 - a01 - b0;
                    if (numer <= zero)  // V2
                    {
                        t0 = zero;
                        t1 = one;
                    }
                    else
                    {
                        denom = a00 - 2 * a01 + a11;
                        if (numer >= denom)  // V1
                        {
                            t0 = one;
                            t1 = zero;
                        }
                        else  // 12
                        {
                            t0 = numer / denom;
                            t1 = one - t0;
                        }
                    }
                }
            }

            result.x = p1.x + t0 * e0x + t1 * e1x;
            result.y = p1.y + t0 * e0y + t1 * e1y;
            result.z = p1.z + t0 * e0z + t1 * e1z;
        }

        /**
         * Calculates the area of a triangle.
         */
        public static float TriangleArea(Vector3 p1, Vector3 p2, Vector3 p3)
        {
            return Mathf.Sqrt(Vector3.Cross(p2 - p1, p3 - p1).sqrMagnitude) / 2f;
        }

        public static float EllipsoidVolume(Vector3 principalRadii)
        {
            return 4.0f / 3.0f * Mathf.PI * principalRadii.x * principalRadii.y * principalRadii.z;
        }

        public static Quaternion RestDarboux(Quaternion q1, Quaternion q2)
        {
            Quaternion darboux = Quaternion.Inverse(q1) * q2;
            Vector4 omega_plus, omega_minus;
            omega_plus = new Vector4(darboux.w, darboux.x, darboux.y, darboux.z) + new Vector4(1, 0, 0, 0);
            omega_minus = new Vector4(darboux.w, darboux.x, darboux.y, darboux.z) - new Vector4(1, 0, 0, 0);
            if (omega_minus.sqrMagnitude > omega_plus.sqrMagnitude)
            {
                darboux = new Quaternion(darboux.x * -1, darboux.y * -1, darboux.z * -1, darboux.w * -1);
            }
            return darboux;
        }

        public static float RestBendingConstraint(Vector3 positionA, Vector3 positionB, Vector3 positionC)
        {
            Vector3 center = (positionA + positionB + positionC) / 3;
            return (positionC - center).magnitude;
        }

        public static System.Collections.IEnumerable BilateralInterleaved(int count)
        {
            for (int i = 0; i < count; ++i)
            {
                if (i % 2 != 0)
                    yield return count - (count % 2) - i;
                else yield return i;
            }
        }

        public static void BarycentricCoordinates(in Vector3 A,
                                                  in Vector3 B,
                                                  in Vector3 C,
                                                  in Vector3 P,
                                                  ref Vector3 bary)
        {

                // Compute vectors
                Vector3 v0 = C - A;
                Vector3 v1 = B - A;
                Vector3 v2 = P - A;

                // Compute dot products
                float dot00 = Vector3.Dot(v0,v0);
                float dot01 = Vector3.Dot(v0,v1);
                float dot02 = Vector3.Dot(v0,v2);
                float dot11 = Vector3.Dot(v1,v1);
                float dot12 = Vector3.Dot(v1,v2);

                // Compute barycentric coordinates
                float det = dot00 * dot11 - dot01 * dot01;
                if (Math.Abs(det) > 1E-38)
                {
                    float u = (dot11 * dot02 - dot01 * dot12) / det;
                    float v = (dot00 * dot12 - dot01 * dot02) / det;
                    bary = new Vector3(1-u-v,v,u);
                }

        }

        public static void BarycentricInterpolation(in Vector3 p1, in Vector3 p2, in Vector3 p3, in Vector3 coords, out Vector3 result)
        {
            result.x = coords.x * p1.x + coords.y * p2.x + coords.z * p3.x;
            result.y = coords.x * p1.y + coords.y * p2.y + coords.z * p3.y;
            result.z = coords.x * p1.z + coords.y * p2.z + coords.z * p3.z;
        }

        public static float BarycentricInterpolation(float p1, float p2, float p3, Vector3 coords)
        {
            return coords[0] * p1 + coords[1] * p2 + coords[2] * p3;
        }

        public static float BarycentricExtrapolationScale(Vector3 coords)
        {

            return 1.0f / (coords[0] * coords[0] +
                           coords[1] * coords[1] +
                           coords[2] * coords[2]);

        }

        public static Vector3[] CalculateAngleWeightedNormals(Vector3[] vertices, int[] triangles)
        {
            Vector3[] normals = new Vector3[vertices.Length];
            var normalBuffer = new Dictionary<Vector3, Vector3>();

            Vector3 v1, v2, v3, e1, e2;
            for (int i = 0; i < triangles.Length; i += 3)
            {
                v1 = vertices[triangles[i]];
                v2 = vertices[triangles[i + 1]];
                v3 = vertices[triangles[i + 2]];

                if (!normalBuffer.ContainsKey(v1))
                    normalBuffer[v1] = Vector3.zero;
                if (!normalBuffer.ContainsKey(v2))
                    normalBuffer[v2] = Vector3.zero;
                if (!normalBuffer.ContainsKey(v3))
                    normalBuffer[v3] = Vector3.zero;

                e1 = v2 - v1;
                e2 = v3 - v1;
                normalBuffer[v1] += Vector3.Cross(e1,e2).normalized * Mathf.Acos(Vector3.Dot(e1.normalized, e2.normalized));

                e1 = v3 - v2;
                e2 = v1 - v2;
                normalBuffer[v2] += Vector3.Cross(e1, e2).normalized * Mathf.Acos(Vector3.Dot(e1.normalized, e2.normalized));

                e1 = v1 - v3;
                e2 = v2 - v3;
                normalBuffer[v3] += Vector3.Cross(e1, e2).normalized * Mathf.Acos(Vector3.Dot(e1.normalized, e2.normalized));
            }

            for (int i = 0; i < vertices.Length; ++i)
                normals[i] = normalBuffer[vertices[i]].normalized;

            return normals;
        }

        public static int MakePhase(int group, Oni.ParticleFlags flags)
        {
            return (group & (int)Oni.ParticleFlags.GroupMask) | (int)flags;
        }

        public static int GetGroupFromPhase(int phase)
        {
            return phase & (int)Oni.ParticleFlags.GroupMask;
        }

        public static int GetFlagsFromPhase(int phase)
        {
            return phase & ~(int)Oni.ParticleFlags.GroupMask;
        }

        public static void EigenSolve(Matrix4x4 D, out Vector3 S, out Matrix4x4 V)
        {
            // D is symmetric
            // S is a vector whose elements are eigenvalues
            // V is a matrix whose columns are eigenvectors
            S = EigenValues(D);
            Vector3 V0, V1, V2;

            if (S[0] - S[1] > S[1] - S[2])
            {
                V0 = EigenVector(D, S[0]);
                if (S[1] - S[2] < Mathf.Epsilon)
                {
                    V2 = V0.unitOrthogonal();
                }
                else
                {
                    V2 = EigenVector(D, S[2]); V2 -= V0 * Vector3.Dot(V0, V2); V2 = Vector3.Normalize(V2);
                }
                V1 = Vector3.Cross(V2, V0);
            }
            else
            {
                V2 = EigenVector(D, S[2]);
                if (S[0] - S[1] < Mathf.Epsilon)
                {
                    V1 = V2.unitOrthogonal();
                }
                else
                {
                    V1 = EigenVector(D, S[1]); V1 -= V2 * Vector3.Dot(V2, V1); V1 = Vector3.Normalize(V1);
                }
                V0 = Vector3.Cross(V1, V2);
            }

            V = Matrix4x4.identity;
            V.SetColumn(0,V0);
            V.SetColumn(1,V1);
            V.SetColumn(2,V2);
        }

        static Vector3 unitOrthogonal(this Vector3 input)
        {
            // Find a vector to cross() the input with.
            if (!(input.x < input.z * Mathf.Epsilon)
             || !(input.y < input.z * Mathf.Epsilon))
            {
                float invnm = 1 / Vector3.Magnitude(new Vector2(input.x,input.y));
                return new Vector3(-input.y * invnm, input.x * invnm, 0);
            }
            else
            {
                float invnm = 1 / Vector3.Magnitude(new Vector2(input.y,input.z));
                return new Vector3(0, -input.z * invnm, input.y * invnm);
            }
        }

        // D is symmetric, S is an eigen value
        static Vector3 EigenVector(Matrix4x4 D, float S)
        {
            // Compute a cofactor matrix of D - sI.
            Vector4 c0 = D.GetColumn(0); c0[0] -= S;
            Vector4 c1 = D.GetColumn(1); c1[1] -= S;
            Vector4 c2 = D.GetColumn(2); c2[2] -= S;

            // Use an upper triangle
            Vector3 c0p = new Vector3(c1[1] * c2[2] - c2[1] * c2[1], 0, 0);
            Vector3 c1p = new Vector3(c2[1] * c2[0] - c1[0] * c2[2], c0[0] * c2[2] - c2[0] * c2[0], 0);
            Vector3 c2p = new Vector3(c1[0] * c2[1] - c1[1] * c2[0], c1[0] * c2[0] - c0[0] * c2[1], c0[0] * c1[1] - c1[0] * c1[0]);

            // Get a column vector with a largest norm (non-zero).
            float C01s = c1p[0] * c1p[0];
            float C02s = c2p[0] * c2p[0];
            float C12s = c2p[1] * c2p[1];
            Vector3 norm = new Vector3(c0p[0] * c0p[0] + C01s + C02s,
                                       C01s + c1p[1] * c1p[1] + C12s,
                                       C02s + C12s + c2p[2] * c2p[2]);

            // index of largest:
            int index = 0;
            if (norm[0] > norm[1] && norm[0] > norm[2])
                index = 0;
            else if (norm[1] > norm[0] && norm[1] > norm[2])
                index = 1;
            else
                index = 2;

            Vector3 V = Vector3.zero;

            // special case
            if (norm[index] < Mathf.Epsilon)
            {
                V[0] = 1; return V;
            }
            else if (index == 0)
            {
                V[0] = c0p[0]; V[1] = c1p[0]; V[2] = c2p[0];
            }
            else if (index == 1)
            {
                V[0] = c1p[0]; V[1] = c1p[1]; V[2] = c2p[1];
            }
            else
            {
                V = c2p;
            }
            return Vector3.Normalize(V);
        }

        static Vector3 EigenValues(Matrix4x4 D)
        {
            float one_third = 1 / 3.0f;
            float one_sixth = 1 / 6.0f;
            float three_sqrt = Mathf.Sqrt(3.0f);

            Vector3 c0 = D.GetColumn(0);
            Vector3 c1 = D.GetColumn(1);
            Vector3 c2 = D.GetColumn(2);

            float m = one_third * (c0[0] + c1[1] + c2[2]);

            // K is D - I*diag(S)
            float K00 = c0[0] - m;
            float K11 = c1[1] - m;
            float K22 = c2[2] - m;

            float K01s = c1[0] * c1[0];
            float K02s = c2[0] * c2[0];
            float K12s = c2[1] * c2[1];

            float q = 0.5f * (K00 * (K11 * K22 - K12s) - K22 * K01s - K11 * K02s) + c1[0] * c2[1] * c0[2];
            float p = one_sixth * (K00 * K00 + K11 * K11 + K22 * K22 + 2 * (K01s + K02s + K12s));

            float p_sqrt = Mathf.Sqrt(p);

            float tmp = p * p * p - q * q;
            float phi = one_third * Mathf.Atan2(Mathf.Sqrt(Mathf.Max(0, tmp)), q);
            float phi_c = Mathf.Cos(phi);
            float phi_s = Mathf.Sin(phi);
            float sqrt_p_c_phi = p_sqrt * phi_c;
            float sqrt_p_3_s_phi = p_sqrt * three_sqrt * phi_s;

            float e0 = m + 2 * sqrt_p_c_phi;
            float e1 = m - sqrt_p_c_phi - sqrt_p_3_s_phi;
            float e2 = m - sqrt_p_c_phi + sqrt_p_3_s_phi;

            float aux;
            if (e0 > e1)
            {
                aux = e0;
                e0 = e1;
                e1 = aux;
            }
            if (e0 > e2)
            {
                aux = e0;
                e0 = e2;
                e2 = aux;
            }
            if (e1 > e2)
            {
                aux = e1;
                e1 = e2;
                e2 = aux;
            }

            return new Vector3(e2, e1, e0);
        }

        public static Vector3 GetPointCloudCentroid(List<Vector3> points)
        {
            Vector3 centroid = Vector3.zero;
            for (int i = 0; i < points.Count; ++i)
                centroid += points[i];
            return centroid / points.Count;
        }

        public static void GetPointCloudAnisotropy(List<Vector3> points, float max_anisotropy, float radius, ref Vector3 hint_normal, ref Vector3 centroid, ref Quaternion orientation, ref Vector3 principal_radii)
        {
            int count = points.Count;
            if (count == 0|| radius <= 0 || max_anisotropy <= 0)
            {
                principal_radii = Vector3.one * radius;
                orientation = Quaternion.identity;
                return;
            }

            centroid = GetPointCloudCentroid(points);

            // three columns of a 3x3 anisotropy matrix:
            Vector4 c0 = Vector4.zero,
                    c1 = Vector4.zero,
                    c2 = Vector4.zero;

            Matrix4x4 anisotropy = Matrix4x4.zero;

            // multiply offset by offset transposed, add to matrix, and average.
            for (int i = 0; i < count; i++)
            {
                Vector4 offset = points[i] - centroid;
                c0 += offset * offset[0];
                c1 += offset * offset[1];
                c2 += offset * offset[2];
            }

            anisotropy.SetColumn(0, c0 / count);
            anisotropy.SetColumn(1, c1 / count);
            anisotropy.SetColumn(2, c2 / count);

            Matrix4x4 orientMat;
            EigenSolve(anisotropy, out principal_radii, out orientMat);

            // flip orientation if it is not in the same side as the hint normal:
            if (Vector3.Dot(orientMat.GetColumn(2),hint_normal) < 0)
            {
                orientMat.SetColumn(2,orientMat.GetColumn(2) * -1);
                orientMat.SetColumn(1,orientMat.GetColumn(1) * -1);
            }

            float max = principal_radii[0];
            principal_radii = Vector3.Max(principal_radii,Vector3.one * max/max_anisotropy) / max * radius;
            orientation = orientMat.rotation;
        }
    }
}

