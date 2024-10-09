#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Burst;

namespace Obi
{
    public static class BurstMath
    {

        public const float epsilon = 0.0000001f;

        // multiplies a column vector by a row vector.
        public static float3x3 multrnsp(float4 column, float4 row)
        {
            return new float3x3(column[0] * row[0], column[0] * row[1], column[0] * row[2],
                                column[1] * row[0], column[1] * row[1], column[1] * row[2],
                                column[2] * row[0], column[2] * row[1], column[2] * row[2]);
        }

        // multiplies a column vector by a row vector.
        public static float4x4 multrnsp4(float4 column, float4 row)
        {
            return new float4x4(column[0] * row[0], column[0] * row[1], column[0] * row[2],0,
                                column[1] * row[0], column[1] * row[1], column[1] * row[2],0,
                                column[2] * row[0], column[2] * row[1], column[2] * row[2],0,
                                0,0,0,1);
        }

        public static float4 project(this float4 vector, float4 onto)
        {
            float len = math.lengthsq(onto);
            if (len < epsilon)
                return float4.zero;
            return math.dot(onto, vector) * onto / len;
        }

        public static float4x4 TransformInertiaTensor(float4 tensor, quaternion rotation)
        {
            float4x4 rotMatrix = rotation.toMatrix();
            return math.mul(rotMatrix, math.mul(tensor.asDiagonal(), math.transpose(rotMatrix)));
        }

        public static float RotationalInvMass(float4x4 inverseInertiaTensor, float4 r, float4 axis)
        {
            float4 cr = math.mul(inverseInertiaTensor, new float4(math.cross(r.xyz, axis.xyz), 0));
            return math.dot(math.cross(cr.xyz, r.xyz), axis.xyz);
        }

        public static float4 GetParticleVelocityAtPoint(float4 position, float4 prevPosition, float4 point, float dt)
        {
            // no angular velocity, so calculate and return linear velocity only:
            return BurstIntegration.DifferentiateLinear(position, prevPosition, dt);
        }

        public static float4 GetParticleVelocityAtPoint(float4 position, float4 prevPosition, quaternion orientation, quaternion prevOrientation, float4 point, float dt)
        {
            // calculate both linear and angular velocities:
            float4 linearVelocity = BurstIntegration.DifferentiateLinear(position, prevPosition, dt);
            float4 angularVelocity = BurstIntegration.DifferentiateAngular(orientation, prevOrientation, dt);

            return linearVelocity + new float4(math.cross(angularVelocity.xyz, (point - prevPosition).xyz), 0);
        }

        public static float4 GetRigidbodyVelocityAtPoint(BurstRigidbody rigidbody, float4 point, float4 linearDelta, float4 angularDelta, BurstAffineTransform transform)
        {
            // Point is assumed to be expressed in solver space. Since rigidbodies are expressed in world space, we need to convert the
            // point to world space, and convert the resulting velocity back to solver space.
            return transform.InverseTransformVector(rigidbody.GetVelocityAtPoint(transform.TransformPoint(point), linearDelta , angularDelta));
        }

        public static float EllipsoidRadius(float4 normSolverDirection, quaternion orientation, float3 radii)
        {
            float3 localDir = math.mul(math.conjugate(orientation), normSolverDirection.xyz);
            float sqrNorm = math.lengthsq(localDir / radii);
            return sqrNorm > epsilon ? math.sqrt(1 / sqrNorm) : radii.x;
        }

        public static quaternion ExtractRotation(float4x4 matrix, quaternion rotation, int iterations)
        {
            float4x4 R;
            for (int i = 0; i < iterations; ++i)
            {
                R = rotation.toMatrix();
                float3 omega = (math.cross(R.c0.xyz, matrix.c0.xyz) + math.cross(R.c1.xyz, matrix.c1.xyz) + math.cross(R.c2.xyz, matrix.c2.xyz)) /
                               (math.abs(math.dot(R.c0.xyz, matrix.c0.xyz) + math.dot(R.c1.xyz, matrix.c1.xyz) + math.dot(R.c2.xyz, matrix.c2.xyz)) + BurstMath.epsilon);

                float w = math.length(omega);
                if (w < BurstMath.epsilon)
                    break;

                rotation = math.normalize(math.mul(quaternion.AxisAngle((1.0f/w) * omega,w),rotation));
            }
            return rotation;
        }

        // decomposes a quaternion in swing and twist around a given axis:
        public static void SwingTwist(quaternion q, float3 vt, out quaternion swing, out quaternion twist)
        {
            float3 p = vt * math.dot(q.value.xyz,vt);
            twist = math.normalize(new quaternion(p[0], p[1], p[2], q.value.w));
            swing = math.mul(q, math.conjugate(twist));
        }

        public static float4x4 toMatrix(this quaternion q)
        {
            float xx = q.value.x * q.value.x;
            float xy = q.value.x * q.value.y;
            float xz = q.value.x * q.value.z;
            float xw = q.value.x * q.value.w;

            float yy = q.value.y * q.value.y;
            float yz = q.value.y * q.value.z;
            float yw = q.value.y * q.value.w;

            float zz = q.value.z * q.value.z;
            float zw = q.value.z * q.value.w;

            return new float4x4(1-2*(yy+zz), 2*(xy-zw), 2*(xz+yw),0,
                                2*(xy+zw), 1-2*(xx+zz), 2*(yz-xw),0,
                                2*(xz-yw),2*(yz+xw),1-2*(xx+yy),0,
                                0,0,0,1);
        }

        public static float4x4 asDiagonal(this float4 v)
        {
            return new float4x4(v.x, 0, 0, 0,
                                0, v.y, 0, 0,
                                0, 0, v.z, 0,
                                0, 0, 0, v.w);
        }

        public static float4 diagonal(this float4x4 value)
        {
            return new float4(value.c0[0], value.c1[1], value.c2[2], value.c3[3]);
        }

        public static float frobeniusNorm(this float4x4 m)
        {
            return math.sqrt(math.lengthsq(m.c0) + math.lengthsq(m.c1) + math.lengthsq(m.c2) + math.lengthsq(m.c3));
        }

        public static void EigenSolve(float3x3 D, out float3 S, out float3x3 V)
        {
            // D is symmetric
            // S is a vector whose elements are eigenvalues
            // V is a matrix whose columns are eigenvectors
            S = EigenValues(D);
            float3 V0, V1, V2;

            if (S[0] - S[1] > S[1] - S[2])
            {
                V0 = EigenVector(D, S[0]);
                if (S[1] - S[2] < math.FLT_MIN_NORMAL)
                {
                    V2 = V0.unitOrthogonal();
                }
                else
                {
                    V2 = EigenVector(D, S[2]); V2 -= V0 * math.dot(V0, V2); V2 = math.normalize(V2);
                }
                V1 = math.cross(V2, V0);
            }
            else
            {
                V2 = EigenVector(D, S[2]);
                if (S[0] - S[1] < math.FLT_MIN_NORMAL)
                {
                    V1 = V2.unitOrthogonal();
                }
                else
                {
                    V1 = EigenVector(D, S[1]); V1 -= V2 * math.dot(V2, V1); V1 = math.normalize(V1);
                }
                V0 = math.cross(V1, V2);
            }

            V.c0 = V0;
            V.c1 = V1;
            V.c2 = V2;
        }

        static float3 unitOrthogonal(this float3 input)
        {
            // Find a vector to cross() the input with.
            if (!(input.x < input.z * BurstMath.epsilon)
            ||  !(input.y < input.z * BurstMath.epsilon))
            {
                float invnm = 1 / math.length(input.xy);
                return new float3(-input.y * invnm, input.x * invnm, 0);
            }
            else
            {
                float invnm = 1 / math.length(input.yz);
                return new float3(0,-input.z * invnm, input.y * invnm);
            }
        }

        // D is symmetric, S is an eigen value
        static float3 EigenVector(float3x3 D, float S)
        {
            // Compute a cofactor matrix of D - sI.
            float3 c0 = D.c0; c0[0] -= S;
            float3 c1 = D.c1; c1[1] -= S;
            float3 c2 = D.c2; c2[2] -= S;

            // Upper triangular matrix
            float3 c0p = new float3(c1[1] * c2[2] - c2[1] * c2[1], 0, 0);
            float3 c1p = new float3(c2[1] * c2[0] - c1[0] * c2[2], c0[0] * c2[2] - c2[0] * c2[0], 0);
            float3 c2p = new float3(c1[0] * c2[1] - c1[1] * c2[0], c1[0] * c2[0] - c0[0] * c2[1], c0[0] * c1[1] - c1[0] * c1[0]);

            // Get a column vector with a largest norm (non-zero).
            float C01s = c1p[0] * c1p[0];
            float C02s = c2p[0] * c2p[0];
            float C12s = c2p[1] * c2p[1];
            float3 norm = new float3(c0p[0]*c0p[0] + C01s + C02s,
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

            float3 V = float3.zero;

            // special case
            if (norm[index] < math.FLT_MIN_NORMAL)
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
            return math.normalize(V);
        }

        static float3 EigenValues(float3x3 D)
        {
            float one_third = 1 / 3.0f;
            float one_sixth = 1 / 6.0f;
            float three_sqrt = math.sqrt(3.0f);

            float3 c0 = D.c0;
            float3 c1 = D.c1;
            float3 c2 = D.c2;

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

            float p_sqrt = math.sqrt(p);

            float tmp = p * p * p - q * q;
            float phi = one_third * math.atan2(math.sqrt(math.max(0, tmp)), q);
            float phi_c = math.cos(phi);
            float phi_s = math.sin(phi);
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

            return new float3(e2, e1, e0);
        }

        public static float4 NearestPointOnTri(float4 p1,
                                                float4 p2,
                                                float4 p3,
                                                float4 p)
        {

            float4 edge0 = p2 - p1;
            float4 edge1 = p3 - p1;
            float4 v0 = p1 - p;

            float a00 = math.dot(edge0, edge0);
            float a01 = math.dot(edge0, edge1);
            float a11 = math.dot(edge1, edge1);
            float b0 = math.dot(edge0, v0);
            float b1 = math.dot(edge1, v0);

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

            return p1 + t0 * edge0 + t1 * edge1;
        }

        public static float4 NearestPointOnEdge(float4 p1,float4 p2,float4 p)
        {
        
            float4 edge = p2 - p1;

            // test if before first point:
            float4 v0 = p - p1;
            float v0_dot_edge = math.dot(v0,edge);
            if (v0_dot_edge <= 0.0)
                return p1;

            // test if after second point:
            float4 v1 = p - p2;
            if (math.dot(v1,edge) >= 0.0)
                return p2 ;
        
            // return projection:
            return p1 + v0_dot_edge / math.dot(edge,edge) * edge;

        }
    }
}
#endif