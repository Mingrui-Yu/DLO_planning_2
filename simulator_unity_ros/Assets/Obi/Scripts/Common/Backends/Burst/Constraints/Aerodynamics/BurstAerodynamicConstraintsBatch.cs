#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using UnityEngine;
using Unity.Jobs;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Burst;
using System.Collections;

namespace Obi
{
    public class BurstAerodynamicConstraintsBatch : BurstConstraintsBatchImpl, IAerodynamicConstraintsBatchImpl
    {
        private NativeArray<float> aerodynamicCoeffs;

        public BurstAerodynamicConstraintsBatch(BurstAerodynamicConstraints constraints)
        {
            m_Constraints = constraints;
            m_ConstraintType = Oni.ConstraintType.Aerodynamics;
        }

        public void SetAerodynamicConstraints(ObiNativeIntList particleIndices, ObiNativeFloatList aerodynamicCoeffs, int count)
        {
            this.particleIndices = particleIndices.AsNativeArray<int>();
            this.aerodynamicCoeffs = aerodynamicCoeffs.AsNativeArray<float>();
            m_ConstraintCount = count;
        }

        public override JobHandle Initialize(JobHandle inputDeps, float deltaTime)
        {
            return inputDeps;
        }

        public override JobHandle Evaluate(JobHandle inputDeps, float deltaTime)
        {
            var projectConstraints = new AerodynamicConstraintsBatchJob()
            {
                particleIndices = particleIndices,
                aerodynamicCoeffs = aerodynamicCoeffs,
                positions = solverImplementation.positions,
                velocities = solverImplementation.velocities,
                normals = solverImplementation.normals,
                wind = solverImplementation.wind,
                invMasses = solverImplementation.invMasses,
                deltaTime = deltaTime
            };

            return projectConstraints.Schedule(m_ConstraintCount, 32, inputDeps);
        }

        public override JobHandle Apply(JobHandle inputDeps, float deltaTime)
        {
            return inputDeps;
        }

        [BurstCompile]
        public struct AerodynamicConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] [NativeDisableParallelForRestriction] public NativeArray<float> aerodynamicCoeffs;

            [ReadOnly] public NativeArray<float4> positions; 
            [ReadOnly] public NativeArray<float4> normals;
            [ReadOnly] public NativeArray<float4> wind;
            [ReadOnly] public NativeArray<float> invMasses;

            [NativeDisableContainerSafetyRestriction]
            public NativeArray<float4> velocities;

            [ReadOnly] public float deltaTime;

            public void Execute(int i)
            {
                int p = particleIndices[i];

                float area = aerodynamicCoeffs[i * 3];
                float dragCoeff = aerodynamicCoeffs[i * 3 + 1];
                float liftCoeff = aerodynamicCoeffs[i * 3 + 2];

                if (invMasses[p] > 0)
                {
                    float4 relVelocity = velocities[p] - wind[p];
                    float rvSqrMag = math.lengthsq(relVelocity);

                    if (rvSqrMag < BurstMath.epsilon)
                        return;

                    float4 rvNorm = relVelocity / math.sqrt(rvSqrMag);

                    // calculate surface normal (always facing wind)
                    float4 surfNormal = normals[p] * math.sign(math.dot(normals[p], rvNorm));

                    // aerodynamic_factor was originally multiplied by air_density. The density is now premultiplied in lift and drag.
                    float aerodynamicFactor = 0.5f * rvSqrMag * area;
                    float attackAngle = math.dot(surfNormal,rvNorm);

                    float3 liftDirection = math.normalizesafe(math.cross(math.cross(surfNormal.xyz, rvNorm.xyz), rvNorm.xyz));

                    //drag:
                    velocities[p] += (-dragCoeff * rvNorm +

                                      // lift:
                                      liftCoeff * new float4(liftDirection.xyz,0)) *

                                      // scale
                                      attackAngle * math.min(aerodynamicFactor * invMasses[p] * deltaTime, 1000);
                }
            }
        }
    }
}
#endif