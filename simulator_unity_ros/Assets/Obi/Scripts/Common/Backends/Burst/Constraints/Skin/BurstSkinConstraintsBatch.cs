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
    public class BurstSkinConstraintsBatch : BurstConstraintsBatchImpl, ISkinConstraintsBatchImpl
    {
        private NativeArray<float4> skinPoints;
        private NativeArray<float4> skinNormals;
        private NativeArray<float> skinRadiiBackstop;
        private NativeArray<float> skinCompliance;

        public BurstSkinConstraintsBatch(BurstSkinConstraints constraints)
        {
            m_Constraints = constraints;
            m_ConstraintType = Oni.ConstraintType.Skin;
        }

        public void SetSkinConstraints(ObiNativeIntList particleIndices, ObiNativeVector4List skinPoints, ObiNativeVector4List skinNormals, ObiNativeFloatList skinRadiiBackstop, ObiNativeFloatList skinCompliance, ObiNativeFloatList lambdas, int count)
        {
            this.particleIndices = particleIndices.AsNativeArray<int>();
            this.skinPoints = skinPoints.AsNativeArray<float4>();
            this.skinNormals = skinNormals.AsNativeArray<float4>();
            this.skinRadiiBackstop = skinRadiiBackstop.AsNativeArray<float>();
            this.skinCompliance = skinCompliance.AsNativeArray<float>();
            this.lambdas = lambdas.AsNativeArray<float>();
            m_ConstraintCount = count;
        }

        public override JobHandle Evaluate(JobHandle inputDeps, float deltaTime)
        {
            var projectConstraints = new SkinConstraintsBatchJob()
            {
                particleIndices = particleIndices,
                skinPoints = skinPoints,
                skinNormals = skinNormals,
                skinRadiiBackstop = skinRadiiBackstop.Reinterpret<float, float3>(),
                skinCompliance = skinCompliance,
                lambdas = lambdas,
                positions = solverImplementation.positions,
                invMasses = solverImplementation.invMasses,
                deltas = solverImplementation.positionDeltas,
                counts = solverImplementation.positionConstraintCounts,
                deltaTimeSqr = deltaTime * deltaTime
            };

            return projectConstraints.Schedule(m_ConstraintCount, 32, inputDeps);
        }

        public override JobHandle Apply(JobHandle inputDeps, float deltaTime)
        {
            var parameters = solverAbstraction.GetConstraintParameters(m_ConstraintType);

            var applyConstraints = new ApplySkinConstraintsBatchJob()
            {
                particleIndices = particleIndices,

                positions = solverImplementation.positions,
                deltas = solverImplementation.positionDeltas,
                counts = solverImplementation.positionConstraintCounts,

                sorFactor = parameters.SORFactor
            };

            return applyConstraints.Schedule(m_ConstraintCount, 64, inputDeps);
        }

        [BurstCompile]
        public struct SkinConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] public NativeArray<float4> skinPoints;
            [ReadOnly] public NativeArray<float4> skinNormals;
            [ReadOnly] public NativeArray<float3> skinRadiiBackstop;
            [ReadOnly] public NativeArray<float> skinCompliance;
            public NativeArray<float> lambdas;

            [ReadOnly] public NativeArray<float4> positions;
            [ReadOnly] public NativeArray<float> invMasses;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> counts;

            [ReadOnly] public float deltaTimeSqr;

            public void Execute(int i)
            {
                float radius = skinRadiiBackstop[i].x;
                float collisionRadius = skinRadiiBackstop[i].y;
                float backstopDistance = collisionRadius + skinRadiiBackstop[i].z;

                float compliance = skinCompliance[i] / deltaTimeSqr;
                int p = particleIndices[i];

                if (invMasses[p] > 0)
                {
                    float4 toSkin = positions[p] - skinPoints[i];
                    float4 toBackstop = positions[p] - (skinPoints[i] - skinNormals[i] * backstopDistance);

                    // distance to skin and backstop sphere centers:
                    float d = math.length(toSkin);
                    float b = math.length(toBackstop);

                    // constrain particle within skin radius.
                    // ignore mass in the equations (use 1), as we don't want particle mass to interfere with skin compliance.
                    // We should be able to adjust skin properties and particle mass (for collisions) independently.
                    float constraint = math.max(0,d - radius);
                    float dlambda = (-constraint - compliance * lambdas[i]) / (1 + compliance); 
                    lambdas[i] += dlambda;
                    deltas[p] += dlambda * toSkin / (d + BurstMath.epsilon);
                    counts[p]++;

                    // constrain particle outside the backstop sphere (0 compliance):
                    constraint = math.min(0, b - collisionRadius);
                    deltas[p] -= constraint * toBackstop / (b + BurstMath.epsilon); 
                }

            }
        }

        [BurstCompile]
        public struct ApplySkinConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] public float sorFactor;

            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> positions;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<int> counts;

            public void Execute(int i)
            {
                int p1 = particleIndices[i];

                if (counts[p1] > 0)
                {
                    positions[p1] += deltas[p1] * sorFactor / counts[p1];
                    deltas[p1] = float4.zero;
                    counts[p1] = 0;
                }
            }
        }
    }
}
#endif