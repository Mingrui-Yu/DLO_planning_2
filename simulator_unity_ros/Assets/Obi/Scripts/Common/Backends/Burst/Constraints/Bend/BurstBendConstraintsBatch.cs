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
    public class BurstBendConstraintsBatch : BurstConstraintsBatchImpl, IBendConstraintsBatchImpl
    {
        private NativeArray<float> restBends;
        private NativeArray<float2> stiffnesses;

        public BurstBendConstraintsBatch(BurstBendConstraints constraints)
        {
            m_Constraints = constraints;
            m_ConstraintType = Oni.ConstraintType.Bending;
        }

        public void SetBendConstraints(ObiNativeIntList particleIndices, ObiNativeFloatList restBends, ObiNativeVector2List bendingStiffnesses, ObiNativeFloatList lambdas, int count)
        {
            this.particleIndices = particleIndices.AsNativeArray<int>();
            this.restBends = restBends.AsNativeArray<float>();
            this.stiffnesses = bendingStiffnesses.AsNativeArray<float2>();
            this.lambdas = lambdas.AsNativeArray<float>();
            m_ConstraintCount = count;
        }

        public override JobHandle Evaluate(JobHandle inputDeps, float deltaTime)
        {
            var projectConstraints = new BendConstraintsBatchJob()
            {
                particleIndices = particleIndices,
                restBends = restBends,
                stiffnesses = stiffnesses,
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

            var applyConstraints = new ApplyBendConstraintsBatchJob()
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
        public struct BendConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] public NativeArray<float> restBends;
            [ReadOnly] public NativeArray<float2> stiffnesses;
            public NativeArray<float> lambdas;

            [ReadOnly] public NativeArray<float4> positions;
            [ReadOnly] public NativeArray<float> invMasses;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> counts;

            [ReadOnly] public float deltaTimeSqr;

            public void Execute(int i)
            {
                int p1 = particleIndices[i * 3];
                int p2 = particleIndices[i * 3 + 1];
                int p3 = particleIndices[i * 3 + 2];

                float w1 = invMasses[p1];
                float w2 = invMasses[p2];
                float w3 = invMasses[p3];

                float wsum = w1 + w2 + 2 * w3;
                if (wsum > 0)
                { 
                    float4 bendVector = positions[p3] - (positions[p1] + positions[p2] + positions[p3]) / 3.0f;
                    float bend = math.length(bendVector);

                    if (bend > 0)
                    {
                        float constraint = 1.0f - (stiffnesses[i].x + restBends[i]) / bend;

                        // remove this to force a certain curvature.
                        if (constraint >= 0)
                        {
                            // calculate time adjusted compliance
                            float compliance = stiffnesses[i].y / deltaTimeSqr;

                            // since the third particle moves twice the amount of the other 2, the modulus of its gradient is 2:
                            float dlambda = (-constraint - compliance * lambdas[i]) / (wsum + compliance + BurstMath.epsilon);
                            float4 correction = dlambda * bendVector;

                            lambdas[i] += dlambda;

                            deltas[p1] -= correction * 2 * w1;
                            deltas[p2] -= correction * 2 * w2;
                            deltas[p3] += correction * 4 * w3;

                            counts[p1]++;
                            counts[p2]++;
                            counts[p3]++;
                        }
                    }
                }
            }
        }

        [BurstCompile]
        public struct ApplyBendConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] public float sorFactor;

            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> positions;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<int> counts;

            public void Execute(int i)
            {
                int p1 = particleIndices[i * 3];
                int p2 = particleIndices[i * 3 + 1];
                int p3 = particleIndices[i * 3 + 2];

                if (counts[p1] > 0)
                {
                    positions[p1] += deltas[p1] * sorFactor / counts[p1];
                    deltas[p1] = float4.zero;
                    counts[p1] = 0;
                }

                if (counts[p2] > 0)
                {
                    positions[p2] += deltas[p2] * sorFactor / counts[p2];
                    deltas[p2] = float4.zero;
                    counts[p2] = 0;
                }

                if (counts[p3] > 0)
                {
                    positions[p3] += deltas[p3] * sorFactor / counts[p3];
                    deltas[p3] = float4.zero;
                    counts[p3] = 0;
                }
            }
        }
    }
}
#endif