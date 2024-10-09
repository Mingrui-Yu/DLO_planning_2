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
    public class BurstStitchConstraintsBatch : BurstConstraintsBatchImpl, IStitchConstraintsBatchImpl
    {
        private NativeArray<float> stiffnesses;

        public BurstStitchConstraintsBatch(BurstStitchConstraints constraints)
        {
            m_Constraints = constraints;
            m_ConstraintType = Oni.ConstraintType.Stitch;
        }

        public void SetStitchConstraints(ObiNativeIntList particleIndices, ObiNativeFloatList stiffnesses, ObiNativeFloatList lambdas, int count)
        {
            this.particleIndices = particleIndices.AsNativeArray<int>();
            this.stiffnesses = stiffnesses.AsNativeArray<float>();
            this.lambdas = lambdas.AsNativeArray<float>();
            m_ConstraintCount = count;
        }

        public override JobHandle Evaluate(JobHandle inputDeps, float deltaTime)
        {
            var projectConstraints = new StitchConstraintsBatchJob()
            {
                particleIndices = particleIndices,
                stiffnesses = stiffnesses,
                lambdas = lambdas,
                positions = solverImplementation.positions,
                invMasses = solverImplementation.invMasses,
                deltas = solverImplementation.positionDeltas,
                counts = solverImplementation.positionConstraintCounts,
                deltaTimeSqr = deltaTime * deltaTime,
                activeConstraintCount = m_ConstraintCount
            };

            return projectConstraints.Schedule(inputDeps);
        }

        public override JobHandle Apply(JobHandle inputDeps, float deltaTime)
        {
            var parameters = solverAbstraction.GetConstraintParameters(m_ConstraintType);

            var applyConstraints = new ApplyStitchConstraintsBatchJob()
            {
                particleIndices = particleIndices,

                positions = solverImplementation.positions,
                deltas = solverImplementation.positionDeltas,
                counts = solverImplementation.positionConstraintCounts,

                sorFactor = parameters.SORFactor,
                activeConstraintCount = m_ConstraintCount
            };

            return applyConstraints.Schedule(inputDeps);
        }

        [BurstCompile]
        public struct StitchConstraintsBatchJob : IJob
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] public NativeArray<float> stiffnesses;
            public NativeArray<float> lambdas;

            [ReadOnly] public NativeArray<float4> positions;
            [ReadOnly] public NativeArray<float> invMasses;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> counts;

            [ReadOnly] public float deltaTimeSqr;
            [ReadOnly] public int activeConstraintCount;

            public void Execute()
            {
                for (int i = 0; i < activeConstraintCount; ++i)
                {
                    int p1 = particleIndices[i * 2];
                    int p2 = particleIndices[i * 2 + 1];

                    float w1 = invMasses[p1];
                    float w2 = invMasses[p2];

                    // calculate time adjusted compliance
                    float compliance = stiffnesses[i] / deltaTimeSqr;

                    // calculate position and lambda deltas:
                    float4 distance = positions[p1] - positions[p2];
                    float constraint = math.length(distance);

                    // calculate lambda and position deltas:
                    float dlambda = (-constraint - compliance * lambdas[i]) / (w1 + w2 + compliance + BurstMath.epsilon);
                    float4 delta = dlambda * distance / (constraint + BurstMath.epsilon);

                    lambdas[i] += dlambda;

                    deltas[p1] += delta * w1;
                    deltas[p2] -= delta * w2;

                    counts[p1]++;
                    counts[p2]++;
                }
            }
        }

        [BurstCompile]
        public struct ApplyStitchConstraintsBatchJob : IJob
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] public float sorFactor;

            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> positions;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<int> counts;

            [ReadOnly] public int activeConstraintCount;

            public void Execute()
            {
                for (int i = 0; i < activeConstraintCount; ++i)
                {
                    int p1 = particleIndices[i * 2];
                    int p2 = particleIndices[i * 2 + 1];

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
                }
            }
        }
    }
}
#endif