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
    public class BurstTetherConstraintsBatch : BurstConstraintsBatchImpl, ITetherConstraintsBatchImpl
    {
        private NativeArray<float2> maxLengthScale;
        private NativeArray<float> stiffnesses;

        public BurstTetherConstraintsBatch(BurstTetherConstraints constraints)
        {
            m_Constraints = constraints;
            m_ConstraintType = Oni.ConstraintType.Tether;
        }

        public void SetTetherConstraints(ObiNativeIntList particleIndices, ObiNativeVector2List maxLengthScale, ObiNativeFloatList stiffnesses, ObiNativeFloatList lambdas, int count)
        {
            this.particleIndices = particleIndices.AsNativeArray<int>();
            this.maxLengthScale = maxLengthScale.AsNativeArray<float2>();
            this.stiffnesses = stiffnesses.AsNativeArray<float>();
            this.lambdas = lambdas.AsNativeArray<float>();
            m_ConstraintCount = count;
        }

        public override JobHandle Evaluate(JobHandle inputDeps, float deltaTime)
        {
            var projectConstraints = new TetherConstraintsBatchJob()
            {
                particleIndices = particleIndices,
                maxLengthScale = maxLengthScale,
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

            var applyConstraints = new ApplyTetherConstraintsBatchJob()
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
        public struct TetherConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] public NativeArray<float2> maxLengthScale;
            [ReadOnly] public NativeArray<float> stiffnesses;
            public NativeArray<float> lambdas;

            [ReadOnly] public NativeArray<float4> positions;
            [ReadOnly] public NativeArray<float> invMasses;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> counts;

            [ReadOnly] public float deltaTimeSqr;

            public void Execute(int i)
            {
                int p1 = particleIndices[i * 2];
                int p2 = particleIndices[i * 2 + 1];

                float w1 = invMasses[p1];
                float w2 = invMasses[p2];

                // calculate time adjusted compliance
                float compliance = stiffnesses[i] / deltaTimeSqr;

                // calculate position and lambda deltas:
                float4 distance = positions[p1] - positions[p2];
                float d = math.length(distance);

                // calculate constraint value (distance - rest length)
                float constraint = d - (maxLengthScale[i].x * maxLengthScale[i].y);

                if (constraint > 0)
                {
                    // calculate lambda and position deltas:
                    float dlambda = (-constraint - compliance * lambdas[i]) / (w1 + w2 + compliance + BurstMath.epsilon);
                    float4 delta = dlambda * distance / (d + BurstMath.epsilon);
                    lambdas[i] += dlambda;
                    deltas[p1] += delta * w1;
                    counts[p1]++;
                }
            }
        }

        [BurstCompile]
        public struct ApplyTetherConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeSlice<int> particleIndices;

            // linear/position properties:
            [NativeDisableParallelForRestriction] public NativeArray<float4> positions;
            [NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableParallelForRestriction] public NativeArray<int> counts;

            [ReadOnly] public float sorFactor;

            public void Execute(int index)
            {
                // only the first particle out of each pair is affected:
                int i = particleIndices[index * 2];

                if (counts[i] > 0)
                {
                    positions[i] += deltas[i] * sorFactor / counts[i];
                    deltas[i] = float4.zero;
                    counts[i] = 0;
                }
            }
        }
    }
}
#endif