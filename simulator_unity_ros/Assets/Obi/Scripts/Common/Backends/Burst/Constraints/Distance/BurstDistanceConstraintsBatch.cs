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
    public class BurstDistanceConstraintsBatch : BurstConstraintsBatchImpl, IDistanceConstraintsBatchImpl
    {
        private NativeArray<float> restLengths;
        private NativeArray<float2> stiffnesses;

        public BurstDistanceConstraintsBatch(BurstDistanceConstraints constraints)
        {
            m_Constraints = constraints;
            m_ConstraintType = Oni.ConstraintType.Distance; 
        }

        public void SetDistanceConstraints(ObiNativeIntList particleIndices, ObiNativeFloatList restLengths, ObiNativeVector2List stiffnesses, ObiNativeFloatList lambdas, int count)
        {
            this.particleIndices = particleIndices.AsNativeArray<int>();
            this.restLengths = restLengths.AsNativeArray<float>();
            this.stiffnesses = stiffnesses.AsNativeArray<float2>();
            this.lambdas = lambdas.AsNativeArray<float>();
            m_ConstraintCount = count;
        }

        public override JobHandle Evaluate(JobHandle inputDeps, float deltaTime)
        {
            var projectConstraints = new DistanceConstraintsBatchJob()
            {
                particleIndices = particleIndices,
                restLengths = restLengths,
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

            var applyConstraints = new ApplyDistanceConstraintsBatchJob()
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
        public struct DistanceConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] public NativeArray<float> restLengths;
            [ReadOnly] public NativeArray<float2> stiffnesses;
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
                float compliance = stiffnesses[i].x / deltaTimeSqr;

                // calculate position and lambda deltas:
                float4 distance = positions[p1] - positions[p2];
                float d = math.length(distance);

                // calculate constraint value:
                float constraint = d - restLengths[i];
                constraint -= math.max(math.min(constraint, 0), -stiffnesses[i].y);

                // calculate lambda and position deltas:
                float dlambda = (-constraint - compliance * lambdas[i]) / (w1 + w2 + compliance + BurstMath.epsilon);
                float4 delta = dlambda * distance / (d + BurstMath.epsilon);

                lambdas[i] += dlambda;

                deltas[p1] += delta * w1;
                deltas[p2] -= delta * w2;

                counts[p1]++;
                counts[p2]++;
            }
        }

        [BurstCompile]
        public struct ApplyDistanceConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] public float sorFactor;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> positions;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> counts;

            public void Execute(int i)
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
#endif