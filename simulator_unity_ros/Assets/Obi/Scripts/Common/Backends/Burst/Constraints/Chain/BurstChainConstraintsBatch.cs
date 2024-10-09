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
    public class BurstChainConstraintsBatch : BurstConstraintsBatchImpl, IChainConstraintsBatchImpl
    {
        private NativeArray<int> firstIndex;
        private NativeArray<int> numIndices;
        private NativeArray<float2> restLengths;

        public BurstChainConstraintsBatch(BurstChainConstraints constraints)
        {
            m_Constraints = constraints;
            m_ConstraintType = Oni.ConstraintType.Chain;
        }

        public void SetChainConstraints(ObiNativeIntList particleIndices, ObiNativeVector2List restLengths, ObiNativeIntList firstIndex, ObiNativeIntList numIndices, int count)
        {
            this.particleIndices = particleIndices.AsNativeArray<int>();
            this.firstIndex = firstIndex.AsNativeArray<int>();
            this.numIndices = numIndices.AsNativeArray<int>();
            this.restLengths = restLengths.AsNativeArray<float2>();
            m_ConstraintCount = count;
        }

        public override JobHandle Evaluate(JobHandle inputDeps, float deltaTime)
        {

            var projectConstraints = new ChainConstraintsBatchJob()
            {
                particleIndices = particleIndices,
                firstIndex = firstIndex,
                numIndices = numIndices,
                restLengths = restLengths,

                positions = solverImplementation.positions,
                invMasses = solverImplementation.invMasses,

                deltas = solverImplementation.positionDeltas,
                counts = solverImplementation.positionConstraintCounts
            };

            return projectConstraints.Schedule(m_ConstraintCount, 4, inputDeps);
        }

        public override JobHandle Apply(JobHandle inputDeps, float deltaTime)
        {
            var parameters = solverAbstraction.GetConstraintParameters(m_ConstraintType);

            var applyConstraints = new ApplyChainConstraintsBatchJob()
            {
                particleIndices = particleIndices,
                firstIndex = firstIndex,
                numIndices = numIndices,

                positions = solverImplementation.positions,
                deltas = solverImplementation.positionDeltas,
                counts = solverImplementation.positionConstraintCounts,

                sorFactor = parameters.SORFactor
            };

            return applyConstraints.Schedule(m_ConstraintCount, 8, inputDeps);
        }

        [BurstCompile]
        public struct ChainConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] public NativeArray<int> firstIndex;
            [ReadOnly] public NativeArray<int> numIndices;
            [ReadOnly] public NativeArray<float2> restLengths;

            [ReadOnly] public NativeArray<float4> positions;
            [ReadOnly] public NativeArray<float> invMasses;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> counts;

            public void Execute(int c)
            {
                int numEdges = numIndices[c] - 1;
                int first = firstIndex[c];
                float minLength = restLengths[c].y;
                float maxLength = restLengths[c].y;

                // (ni:constraint gradient, di:desired lenght)
                NativeArray<float4> ni = new NativeArray<float4>(numEdges, Allocator.Temp);
                NativeArray<float> di = new NativeArray<float>(numEdges, Allocator.Temp);

                for (int i = 0; i < numEdges; ++i)
                {
                    int edge = first + i;

                    float4 p1 = positions[particleIndices[edge]];
                    float4 p2 = positions[particleIndices[edge+1]];
                    float4 diff = p1 - p2;

                    float distance = math.length(diff);
                    float correction = 0;

                    if (distance >= maxLength)
                        correction = distance - maxLength;
                    else if (distance <= minLength)
                        correction = distance - minLength;

                    di[i] = correction;
                    ni[i] = new float4(diff/(distance + BurstMath.epsilon));
                }

                // calculate ai (subdiagonals), bi (diagonals) and ci (superdiagonals):
                NativeArray<float3> diagonals = new NativeArray<float3>(numEdges, Allocator.Temp);

                for (int i = 0; i < numEdges; ++i)
                {
                    int edge = first + i;

                    float w_i_ = invMasses[particleIndices[edge]];
                    float w__i = invMasses[particleIndices[edge+1]];

                    float4 ni__ = (i > 0) ? ni[i - 1] : float4.zero;
                    float4 n_i_ = ni[i];
                    float4 n__i = (i < numEdges - 1) ? ni[i + 1] : float4.zero;

                    diagonals[i] = new float3(
                                    -w_i_ * math.dot(n_i_, ni__), // ai
                                    w_i_ + w__i,                  // bi
                                    -w__i * math.dot(n_i_, n__i));// ci
                }

                NativeArray<float2> sweep = new NativeArray<float2>(numEdges, Allocator.Temp);

                // solve step #1, forward sweep:
                for (int i = 0; i < numEdges; ++i)
                {
                    int edge = first + i;

                    float cip_ = (i > 0) ? sweep[i - 1].x : 0;
                    float dip_ = (i > 0) ? sweep[i - 1].y : 0;
                    float den = diagonals[i].y - cip_ * diagonals[i].x;

                    if (den != 0)
                    {
                        sweep[i] = new float2((diagonals[i].z / den),
                                              (di[i] - dip_ * diagonals[i].x) / den);
                    }
                    else
                        sweep[i] = float2.zero;
                }

                // solve step #2, backward sweep:
                NativeArray<float> xi = new NativeArray<float>(numEdges, Allocator.Temp);
                for (int i = numEdges - 1; i >= 0; --i)
                {
                    int edge = first + i;

                    float xi_ = (i < numEdges - 1) ? xi[i + 1] : 0;
                    xi[i] = sweep[i].y - sweep[i].x * xi_;
                }

                // calculate deltas:
                for (int i = 0; i < numIndices[c]; ++i)
                {
                    int index = first + i;

                    float4 ni__ = (i > 0) ? ni[i - 1] : float4.zero;
                    float4 n_i_ = (i < numIndices[c] - 1) ? ni[i] : float4.zero;

                    float xi_ = (i > 0) ? xi[i - 1] : 0;
                    float nxi = (i < numIndices[c] - 1) ? xi[i] : 0;

                    int p = particleIndices[index];
                    deltas[p] -= invMasses[p] * (-ni__ * xi_ + n_i_ * nxi);
                    counts[p]++;
                }
            }
        }

        [BurstCompile]
        public struct ApplyChainConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] public NativeArray<int> firstIndex;
            [ReadOnly] public NativeArray<int> numIndices;
            [ReadOnly] public float sorFactor;

            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> positions;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<int> counts;

            public void Execute(int i)
            {
                int first = firstIndex[i];
                int last = first + numIndices[i];

                for (int k = first; k < last; ++k)
                {
                    int p = particleIndices[k];
                    if (counts[p] > 0)
                    {
                        positions[p] += deltas[p] * sorFactor / counts[p];
                        deltas[p] = float4.zero;
                        counts[p] = 0;
                    }
                }
            }
        }
    }
}
#endif