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
    public class BurstVolumeConstraintsBatch : BurstConstraintsBatchImpl, IVolumeConstraintsBatchImpl
    {
        private NativeArray<int> firstTriangle;
        private NativeArray<int> numTriangles;
        private NativeArray<float> restVolumes;
        private NativeArray<float2> pressureStiffness;

        public BurstVolumeConstraintsBatch(BurstVolumeConstraints constraints)
        {
            m_Constraints = constraints;
            m_ConstraintType = Oni.ConstraintType.Volume;
        }

        public void SetVolumeConstraints(ObiNativeIntList triangles,
                                         ObiNativeIntList firstTriangle,
                                         ObiNativeIntList numTriangles,
                                         ObiNativeFloatList restVolumes,
                                         ObiNativeVector2List pressureStiffness,
                                         ObiNativeFloatList lambdas,
                                         int count)
        {
            this.particleIndices = triangles.AsNativeArray<int>();
            this.firstTriangle = firstTriangle.AsNativeArray<int>();
            this.numTriangles = numTriangles.AsNativeArray<int>();
            this.restVolumes = restVolumes.AsNativeArray<float>();
            this.pressureStiffness = pressureStiffness.AsNativeArray<float2>();
            this.lambdas = lambdas.AsNativeArray<float>();
            m_ConstraintCount = count;
        }


        public override JobHandle Evaluate(JobHandle inputDeps, float deltaTime)
        {
            var projectConstraints = new VolumeConstraintsBatchJob()
            {
                triangles = particleIndices,
                firstTriangle = firstTriangle,
                numTriangles = numTriangles,
                restVolumes = restVolumes,
                pressureStiffness = pressureStiffness,
                lambdas = lambdas,

                positions = solverImplementation.positions,
                invMasses = solverImplementation.invMasses,

                gradients = solverImplementation.fluidData, // reuse fluidData for temp gradients.
                deltas = solverImplementation.positionDeltas,
                counts = solverImplementation.positionConstraintCounts,

                deltaTimeSqr = deltaTime * deltaTime
            };

            return projectConstraints.Schedule(m_ConstraintCount, 4, inputDeps);
        }

        public override JobHandle Apply(JobHandle inputDeps, float deltaTime)
        {
            var parameters = solverAbstraction.GetConstraintParameters(m_ConstraintType);

            var applyConstraints = new ApplyVolumeConstraintsBatchJob()
            {
                triangles = particleIndices,
                firstTriangle = firstTriangle,
                numTriangles = numTriangles,
                positions = solverImplementation.positions,
                deltas = solverImplementation.positionDeltas,
                counts = solverImplementation.positionConstraintCounts,

                sorFactor = parameters.SORFactor
            };

            return applyConstraints.Schedule(m_ConstraintCount, 8, inputDeps);
        }

        [BurstCompile]
        public struct VolumeConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> triangles;
            [ReadOnly] public NativeArray<int> firstTriangle;
            [ReadOnly] public NativeArray<int> numTriangles;
            [ReadOnly] public NativeArray<float> restVolumes;
            [ReadOnly] public NativeArray<float2> pressureStiffness;
            public NativeArray<float> lambdas;

            [ReadOnly] public NativeArray<float4> positions;
            [ReadOnly] public NativeArray<float> invMasses;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> gradients;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> counts;

            [ReadOnly] public float deltaTimeSqr;

            public void Execute(int i)
            {
                float compliance = pressureStiffness[i].y / deltaTimeSqr;

                NativeList<int> particleIndices = new NativeList<int>(numTriangles[i] * 3, Allocator.Temp);

                // calculate volume:
                float volume = 0;
                for (int j = 0; j < numTriangles[i]; ++j)
                {
                    int v = (firstTriangle[i] + j) * 3;
                    int i1 = triangles[v];
                    int i2 = triangles[v + 1];
                    int i3 = triangles[v + 2];

                    particleIndices.Add(i1);
                    particleIndices.Add(i2);
                    particleIndices.Add(i3);

                    //calculate this triangle's volume contribution:
                    volume += math.dot(math.cross(positions[i1].xyz, positions[i2].xyz), positions[i3].xyz) / 6.0f;
                }

                particleIndices.Sort();
                int particleCount = particleIndices.AsArray().Unique();

                // accumulate particle gradients:
                for (int j = 0; j < particleCount; ++j)
                    gradients[particleIndices[j]] = float4.zero;

                for (int j = 0; j < numTriangles[i]; ++j)
                {
                    int v = (firstTriangle[i] + j) * 3;
                    int i1 = triangles[v];
                    int i2 = triangles[v + 1];
                    int i3 = triangles[v + 2];

                    //accumulate gradient for each particle:
                    gradients[i1] += new float4(math.cross(positions[i2].xyz, positions[i3].xyz), 0);
                    gradients[i2] += new float4(math.cross(positions[i3].xyz, positions[i1].xyz), 0);
                    gradients[i3] += new float4(math.cross(positions[i1].xyz, positions[i2].xyz), 0);
                }

                // calculate constraint denominator (G(Cj)*inv(M)):
                float denominator = 0;
                for (int j = 0; j < particleCount; ++j)
                {
                    int p = particleIndices[j];
                    denominator += invMasses[p] * math.lengthsq(gradients[p]);
                }

                // equality constraint: volume - pressure * rest volume = 0
                float constraint = volume - pressureStiffness[i].x * restVolumes[i];

                // calculate lagrange multiplier delta:
                float dlambda = (-constraint - compliance * lambdas[i]) / (denominator + compliance + BurstMath.epsilon);
                lambdas[i] += dlambda;

                // calculate position deltas:
                for (int j = 0; j < particleCount; ++j)
                {
                    int p = particleIndices[j];
                    deltas[p] += dlambda * invMasses[p] * gradients[p];
                    counts[p]++;
                }
            }
        }

        [BurstCompile]
        public struct ApplyVolumeConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> triangles;
            [ReadOnly] public NativeArray<int> firstTriangle;
            [ReadOnly] public NativeArray<int> numTriangles;
            [ReadOnly] public float sorFactor;

            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> positions;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<int> counts;

            public void Execute(int i)
            {

                for (int j = 0; j < numTriangles[i]; ++j)
                {
                    int v = (firstTriangle[i] + j) * 3;
                    int p1 = triangles[v];
                    int p2 = triangles[v + 1];
                    int p3 = triangles[v + 2];

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
}
#endif