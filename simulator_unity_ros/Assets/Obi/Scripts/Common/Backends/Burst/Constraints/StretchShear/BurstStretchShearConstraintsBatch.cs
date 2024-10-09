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
    public class BurstStretchShearConstraintsBatch : BurstConstraintsBatchImpl, IStretchShearConstraintsBatchImpl
    {
        private NativeArray<int> orientationIndices;
        private NativeArray<float> restLengths;
        private NativeArray<quaternion> restOrientations;
        private NativeArray<float3> stiffnesses;

        public BurstStretchShearConstraintsBatch(BurstStretchShearConstraints constraints)
        {
            m_Constraints = constraints;
            m_ConstraintType = Oni.ConstraintType.StretchShear;
        }

        public void SetStretchShearConstraints(ObiNativeIntList particleIndices, ObiNativeIntList orientationIndices, ObiNativeFloatList restLengths, ObiNativeQuaternionList restOrientations, ObiNativeVector3List stiffnesses, ObiNativeFloatList lambdas, int count)
        {
            this.particleIndices = particleIndices.AsNativeArray<int>();
            this.orientationIndices = orientationIndices.AsNativeArray<int>();
            this.restLengths = restLengths.AsNativeArray<float>();
            this.restOrientations = restOrientations.AsNativeArray<quaternion>();
            this.stiffnesses = stiffnesses.AsNativeArray<float3>();
            this.lambdas = lambdas.AsNativeArray<float>();
            m_ConstraintCount = count;
        }

        public override JobHandle Evaluate(JobHandle inputDeps, float deltaTime)
        {
            var projectConstraints = new StretchShearConstraintsBatchJob()
            {
                particleIndices = particleIndices,
                orientationIndices = orientationIndices,
                restLengths = restLengths,
                restOrientations = restOrientations,
                stiffnesses = stiffnesses,
                lambdas = lambdas.Reinterpret<float, float3>(),

                positions = solverImplementation.positions,
                orientations = solverImplementation.orientations,
                invMasses = solverImplementation.invMasses,
                invRotationalMasses = solverImplementation.invRotationalMasses,

                deltas = solverImplementation.positionDeltas,
                orientationDeltas = solverImplementation.orientationDeltas,
                counts = solverImplementation.positionConstraintCounts,
                orientationCounts = solverImplementation.orientationConstraintCounts,

                deltaTimeSqr = deltaTime * deltaTime
            };

            return projectConstraints.Schedule(m_ConstraintCount, 32, inputDeps);
        }

        public override JobHandle Apply(JobHandle inputDeps, float deltaTime)
        {
            var parameters = solverAbstraction.GetConstraintParameters(m_ConstraintType);

            var applyConstraints = new ApplyStretchShearConstraintsBatchJob()
            {
                particleIndices = particleIndices,
                orientationIndices = orientationIndices,

                positions = solverImplementation.positions,
                deltas = solverImplementation.positionDeltas,
                counts = solverImplementation.positionConstraintCounts,

                orientations = solverImplementation.orientations,
                orientationDeltas = solverImplementation.orientationDeltas,
                orientationCounts = solverImplementation.orientationConstraintCounts,

                sorFactor = parameters.SORFactor
            };

            return applyConstraints.Schedule(m_ConstraintCount, 64, inputDeps);
        }

        [BurstCompile]
        public struct StretchShearConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] public NativeArray<int> orientationIndices;
            [ReadOnly] public NativeArray<float> restLengths;
            [ReadOnly] public NativeArray<quaternion> restOrientations;
            [ReadOnly] public NativeArray<float3> stiffnesses;
            public NativeArray<float3> lambdas;

            [ReadOnly] public NativeArray<float4> positions;
            [ReadOnly] public NativeArray<quaternion> orientations;
            [ReadOnly] public NativeArray<float> invMasses;
            [ReadOnly] public NativeArray<float> invRotationalMasses;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<quaternion> orientationDeltas;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> counts;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> orientationCounts;

            [ReadOnly] public float deltaTimeSqr;

            public void Execute(int i)
            {
                int p1 = particleIndices[i * 2];
                int p2 = particleIndices[i * 2 + 1];
                int q = orientationIndices[i];

                float w1 = invMasses[p1];
                float w2 = invMasses[p2];

                // calculate time adjusted compliance
                float3 compliances = stiffnesses[i] / deltaTimeSqr;

                float3 e = math.rotate(restOrientations[i],new float3(0, 0, 1));
                quaternion basis = math.mul(orientations[q],restOrientations[i]);

                // calculate rod vector in local element space:
                float3 gamma = math.rotate(math.conjugate(basis), (positions[p2] - positions[p1]).xyz) / (restLengths[i] + BurstMath.epsilon);

                // subtract third director vector (0,0,1):
                gamma[2] -= 1;

                float3 W = new float3((w1 + w2) / (restLengths[i] + BurstMath.epsilon) + invRotationalMasses[q] * 4.0f * restLengths[i] + BurstMath.epsilon);
                float3 dlambda = (gamma - compliances * lambdas[i]) / (compliances + W);
                lambdas[i] += dlambda;

                // convert lambda delta lambda back to world space:
                dlambda = math.mul(basis, dlambda);

                deltas[p1] += new float4(dlambda, 0) * w1;
                deltas[p2] -= new float4(dlambda, 0) * w2;

                quaternion e_3 = new quaternion(e.x,e.y,e.z,0);
                quaternion q_e_3_bar = math.mul(orientations[q],math.conjugate(e_3));

                // calculate rotation delta:
                quaternion rotDelta = math.mul(new quaternion(dlambda[0], dlambda[1], dlambda[2], 0.0f),q_e_3_bar);
                rotDelta.value *= 2.0f * invRotationalMasses[q] * restLengths[i];
                orientationDeltas[q] = rotDelta;

                counts[p1]++;
                counts[p2]++;
                orientationCounts[q]++;

            }
        }

        [BurstCompile]
        public struct ApplyStretchShearConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] public NativeArray<int> orientationIndices;
            [ReadOnly] public float sorFactor;

            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> positions;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<int> counts;

            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<quaternion> orientations;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<quaternion> orientationDeltas;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<int> orientationCounts;

            public void Execute(int i)
            {
                int p1 = particleIndices[i * 2];
                int p2 = particleIndices[i * 2 + 1];
                int q1 = orientationIndices[i];

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

                if (orientationCounts[q1] > 0)
                {
                    quaternion q = orientations[q1];
                    q.value += orientationDeltas[q1].value * sorFactor / orientationCounts[q1];
                    orientations[q1] = math.normalize(q);

                    orientationDeltas[q1] = new quaternion(0, 0, 0, 0);
                    orientationCounts[q1] = 0;
                }

            }
        }
    }
}
#endif