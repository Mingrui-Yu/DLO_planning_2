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
    public class BurstBendTwistConstraintsBatch : BurstConstraintsBatchImpl, IBendTwistConstraintsBatchImpl
    {
        private NativeArray<int> orientationIndices;
        private NativeArray<quaternion> restDarboux;
        private NativeArray<float3> stiffnesses;

        public BurstBendTwistConstraintsBatch(BurstBendTwistConstraints constraints)
        {
            m_Constraints = constraints;
            m_ConstraintType = Oni.ConstraintType.BendTwist;
        }

        public void SetBendTwistConstraints(ObiNativeIntList orientationIndices, ObiNativeQuaternionList restDarboux, ObiNativeVector3List stiffnesses, ObiNativeFloatList lambdas, int count)
        {
            this.orientationIndices = orientationIndices.AsNativeArray<int>();
            this.restDarboux = restDarboux.AsNativeArray<quaternion>();
            this.stiffnesses = stiffnesses.AsNativeArray<float3>();
            this.lambdas = lambdas.AsNativeArray<float>();
            m_ConstraintCount = count;
        }

        public override JobHandle Evaluate(JobHandle inputDeps, float deltaTime)
        {
            var projectConstraints = new BendTwistConstraintsBatchJob()
            {
                orientationIndices = orientationIndices,
                restDarboux = restDarboux,
                stiffnesses = stiffnesses,
                lambdas = lambdas.Reinterpret<float, float3>(),

                orientations = solverImplementation.orientations,
                invRotationalMasses = solverImplementation.invRotationalMasses,

                orientationDeltas = solverImplementation.orientationDeltas,
                orientationCounts = solverImplementation.orientationConstraintCounts ,

                deltaTimeSqr = deltaTime * deltaTime
            };

            return projectConstraints.Schedule(m_ConstraintCount, 32, inputDeps);
        }

        public override JobHandle Apply(JobHandle inputDeps, float deltaTime)
        {
            var parameters = solverAbstraction.GetConstraintParameters(m_ConstraintType);

            var applyConstraints = new ApplyBendTwistConstraintsBatchJob()
            {
                orientationIndices = orientationIndices,

                orientations = solverImplementation.orientations,
                orientationDeltas = solverImplementation.orientationDeltas,
                orientationCounts = solverImplementation.orientationConstraintCounts,

                sorFactor = parameters.SORFactor
            };

            return applyConstraints.Schedule(m_ConstraintCount, 64, inputDeps);
        }

        [BurstCompile]
        public struct BendTwistConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> orientationIndices;
            [ReadOnly] public NativeArray<quaternion> restDarboux;
            [ReadOnly] public NativeArray<float3> stiffnesses;
            public NativeArray<float3> lambdas;

            [ReadOnly] public NativeArray<quaternion> orientations;
            [ReadOnly] public NativeArray<float> invRotationalMasses;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<quaternion> orientationDeltas;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> orientationCounts;

            [ReadOnly] public float deltaTimeSqr;

            public void Execute(int i)
            {
                int q1 = orientationIndices[i * 2];
                int q2 = orientationIndices[i * 2 + 1];

                float w1 = invRotationalMasses[q1];
                float w2 = invRotationalMasses[q2];

                // calculate time adjusted compliance
                float3 compliances = stiffnesses[i] / deltaTimeSqr;

                quaternion omega = math.mul(math.conjugate(orientations[q1]), orientations[q2]);   //darboux vector

                quaternion omega_plus;
                omega_plus.value = omega.value + restDarboux[i].value;  //delta Omega with - omega_0
                omega.value -= restDarboux[i].value;                    //delta Omega with + omega_0

                if (math.lengthsq(omega.value) > math.lengthsq(omega_plus.value))
                    omega = omega_plus;

                float3 dlambda = (omega.value.xyz - compliances * lambdas[i]) / (compliances + new float3(w1 + w2 + BurstMath.epsilon));

                //discrete Darboux vector does not have vanishing scalar part
                quaternion dlambdaQ = new quaternion(dlambda[0], dlambda[1], dlambda[2],0);

                quaternion d1 = orientationDeltas[q1];
                quaternion d2 = orientationDeltas[q2];

                d1.value += math.mul(orientations[q2], dlambdaQ).value * w1;
                d2.value += math.mul(orientations[q1], dlambdaQ).value * -w2;

                orientationDeltas[q1] = d1;
                orientationDeltas[q2] = d2;

                orientationCounts[q1]++;
                orientationCounts[q2]++;

                lambdas[i] += dlambda;

            }
        }

        [BurstCompile]
        public struct ApplyBendTwistConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> orientationIndices;
            [ReadOnly] public float sorFactor;

            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<quaternion> orientations;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<quaternion> orientationDeltas;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<int> orientationCounts;

            public void Execute(int i)
            {
                int p1 = orientationIndices[i * 2];
                int p2 = orientationIndices[i * 2 + 1];

                if (orientationCounts[p1] > 0)
                {
                    quaternion q = orientations[p1];
                    q.value += orientationDeltas[p1].value * sorFactor / orientationCounts[p1];
                    orientations[p1] = math.normalize(q);

                    orientationDeltas[p1] = new quaternion(0, 0, 0, 0);
                    orientationCounts[p1] = 0;
                }

                if (orientationCounts[p2] > 0)
                {
                    quaternion q = orientations[p2];
                    q.value += orientationDeltas[p2].value * sorFactor / orientationCounts[p2];
                    orientations[p2] = math.normalize(q);

                    orientationDeltas[p2] = new quaternion(0, 0, 0, 0);
                    orientationCounts[p2] = 0;
                }
            }

        }
    }
}
#endif