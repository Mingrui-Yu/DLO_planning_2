#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using UnityEngine;
using Unity.Jobs;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Burst;
using System;
using System.Collections;

namespace Obi
{
    [BurstCompile]
    struct UpdateInertiaTensorsJob : IJobParallelFor
    {
        [ReadOnly] public NativeList<int> activeParticles;

        [ReadOnly] public NativeArray<float> inverseMasses;
        [ReadOnly] public NativeArray<float> inverseRotationalMasses;
        [ReadOnly] public NativeArray<float4> principalRadii;

        [NativeDisableParallelForRestriction]
        public NativeArray<float4> inverseInertiaTensors;

        public void Execute(int index)
        {
            int i = activeParticles[index];

            float4 sqrRadii = principalRadii[i] * principalRadii[i];

            inverseInertiaTensors[i] = 5 * inverseRotationalMasses[i] * new float4(
                                        1 / math.max(sqrRadii[1] + sqrRadii[2], BurstMath.epsilon),
                                        1 / math.max(sqrRadii[0] + sqrRadii[2], BurstMath.epsilon),
                                        1 / math.max(sqrRadii[0] + sqrRadii[1], BurstMath.epsilon),
                                        0);
        }
    }
}
#endif