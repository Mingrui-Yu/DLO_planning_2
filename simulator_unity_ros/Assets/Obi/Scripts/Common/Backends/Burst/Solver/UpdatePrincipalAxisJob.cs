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
    struct UpdatePrincipalAxisJob : IJobParallelFor
    {
        [ReadOnly] public NativeList<int> activeParticles;
        [ReadOnly] public NativeArray<quaternion> renderableOrientations;
        [ReadOnly] public NativeArray<int> phases;
        [ReadOnly] public NativeArray<float4> principalRadii;

        [NativeDisableParallelForRestriction]
        public NativeArray<float4> principalAxis;

        public void Execute(int index)
        {
            int i = activeParticles[index];

            // fluid particles get their principal axes from the neighborhood matrix, so skip them.
            if ((phases[i] & (int)Oni.ParticleFlags.Fluid) == 0)
            {
                int i3 = i * 3;
                float4x4 rot = renderableOrientations[i].toMatrix();

                // set axis direction:
                float4 pAxis1 = rot.c0;
                float4 pAxis2 = rot.c1;
                float4 pAxis3 = rot.c2;

                // set axis length:
                pAxis1[3] = principalRadii[i][0];
                pAxis2[3] = principalRadii[i][1];
                pAxis3[3] = principalRadii[i][2];

                principalAxis[i3] = pAxis1;
                principalAxis[i3+1] = pAxis2;
                principalAxis[i3+2] = pAxis3;
            }
        }
    }
}
#endif