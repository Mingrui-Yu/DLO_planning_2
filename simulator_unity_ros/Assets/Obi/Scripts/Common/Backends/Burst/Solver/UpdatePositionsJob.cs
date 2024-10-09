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
    struct UpdatePositionsJob : IJobParallelFor
    {
        [ReadOnly] public NativeList<int> activeParticles;

        // linear/position properties:
        [NativeDisableParallelForRestriction] public NativeArray<float4> positions;
        [ReadOnly] public NativeArray<float4> previousPositions;
        [NativeDisableParallelForRestriction] public NativeArray<float4> velocities;

        // angular/orientation properties:
        [NativeDisableParallelForRestriction] public NativeArray<quaternion> orientations;
        [ReadOnly] public NativeArray<quaternion> previousOrientations;
        [NativeDisableParallelForRestriction] public NativeArray<float4> angularVelocities;

        [ReadOnly] public float velocityScale;
        [ReadOnly] public float sleepThreshold;

        // The code actually running on the job
        public void Execute(int index)
        {
            int i = activeParticles[index];

            // damp velocities:
            velocities[i] *= velocityScale;
            angularVelocities[i] *= velocityScale;

            // if the kinetic energy is below the sleep threshold, keep the particle at its previous position.
            if (math.lengthsq(velocities[i]) * 0.5f + math.lengthsq(angularVelocities[i]) * 0.5f <= sleepThreshold)
            {
                positions[i] = previousPositions[i];
                orientations[i] = previousOrientations[i];
                velocities[i] = float4.zero;
                angularVelocities[i] = float4.zero;
            }

        }
    }
}
#endif