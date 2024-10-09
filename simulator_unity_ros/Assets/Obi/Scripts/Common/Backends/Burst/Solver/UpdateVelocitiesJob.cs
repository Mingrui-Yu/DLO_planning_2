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
    struct UpdateVelocitiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeList<int> activeParticles;

        // linear/position properties:
        [ReadOnly] public NativeArray<float> inverseMasses;
        [ReadOnly] public NativeArray<float4> previousPositions;
        [NativeDisableParallelForRestriction] public NativeArray<float4> positions;
        [NativeDisableParallelForRestriction] [WriteOnly] public NativeArray<float4> velocities;

        // angular/orientation properties:
        [ReadOnly] public NativeArray<float> inverseRotationalMasses;
        [ReadOnly] public NativeArray<quaternion> previousOrientations;
        [NativeDisableParallelForRestriction] public NativeArray<quaternion> orientations;
        [NativeDisableParallelForRestriction] [WriteOnly] public NativeArray<float4> angularVelocities;

        [ReadOnly] public float deltaTime;
        [ReadOnly] public bool is2D;

        // The code actually running on the job
        public void Execute(int index)
        {
            int i = activeParticles[index];

            // Project particles on the XY plane if we are in 2D mode:
            if (is2D)
            {
                // restrict position to the 2D plane
                float4 pos = positions[i];
                pos[2] = previousPositions[i][2];
                positions[i] = pos;

                // restrict rotation to the axis perpendicular to the 2D plane.
                quaternion swing, twist;
                BurstMath.SwingTwist(orientations[i], new float3(0, 0, 1), out swing, out twist);
                orientations[i] = twist;
            }

            if (inverseMasses[i] > 0)
                velocities[i] = BurstIntegration.DifferentiateLinear(positions[i], previousPositions[i], deltaTime);
            else
                velocities[i] = float4.zero;

            if (inverseRotationalMasses[i] > 0)
                angularVelocities[i] = BurstIntegration.DifferentiateAngular(orientations[i], previousOrientations[i], deltaTime);
            else
                angularVelocities[i] = float4.zero;
        }
    }
}
#endif