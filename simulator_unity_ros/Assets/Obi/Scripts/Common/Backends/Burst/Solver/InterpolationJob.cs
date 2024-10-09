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
    struct InterpolationJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float4> startPositions;
        [ReadOnly] public NativeArray<float4> positions;
        [WriteOnly] public NativeArray<float4> renderablePositions;

        [ReadOnly] public NativeArray<quaternion> startOrientations;
        [ReadOnly] public NativeArray<quaternion> orientations;
        [WriteOnly] public NativeArray<quaternion> renderableOrientations;

        [ReadOnly] public float deltaTime;
        [ReadOnly] public float unsimulatedTime;
        [ReadOnly] public Oni.SolverParameters.Interpolation interpolationMode;

        // The code actually running on the job
        public void Execute(int i)
        {
            if (interpolationMode == Oni.SolverParameters.Interpolation.Interpolate && deltaTime > 0)
            {
                float alpha = unsimulatedTime / deltaTime;

                renderablePositions[i] = math.lerp(startPositions[i], positions[i], alpha);
                renderableOrientations[i] = math.normalize(math.slerp(startOrientations[i], orientations[i], alpha));
            }
            else
            {
                renderablePositions[i] = positions[i];
                renderableOrientations[i] = math.normalize(orientations[i]);
            }
        }
    }
}
#endif