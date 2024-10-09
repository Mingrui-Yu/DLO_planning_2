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
    struct ParticleToBoundsJob : IJobParallelFor
    {
        [ReadOnly] public NativeList<int> activeParticles;
        [ReadOnly] public NativeArray<float4> positions;
        [ReadOnly] public NativeArray<float4> radii;

        public NativeArray<BurstAabb> bounds;

        public void Execute(int i)
        {
            int p = activeParticles[i];
            bounds[i] = new BurstAabb(positions[p] - radii[p].x, positions[p] + radii[p].x);
        }
    }

    [BurstCompile]
    struct BoundsReductionJob : IJobParallelFor
    {
        [NativeDisableParallelForRestriction] public NativeArray<BurstAabb> bounds; // the length of bounds must be a multiple of size.
        [ReadOnly] public int stride;
        [ReadOnly] public int size;

        public void Execute(int first)
        {
            int baseIndex = first * size;
            for (int i = 1; i < size; ++i)
            {
                int dest = baseIndex * stride;
                int source = (baseIndex + i) * stride;

                if (source < bounds.Length)
                {
                    var v = bounds[dest];
                    v.EncapsulateBounds(bounds[source]);
                    bounds[dest] = v;
                }
            }
        }
    }
}
#endif