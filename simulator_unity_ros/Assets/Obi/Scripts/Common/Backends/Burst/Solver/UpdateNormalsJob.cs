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
    struct UpdateNormalsJob : IJob
    {
        [ReadOnly] public NativeList<int> deformableTriangles;
        [ReadOnly] public NativeArray<float4> renderPositions;
        public NativeArray<float4> normals;

        public void Execute()
        {
            for (int i = 0; i < normals.Length; ++i)
                normals[i] = float4.zero;

            // Accumulate normals:
            for (int i = 0; i < deformableTriangles.Length; i += 3)
            {
                float4 v1 = renderPositions[deformableTriangles[i]];
                float4 v2 = renderPositions[deformableTriangles[i + 1]];
                float4 v3 = renderPositions[deformableTriangles[i + 2]];

                float4 n = new float4(math.cross((v2 - v1).xyz, (v3 - v1).xyz), 0);

                normals[deformableTriangles[i]] += n;
                normals[deformableTriangles[i + 1]] += n;
                normals[deformableTriangles[i + 2]] += n;
            }

            for (int i = 0; i < normals.Length; ++i)
                normals[i] = math.normalizesafe(normals[i]);
        }
    }
}
#endif