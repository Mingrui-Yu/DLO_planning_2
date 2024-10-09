#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;
using System.Runtime.InteropServices;
using UnityEngine;
using Unity.Mathematics;

namespace Obi
{
    public struct BurstDFNode
    {
        public float4 distancesA;
        public float4 distancesB;
        public float4 center;
        public int firstChild;

        // add 12 bytes of padding to ensure correct memory alignment:
        private int pad0;
        private int pad1;
        private int pad2;

        public float4 SampleWithGradient(float4 position)
        {
            float4 nPos = GetNormalizedPos(position);

            // trilinear interpolation of distance:
            float4 x = distancesA + (distancesB - distancesA) * nPos[0];
            float2 y = x.xy + (x.zw - x.xy) * nPos[1];
            float distance = y[0] + (y[1] - y[0]) * nPos[2];

            // gradient estimation:
            // x == 0
            float2 a = distancesA.xy + (distancesA.zw - distancesA.xy) * nPos[1];
            float x0 = a[0] + (a[1] - a[0]) * nPos[2];

            // x == 1
            a = distancesB.xy + (distancesB.zw - distancesB.xy) * nPos[1];
            float x1 = a[0] + (a[1] - a[0]) * nPos[2];

            // y == 0
            float y0 = x[0] + (x[1] - x[0]) * nPos[2];

            // y == 1
            float y1 = x[2] + (x[3] - x[2]) * nPos[2];

            return new float4(x1 - x0, y1 - y0, y[1] - y[0], distance);

        }

        public float4 GetNormalizedPos(float4 position)
        {
            float4 corner = center - new float4(center[3]);
            return (position - corner) / (center[3] * 2);
        }

        public int GetOctant(float4 position)
        {
            int index = 0;
            if (position[0] > center[0]) index |= 4;
            if (position[1] > center[1]) index |= 2;
            if (position[2] > center[2]) index |= 1;
            return index;
        }
    }
}
#endif