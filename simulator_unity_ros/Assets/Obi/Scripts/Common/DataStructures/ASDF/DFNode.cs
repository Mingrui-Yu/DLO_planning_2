using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace Obi
{
    [Serializable]
    public struct DFNode
    {
        public Vector4 distancesA;
        public Vector4 distancesB;
        public Vector4 center;
        public int firstChild;

        // add 12 bytes of padding to ensure correct memory alignment:
        private int pad0;
        private int pad1;
        private int pad2;

        public DFNode(Vector4 center)
        {
            this.distancesA = Vector4.zero;
            this.distancesB = Vector4.zero;
            this.center = center;
            this.firstChild = -1;
            this.pad0 = 0;
            this.pad1 = 0;
            this.pad2 = 0;
        }

        public float Sample(Vector3 position)
        {
            Vector3 nPos = GetNormalizedPos(position);

            // trilinear interpolation: interpolate along x axis
            Vector4 x = distancesA + (distancesB - distancesA) * nPos[0];

            // interpolate along y axis
            float y0 = x[0] + (x[2] - x[0]) * nPos[1];
            float y1 = x[1] + (x[3] - x[1]) * nPos[1];

            // interpolate along z axis.
            return y0 + (y1 - y0) * nPos[2];
        }

        public Vector3 GetNormalizedPos(Vector3 position)
        {
            float size = center[3] * 2;
            return new Vector3(
                (position[0] - (center[0] - center[3])) / size,
                (position[1] - (center[1] - center[3])) / size,
                (position[2] - (center[2] - center[3])) / size
            );
        }

        public int GetOctant(Vector3 position)
        {
            int index = 0;
            if (position[0] > center[0]) index |= 4;
            if (position[1] > center[1]) index |= 2;
            if (position[2] > center[2]) index |= 1;
            return index;
        }
    }
}
