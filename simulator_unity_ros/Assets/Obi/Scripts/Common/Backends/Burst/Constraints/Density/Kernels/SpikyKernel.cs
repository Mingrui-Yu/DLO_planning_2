#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;
using Unity.Mathematics;

namespace Obi
{
    public struct SpikyKernel
    {
        public float norm;
        public bool norm2D;

        public SpikyKernel(bool norm2D)
        {
            this.norm2D = norm2D;
            if (norm2D)
                norm = -30.0f / math.PI;
            else
                norm = -45.0f / math.PI;
        }

        public float W(float r, float h)
        {
            float h2 = h * h;
            float h4 = h2 * h2;

            float rl = math.min(r, h);
            float hr = h - rl;

            if (norm2D)
                return norm / (h4 * h) * hr * hr;
            return norm / (h4 * h2) * hr * hr;
        }
    }
}
#endif