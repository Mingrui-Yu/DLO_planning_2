#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;
using Unity.Mathematics;

namespace Obi
{
    public struct Poly6Kernel
    {
        public float norm;
        public bool norm2D;

        public Poly6Kernel(bool norm2D)
        {
            this.norm2D = norm2D;
            if (norm2D)
                norm = 4.0f / math.PI;
            else
                norm = 315.0f / (64.0f * math.PI);
        }

        public float W(float r, float h)
        {
            float h2 = h * h;
            float h4 = h2 * h2;
            float h8 = h4 * h4;

            float rl = math.min(r, h);
            float hr = h2 - rl * rl;

            if (norm2D)
                return norm / h8 * hr * hr * hr;
            return norm / (h8 * h) * hr * hr * hr;
        }
    }
}
#endif