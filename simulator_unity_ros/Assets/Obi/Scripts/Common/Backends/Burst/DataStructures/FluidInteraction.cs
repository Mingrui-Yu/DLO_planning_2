#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;
using Unity.Mathematics;

namespace Obi
{
    public struct FluidInteraction : IConstraint
    {
        public float4 gradient;
        public float avgKernel;
        public float avgGradient;
        public int particleA;
        public int particleB;

        public int GetParticleCount() { return 2; }
        public int GetParticle(int index) { return index == 0 ? particleA : particleB; }
    }
}
#endif