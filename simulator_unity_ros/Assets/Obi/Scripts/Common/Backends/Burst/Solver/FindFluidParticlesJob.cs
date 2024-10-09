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
    struct FindFluidParticlesJob : IJob
    {
        [ReadOnly] public NativeList<int> activeParticles;
        [ReadOnly] public NativeArray<int> phases;

        public NativeList<int> fluidParticles;

        public void Execute()
        {
            fluidParticles.Clear();

            for (int i = 0; i < activeParticles.Length; ++i)
            {
                int p = activeParticles[i];
                if ((phases[p] & (int)Oni.ParticleFlags.Fluid) != 0)
                    fluidParticles.Add(p);
            }
        }
    }
}
#endif