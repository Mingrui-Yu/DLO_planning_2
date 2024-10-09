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
    struct ApplyInertialForcesJob : IJobParallelFor
    {
		[ReadOnly] public NativeList<int> activeParticles;
        [ReadOnly] public NativeArray<float4> positions;
		[ReadOnly] public NativeArray<float> invMasses;

		[ReadOnly] public float4 angularVel;
		[ReadOnly] public float4 inertialAccel;
		[ReadOnly] public float4 eulerAccel;

		[ReadOnly] public float worldLinearInertiaScale;
		[ReadOnly] public float worldAngularInertiaScale;

        [NativeDisableParallelForRestriction] public NativeArray<float4> velocities;

        [ReadOnly] public float deltaTime;

        public void Execute(int index)
        {
            int i = activeParticles[index];

            if (invMasses[i] > 0)
			{
				float4 euler = new float4(math.cross(eulerAccel.xyz, positions[i].xyz), 0);
				float4 centrifugal = new float4(math.cross(angularVel.xyz, math.cross(angularVel.xyz, positions[i].xyz)), 0);
				float4 coriolis = 2 * new float4(math.cross(angularVel.xyz, velocities[i].xyz), 0);
				float4 angularAccel = euler + coriolis + centrifugal;

				velocities[i] -= (inertialAccel * worldLinearInertiaScale + angularAccel * worldAngularInertiaScale) * deltaTime;
			}
        }
    }
}
#endif