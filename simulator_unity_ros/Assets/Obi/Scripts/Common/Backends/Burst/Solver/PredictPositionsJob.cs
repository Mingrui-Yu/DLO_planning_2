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
    struct PredictPositionsJob : IJobParallelFor
    {
        [ReadOnly] public NativeList<int> activeParticles;
        [ReadOnly] public NativeArray<int> phases;
        [ReadOnly] public NativeArray<float> buoyancies;

        // linear/position properties:
        [ReadOnly] public NativeArray<float4> externalForces;
        [ReadOnly] public NativeArray<float> inverseMasses;
        [NativeDisableParallelForRestriction] public NativeArray<float4> previousPositions;
        [NativeDisableParallelForRestriction] public NativeArray<float4> positions;
        [NativeDisableParallelForRestriction] public NativeArray<float4> velocities;

        // angular/orientation properties:
        [ReadOnly] public NativeArray<float4> externalTorques;
        [ReadOnly] public NativeArray<float> inverseRotationalMasses;
        [NativeDisableParallelForRestriction] public NativeArray<quaternion> previousOrientations;
        [NativeDisableParallelForRestriction] public NativeArray<quaternion> orientations;
        [NativeDisableParallelForRestriction] public NativeArray<float4> angularVelocities;

        [ReadOnly] public float4 gravity;
        [ReadOnly] public float deltaTime;
        [ReadOnly] public bool is2D;

        public void Execute(int index)
        {
            int i = activeParticles[index];

            // the previous position/orientation is the current position/orientation at the start of the step.
            previousPositions[i] = positions[i];
            previousOrientations[i] = orientations[i];

            if (inverseMasses[i] > 0)
            {
                float4 effectiveGravity = gravity;

                // Adjust gravity for buoyant fluid particles:
                if ((phases[i] & (int)Oni.ParticleFlags.Fluid) != 0)
                    effectiveGravity *= -buoyancies[i];

                // apply external forces and gravity:
                float4 vel = velocities[i] + (inverseMasses[i] * externalForces[i] + effectiveGravity) * deltaTime; 

                // project velocity to 2D plane if needed:
                if (is2D)
                    vel[3] = 0;

                velocities[i] = vel;
            }

            if (inverseRotationalMasses[i] > 0)
            {
                // apply external torques (simplification: we don't use full inertia tensor here)
                float4 angularVel = angularVelocities[i] + inverseRotationalMasses[i] * externalTorques[i] * deltaTime;

                // project angular velocity to 2D plane normal if needed:
                if (is2D)
                    angularVel = angularVel.project(new float4(0, 0, 1, 0));

                angularVelocities[i] = angularVel;
            }

            // integrate velocities:
            positions[i]    = BurstIntegration.IntegrateLinear(positions[i], velocities[i], deltaTime);
            orientations[i] = BurstIntegration.IntegrateAngular(orientations[i], angularVelocities[i], deltaTime);

        }
    }
}
#endif