#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using UnityEngine;
using Unity.Jobs;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Burst;
using System.Collections;

namespace Obi
{
    public class BurstDensityConstraintsBatch : BurstConstraintsBatchImpl, IDensityConstraintsBatchImpl
    {
        public BatchData batchData;

        public BurstDensityConstraintsBatch(BurstDensityConstraints constraints)
        {
            m_Constraints = constraints;
            m_ConstraintType = Oni.ConstraintType.Density;
        }

        public override JobHandle Initialize(JobHandle inputDeps, float deltaTime)
        {
            return inputDeps;
        }

        public override JobHandle Evaluate(JobHandle inputDeps, float deltaTime)
        {

            // update densities and gradients:
            var updateDensities = new UpdateDensitiesJob()
            {
                pairs = ((BurstSolverImpl)constraints.solver).fluidInteractions,
                positions = solverImplementation.positions,
                invMasses = solverImplementation.invMasses,
                restDensities = solverImplementation.restDensities,
                diffusion = solverImplementation.diffusion,
                userData = solverImplementation.userData,
                fluidData = solverImplementation.fluidData,
                batchData = batchData,
                dt = deltaTime
            };

            int batchCount = batchData.isLast ? batchData.workItemCount : 1;
            return updateDensities.Schedule(batchData.workItemCount, batchCount, inputDeps);
        }

        public override JobHandle Apply(JobHandle inputDeps, float deltaTime)
        {
            var parameters = solverAbstraction.GetConstraintParameters(m_ConstraintType);

            // update densities and gradients:
            var apply = new ApplyDensityConstraintsJob()
            {
                invMasses = solverImplementation.invMasses,
                radii = solverImplementation.smoothingRadii,
                restDensities = solverImplementation.restDensities,
                surfaceTension = solverImplementation.surfaceTension,
                pairs = ((BurstSolverImpl)constraints.solver).fluidInteractions,
                densityKernel = new Poly6Kernel(solverAbstraction.parameters.mode == Oni.SolverParameters.Mode.Mode2D),
                positions = solverImplementation.positions,
                fluidData = solverImplementation.fluidData,
                batchData = batchData,
                sorFactor = parameters.SORFactor
            };

            int batchCount = batchData.isLast ? batchData.workItemCount : 1;
            return apply.Schedule(batchData.workItemCount, batchCount, inputDeps);
        }

        public JobHandle CalculateViscosityAndNormals(JobHandle inputDeps, float deltaTime)
        {
            var viscosity = new NormalsViscosityAndVorticityJob()
            {
                positions = solverImplementation.positions,
                invMasses = solverImplementation.invMasses,
                radii = solverImplementation.smoothingRadii,
                restDensities = solverImplementation.restDensities,
                viscosities = solverImplementation.viscosities,
                fluidData = solverImplementation.fluidData,
                pairs = ((BurstSolverImpl)constraints.solver).fluidInteractions,
                velocities = solverImplementation.velocities,
                vorticities = solverImplementation.vorticities,
                normals = solverImplementation.normals,
                batchData = batchData
            };

            int batchCount = batchData.isLast ? batchData.workItemCount : 1;
            return viscosity.Schedule(batchData.workItemCount, batchCount, inputDeps);
        }

        public JobHandle CalculateVorticity(JobHandle inputDeps)
        {
            var eta = new CalculateVorticityEta()
            {
                invMasses = solverImplementation.invMasses,
                restDensities = solverImplementation.restDensities,
                pairs = ((BurstSolverImpl)constraints.solver).fluidInteractions,
                vorticities = solverImplementation.vorticities,
                eta = ((BurstDensityConstraints)this.constraints).eta,
                batchData = batchData
            };

            int batchCount = batchData.isLast ? batchData.workItemCount : 1;
            return eta.Schedule(batchData.workItemCount, batchCount, inputDeps);
        }

        public JobHandle AccumulateSmoothPositions(JobHandle inputDeps)
        {
            var accumulateSmooth = new AccumulateSmoothPositionsJob()
            {
                renderablePositions = solverImplementation.renderablePositions,
                smoothPositions = ((BurstDensityConstraints)this.constraints).smoothPositions,
                radii = solverImplementation.smoothingRadii,
                densityKernel = new Poly6Kernel(solverAbstraction.parameters.mode == Oni.SolverParameters.Mode.Mode2D),
                pairs = ((BurstSolverImpl)constraints.solver).fluidInteractions,
                batchData = batchData
            };

            int batchCount = batchData.isLast ? batchData.workItemCount : 1;
            return accumulateSmooth.Schedule(batchData.workItemCount, batchCount, inputDeps);
        }

        public JobHandle AccumulateAnisotropy(JobHandle inputDeps)
        {
            var accumulateAnisotropy = new AccumulateAnisotropyJob()
            {
                renderablePositions = solverImplementation.renderablePositions,
                smoothPositions = ((BurstDensityConstraints)this.constraints).smoothPositions,
                anisotropies = ((BurstDensityConstraints)this.constraints).anisotropies,
                pairs = ((BurstSolverImpl)constraints.solver).fluidInteractions,
                batchData = batchData
            };

            int batchCount = batchData.isLast ? batchData.workItemCount : 1;
            return accumulateAnisotropy.Schedule(batchData.workItemCount, batchCount, inputDeps);
        }

        [BurstCompile]
        public struct UpdateDensitiesJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float4> positions;
            [ReadOnly] public NativeArray<float> invMasses;
            [ReadOnly] public NativeArray<float> restDensities;
            [ReadOnly] public NativeArray<float> diffusion;
            [ReadOnly] public NativeArray<FluidInteraction> pairs;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> userData;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> fluidData;

            [ReadOnly] public BatchData batchData;

            [ReadOnly] public float dt;

            public void Execute(int workItemIndex)
            {
                int start, end;
                batchData.GetConstraintRange(workItemIndex, out start, out end);

                for (int i = start; i < end; ++i)
                {
                    var pair = pairs[i];

                    float restVolumeA = 1.0f / invMasses[pair.particleA] / restDensities[pair.particleA];
                    float restVolumeB = 1.0f / invMasses[pair.particleB] / restDensities[pair.particleB];

                    float gradA = restVolumeB * pair.avgGradient;
                    float gradB = restVolumeA * pair.avgGradient;

                    float vA = restVolumeB / restVolumeA;
                    float vB = restVolumeA / restVolumeB;

                    // accumulate pbf data (density, gradients):
                    fluidData[pair.particleA] += new float4(vA * pair.avgKernel, 0, gradA, gradA * gradA);
                    fluidData[pair.particleB] += new float4(vB * pair.avgKernel, 0, gradB, gradB * gradB);

                    // property diffusion:
                    float diffusionSpeed = (diffusion[pair.particleA] + diffusion[pair.particleB]) * pair.avgKernel * dt;
                    float4 userDelta = (userData[pair.particleB] - userData[pair.particleA]) * diffusionSpeed;
                    userData[pair.particleA] += vA * userDelta;
                    userData[pair.particleB] -= vB * userDelta;
                }
            }
        }

        [BurstCompile]
        public struct ApplyDensityConstraintsJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float> invMasses;
            [ReadOnly] public NativeArray<float> radii;
            [ReadOnly] public NativeArray<float> restDensities;
            [ReadOnly] public NativeArray<float> surfaceTension;
            [ReadOnly] public NativeArray<FluidInteraction> pairs;
            [ReadOnly] public Poly6Kernel densityKernel;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> positions;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> fluidData;

            [ReadOnly] public BatchData batchData;
            [ReadOnly] public float sorFactor;

            public void Execute(int workItemIndex)
            {
                int start, end;
                batchData.GetConstraintRange(workItemIndex, out start, out end);

                for (int i = start; i < end; ++i)
                {
                    var pair = pairs[i];

                    float restVolumeA = 1.0f / invMasses[pair.particleA] / restDensities[pair.particleA];
                    float restVolumeB = 1.0f / invMasses[pair.particleB] / restDensities[pair.particleB];

                    // calculate tensile instability correction factor:
                    float wAvg = pair.avgKernel / ((densityKernel.W(0, radii[pair.particleA]) + densityKernel.W(0, radii[pair.particleB])) * 0.5f);
                    float scorrA = -(0.001f + 0.2f * surfaceTension[pair.particleA]) * wAvg / (invMasses[pair.particleA] * fluidData[pair.particleA][3]);
                    float scorrB = -(0.001f + 0.2f * surfaceTension[pair.particleB]) * wAvg / (invMasses[pair.particleB] * fluidData[pair.particleB][3]);

                    // calculate position delta:
                    float4 delta = pair.gradient * pair.avgGradient * ((fluidData[pair.particleA][1] + scorrA) * restVolumeB + (fluidData[pair.particleB][1] + scorrB) * restVolumeA) * sorFactor;
                    positions[pair.particleA] += delta * invMasses[pair.particleA];
                    positions[pair.particleB] -= delta * invMasses[pair.particleB];
                }
            }
        }

        [BurstCompile]
        public struct NormalsViscosityAndVorticityJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float4> positions;
            [ReadOnly] public NativeArray<float> invMasses;
            [ReadOnly] public NativeArray<float> radii;
            [ReadOnly] public NativeArray<float> restDensities;
            [ReadOnly] public NativeArray<float> viscosities;
            [ReadOnly] public NativeArray<float4> fluidData;
            [ReadOnly] public NativeArray<FluidInteraction> pairs;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> velocities;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> vorticities;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> normals;

            [ReadOnly] public BatchData batchData;

            public void Execute(int workItemIndex)
            {
                int start, end;
                batchData.GetConstraintRange(workItemIndex, out start, out end);

                for (int i = start; i < end; ++i)
                {
                    var pair = pairs[i];

                    float restVolumeA = 1.0f / invMasses[pair.particleA] / restDensities[pair.particleA];
                    float restVolumeB = 1.0f / invMasses[pair.particleB] / restDensities[pair.particleB];

                    // XSPH viscosity:
                    float viscosityCoeff = math.min(viscosities[pair.particleA], viscosities[pair.particleB]);
                    float4 relVelocity = velocities[pair.particleB] - velocities[pair.particleA];
                    float4 viscosity = viscosityCoeff * relVelocity * pair.avgKernel;
                    velocities[pair.particleA] += viscosity * restVolumeB;
                    velocities[pair.particleB] -= viscosity * restVolumeA;

                    // calculate vorticity:
                    float4 vgrad = pair.gradient * pair.avgGradient;
                    float4 vorticity = new float4(math.cross(relVelocity.xyz,vgrad.xyz),0);
                    vorticities[pair.particleA] += vorticity * restVolumeB;
                    vorticities[pair.particleB] += vorticity * restVolumeA;

                    // calculate color field  normal:
                    float radius = (radii[pair.particleA] + radii[pair.particleB]) * 0.5f;
                    normals[pair.particleA] += vgrad * radius / invMasses[pair.particleB] / fluidData[pair.particleB][0];
                    normals[pair.particleB] -= vgrad * radius / invMasses[pair.particleA] / fluidData[pair.particleA][0];
                }
            }
        }

        [BurstCompile]
        public struct CalculateVorticityEta : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float4> vorticities;
            [ReadOnly] public NativeArray<float> invMasses;
            [ReadOnly] public NativeArray<float> restDensities;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<FluidInteraction> pairs;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> eta;

            [ReadOnly] public BatchData batchData;

            public void Execute(int workItemIndex)
            {
                int start, end;
                batchData.GetConstraintRange(workItemIndex, out start, out end);

                for (int i = start; i < end; ++i)
                {
                    var pair = pairs[i];

                    float4 vgrad = pair.gradient * pair.avgGradient;
                    eta[pair.particleA] += math.length(vorticities[pair.particleA]) * vgrad / invMasses[pair.particleB] / restDensities[pair.particleB];
                    eta[pair.particleB] -= math.length(vorticities[pair.particleB]) * vgrad / invMasses[pair.particleA] / restDensities[pair.particleA];
                }
            }
        }

        [BurstCompile]
        public struct AccumulateSmoothPositionsJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float4> renderablePositions;
            [ReadOnly] public NativeArray<float> radii;
            [ReadOnly] public Poly6Kernel densityKernel;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> smoothPositions;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<FluidInteraction> pairs;

            [ReadOnly] public BatchData batchData;

            public void Execute(int workItemIndex)
            {
                int start, end;
                batchData.GetConstraintRange(workItemIndex, out start, out end);

                for (int i = start; i < end; ++i)
                {
                    var pair = pairs[i];

                    float4 gradient = (renderablePositions[pair.particleA] - renderablePositions[pair.particleB]);
                    float distance = math.length(gradient);

                    pair.avgKernel = (densityKernel.W(distance, radii[pair.particleA]) +
                                      densityKernel.W(distance, radii[pair.particleB])) * 0.5f;

                    smoothPositions[pair.particleA] += new float4(renderablePositions[pair.particleB].xyz,1) * pair.avgKernel;
                    smoothPositions[pair.particleB] += new float4(renderablePositions[pair.particleA].xyz,1) * pair.avgKernel;

                    pairs[i] = pair;
                }
            }
        }

        [BurstCompile]
        public struct AccumulateAnisotropyJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float4> renderablePositions;
            [ReadOnly] public NativeArray<float4> smoothPositions;
            [ReadOnly] public NativeArray<FluidInteraction> pairs;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float3x3> anisotropies;

            [ReadOnly] public BatchData batchData;

            public void Execute(int workItemIndex)
            {
                int start, end;
                batchData.GetConstraintRange(workItemIndex, out start, out end);

                for (int i = start; i < end; ++i)
                {
                    var pair = pairs[i];

                    float4 distanceA = renderablePositions[pair.particleB] - smoothPositions[pair.particleA];
                    float4 distanceB = renderablePositions[pair.particleA] - smoothPositions[pair.particleB];

                    anisotropies[pair.particleA] += BurstMath.multrnsp(distanceA,distanceA) * pair.avgKernel;
                    anisotropies[pair.particleB] += BurstMath.multrnsp(distanceB,distanceB) * pair.avgKernel;
                }
            }
        }

    }
}
#endif