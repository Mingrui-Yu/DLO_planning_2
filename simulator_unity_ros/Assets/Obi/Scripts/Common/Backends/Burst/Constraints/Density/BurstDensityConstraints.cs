#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;
using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Obi
{
    public class BurstDensityConstraints : BurstConstraintsImpl<BurstDensityConstraintsBatch>
    {
        public NativeList<int> fluidParticles;

        public NativeArray<float4> eta;
        public NativeArray<float4> smoothPositions;
        public NativeArray<float3x3> anisotropies;

        public BurstDensityConstraints(BurstSolverImpl solver) : base(solver, Oni.ConstraintType.Density)
        {
            fluidParticles = new NativeList<int>(Allocator.Persistent);
        }

        public override IConstraintsBatchImpl CreateConstraintsBatch()
        {
            var dataBatch = new BurstDensityConstraintsBatch(this);
            batches.Add(dataBatch);
            return dataBatch;
        }

        public override void Dispose()
        {
            fluidParticles.Dispose();
        }

        public override void RemoveBatch(IConstraintsBatchImpl batch)
        {
            batches.Remove(batch as BurstDensityConstraintsBatch);
            batch.Destroy();
        }

        protected override JobHandle EvaluateSequential(JobHandle inputDeps, float deltaTime)
        {
            return EvaluateParallel(inputDeps,deltaTime);
        }

        protected override JobHandle EvaluateParallel(JobHandle inputDeps, float deltaTime)
        {
            inputDeps = UpdateInteractions(inputDeps);

            // evaluate all batches as a chain of dependencies:
            for (int i = 0; i < batches.Count; ++i)
            {
                inputDeps = batches[i].Evaluate(inputDeps, deltaTime);
                m_Solver.ScheduleBatchedJobsIfNeeded();
            }

            // calculate per-particle lambdas:
            inputDeps = CalculateLambdas(inputDeps, deltaTime);

            // then apply them:
            for (int i = 0; i < batches.Count; ++i)
            {
                inputDeps = batches[i].Apply(inputDeps, deltaTime);
                m_Solver.ScheduleBatchedJobsIfNeeded();
            }

            return inputDeps;
        }

        public JobHandle ApplyVelocityCorrections(JobHandle inputDeps, float deltaTime)
        {
            eta = new NativeArray<float4>(((BurstSolverImpl)solver).particleCount, Allocator.TempJob);

            for (int i = 0; i < batches.Count; ++i)
            {
                inputDeps = batches[i].CalculateViscosityAndNormals(inputDeps, deltaTime);
                m_Solver.ScheduleBatchedJobsIfNeeded();
            }

            for (int i = 0; i < batches.Count; ++i)
            {
                inputDeps = batches[i].CalculateVorticity(inputDeps);
                m_Solver.ScheduleBatchedJobsIfNeeded();
            }

            inputDeps = ApplyVorticityAndAtmosphere(inputDeps, deltaTime);
            m_Solver.ScheduleBatchedJobsIfNeeded();

            return inputDeps;
        }

        public JobHandle CalculateAnisotropyLaplacianSmoothing(JobHandle inputDeps)
        {
            // if the constraints are deactivated or we need no anisotropy:
            if (((BurstSolverImpl)solver).abstraction.parameters.maxAnisotropy <= 1)
                return IdentityAnisotropy(inputDeps);

            smoothPositions = new NativeArray<float4>(((BurstSolverImpl)solver).particleCount, Allocator.TempJob);
            anisotropies = new NativeArray<float3x3>(((BurstSolverImpl)solver).particleCount, Allocator.TempJob);

            for (int i = 0; i < batches.Count; ++i)
            {
                inputDeps = batches[i].AccumulateSmoothPositions(inputDeps);
                m_Solver.ScheduleBatchedJobsIfNeeded();
            }

            inputDeps = AverageSmoothPositions(inputDeps);

            for (int i = 0; i < batches.Count; ++i)
            {
                inputDeps = batches[i].AccumulateAnisotropy(inputDeps);
                m_Solver.ScheduleBatchedJobsIfNeeded();
            }
             
            return AverageAnisotropy(inputDeps);
        }

        private JobHandle UpdateInteractions(JobHandle inputDeps)
        {
            // clear existing fluid data:
            var clearData = new ClearFluidDataJob()
            {
                fluidParticles = fluidParticles.AsDeferredJobArray(),
                fluidData = ((BurstSolverImpl)solver).abstraction.fluidData.AsNativeArray<float4>(),
            };

            inputDeps = clearData.Schedule(fluidParticles.Length, 64, inputDeps);

            // update fluid interactions:
            var updateInteractions = new UpdateInteractionsJob()
            {
                pairs = m_Solver.fluidInteractions,
                positions = m_Solver.positions,
                radii = m_Solver.smoothingRadii,
                densityKernel = new Poly6Kernel(((BurstSolverImpl)solver).abstraction.parameters.mode == Oni.SolverParameters.Mode.Mode2D),
                gradientKernel = new SpikyKernel(((BurstSolverImpl)solver).abstraction.parameters.mode == Oni.SolverParameters.Mode.Mode2D),
            };

            return updateInteractions.Schedule(((BurstSolverImpl)solver).fluidInteractions.Length, 64, inputDeps);
        }

        private JobHandle CalculateLambdas(JobHandle inputDeps, float deltaTime)
        {
            // calculate lagrange multipliers:
            var calculateLambdas = new CalculateLambdasJob()
            {
                fluidParticles = fluidParticles.AsDeferredJobArray(),
                invMasses = m_Solver.invMasses,
                radii = m_Solver.smoothingRadii,
                restDensities = m_Solver.restDensities,
                surfaceTension = m_Solver.surfaceTension,
                densityKernel = new Poly6Kernel(m_Solver.abstraction.parameters.mode == Oni.SolverParameters.Mode.Mode2D),
                gradientKernel = new SpikyKernel(m_Solver.abstraction.parameters.mode == Oni.SolverParameters.Mode.Mode2D),
                normals = m_Solver.normals,
                vorticity = m_Solver.vorticities,
                fluidData = m_Solver.fluidData
            };

            return calculateLambdas.Schedule(fluidParticles.Length,64,inputDeps);
        }

        private JobHandle ApplyVorticityAndAtmosphere(JobHandle inputDeps, float deltaTime)
        {
            // calculate lagrange multipliers:
            var conf = new ApplyVorticityConfinementAndAtmosphere()
            {
                fluidParticles = fluidParticles.AsDeferredJobArray(),
                wind = m_Solver.wind,
                vorticities = m_Solver.vorticities,
                eta = eta,
                atmosphericDrag = m_Solver.athmosphericDrag,
                atmosphericPressure = m_Solver.athmosphericPressure,
                vorticityConfinement = m_Solver.vortConfinement,
                restDensities = m_Solver.restDensities,
                normals = m_Solver.normals,
                fluidData = m_Solver.fluidData,
                velocities = m_Solver.velocities,
                dt = deltaTime
            };

            return conf.Schedule(fluidParticles.Length, 64, inputDeps);
        }

        private JobHandle IdentityAnisotropy(JobHandle inputDeps)
        {
            var idAnisotropy = new IdentityAnisotropyJob()
            {
                fluidParticles = fluidParticles.AsDeferredJobArray(),
                principalAxes = m_Solver.anisotropies,
                radii = m_Solver.principalRadii
            };

            return idAnisotropy.Schedule(fluidParticles.Length, 64, inputDeps);
        }

        private JobHandle AverageSmoothPositions(JobHandle inputDeps)
        {
            var average = new AverageSmoothPositionsJob()
            {
                fluidParticles = fluidParticles.AsDeferredJobArray(),
                renderablePositions = m_Solver.renderablePositions,
                smoothPositions = smoothPositions
            };

            return average.Schedule(fluidParticles.Length, 64, inputDeps);
        }

        private JobHandle AverageAnisotropy(JobHandle inputDeps)
        {
            var average = new AverageAnisotropyJob()
            {
                fluidParticles = fluidParticles.AsDeferredJobArray(),
                renderablePositions = m_Solver.renderablePositions,
                smoothPositions = smoothPositions,
                principalRadii = m_Solver.principalRadii,
                anisotropies = anisotropies,
                maxAnisotropy = m_Solver.abstraction.parameters.maxAnisotropy,
                principalAxes = m_Solver.anisotropies
            };

            return average.Schedule(fluidParticles.Length, 64, inputDeps);
        }

        [BurstCompile]
        public struct ClearFluidDataJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> fluidParticles;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> fluidData;

            public void Execute(int i)
            {
                fluidData[fluidParticles[i]] = float4.zero;
            }
        }

        [BurstCompile]
        public struct UpdateInteractionsJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float4> positions;
            [ReadOnly] public NativeArray<float> radii;
            [ReadOnly] public Poly6Kernel densityKernel;
            [ReadOnly] public SpikyKernel gradientKernel;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<FluidInteraction> pairs;

            [ReadOnly] public BatchData batchData;

            public void Execute(int i)
            {
                var pair = pairs[i];

                // calculate normalized gradient vector:
                pair.gradient = (positions[pair.particleA] - positions[pair.particleB]);
                float distance = math.length(pair.gradient);
                pair.gradient /= distance + math.FLT_MIN_NORMAL;

                // calculate and store average density and gradient kernels:
                pair.avgKernel = (densityKernel.W(distance, radii[pair.particleA]) +
                                  densityKernel.W(distance, radii[pair.particleB])) * 0.5f;

                pair.avgGradient = (gradientKernel.W(distance, radii[pair.particleA]) +
                                    gradientKernel.W(distance, radii[pair.particleB])) * 0.5f;

                pairs[i] = pair;
            }
        }

        [BurstCompile]
        public struct CalculateLambdasJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> fluidParticles;
            [ReadOnly] public NativeArray<float> invMasses;
            [ReadOnly] public NativeArray<float> radii;
            [ReadOnly] public NativeArray<float> restDensities;
            [ReadOnly] public NativeArray<float> surfaceTension;
            [ReadOnly] public Poly6Kernel densityKernel;
            [ReadOnly] public SpikyKernel gradientKernel; 

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> normals;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> vorticity;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> fluidData;

            public void Execute(int p)
            {
                int i = fluidParticles[p];

                normals[i] = float4.zero;
                vorticity[i] = float4.zero;

                float4 data = fluidData[i];

                float grad = gradientKernel.W(0, radii[i]) / invMasses[i] / restDensities[i];

                // self particle contribution to density and gradient:
                data += new float4(densityKernel.W(0, radii[i]), 0, grad, grad * grad + data[2] * data[2]);

                // weight by mass:
                data[0] /= invMasses[i];

                // evaluate density constraint (clamp pressure):
                float constraint = math.max(-0.5f * surfaceTension[i], data[0] / restDensities[i] - 1);

                // calculate lambda:
                data[1] = -constraint / (invMasses[i] * data[3] + math.FLT_MIN_NORMAL);

                fluidData[i] = data;
            }
        }

        [BurstCompile]
        public struct ApplyVorticityConfinementAndAtmosphere : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> fluidParticles;
            [ReadOnly] public NativeArray<float4> wind;
            [ReadOnly] public NativeArray<float4> vorticities;
            [ReadOnly] public NativeArray<float> atmosphericDrag;
            [ReadOnly] public NativeArray<float> atmosphericPressure;
            [ReadOnly] public NativeArray<float> vorticityConfinement;
            [ReadOnly] public NativeArray<float> restDensities;
            [ReadOnly] public NativeArray<float4> normals;
            [ReadOnly] public NativeArray<float4> fluidData;

            [DeallocateOnJobCompletion] [ReadOnly] public NativeArray<float4> eta;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> velocities;

            [ReadOnly] public float dt;

            public void Execute(int p)
            {
                int i = fluidParticles[p];

                //atmospheric drag:
                float4 velocityDiff = velocities[i] - wind[i];

                // particles near the surface should experience drag:
                velocities[i] -= atmosphericDrag[i] * velocityDiff * math.max(0, 1 - fluidData[i][0] / restDensities[i]) * dt;

                // ambient pressure:
                velocities[i] += atmosphericPressure[i] * normals[i] * dt;

                // apply vorticity confinement:
                velocities[i] += new float4(math.cross(math.normalizesafe(eta[i]).xyz,vorticities[i].xyz), 0) * vorticityConfinement[i] * dt;
            }
        }

        [BurstCompile]
        public struct IdentityAnisotropyJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> fluidParticles;
            [ReadOnly] public NativeArray<float4> radii;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> principalAxes;

            public void Execute(int p)
            {
                int i = fluidParticles[p];

                // align the principal axes of the particle with the solver axes:
                principalAxes[i * 3]     = new float4(1,0,0,radii[i].x);
                principalAxes[i * 3 + 1] = new float4(0,1,0,radii[i].x);
                principalAxes[i * 3 + 2] = new float4(0,0,1,radii[i].x);
            }
        }

        [BurstCompile]
        public struct AverageSmoothPositionsJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> fluidParticles;
            [ReadOnly] public NativeArray<float4> renderablePositions;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> smoothPositions;

            public void Execute(int p)
            {
                int i = fluidParticles[p];

                if (smoothPositions[i].w > 0)
                    smoothPositions[i] /= smoothPositions[i].w;
                else
                    smoothPositions[i] = renderablePositions[i];
            }
        }

        [BurstCompile]
        public struct AverageAnisotropyJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> fluidParticles;
            [ReadOnly] public NativeArray<float4> principalRadii;
            [ReadOnly] public float maxAnisotropy;

            [ReadOnly]
            [DeallocateOnJobCompletion]
            public NativeArray<float4> smoothPositions;

            [ReadOnly]
            [DeallocateOnJobCompletion]
            public NativeArray<float3x3> anisotropies;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> renderablePositions;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> principalAxes;

            public void Execute(int p)
            {
                int i = fluidParticles[p];

                if (smoothPositions[i].w > 0)
                {
                    float3 singularValues;
                    float3x3 u;
                    BurstMath.EigenSolve(anisotropies[i] / smoothPositions[i].w, out singularValues, out u); //TODO: smoothPositions.w is always 1? we divided it all by w in AverageSmoothPositionsJob...

                    float max = singularValues[0];
                    float3 s = math.max(singularValues,new float3(max / maxAnisotropy)) / max * principalRadii[i].x;

                    principalAxes[i * 3]     = new float4(u.c0, s.x);
                    principalAxes[i * 3 + 1] = new float4(u.c1, s.y);
                    principalAxes[i * 3 + 2] = new float4(u.c2, s.z);
                }
                else
                {
                    float radius = principalRadii[i].x / maxAnisotropy;
                    principalAxes[i * 3]     = new float4(1, 0, 0, radius);
                    principalAxes[i * 3 + 1] = new float4(0, 1, 0, radius);
                    principalAxes[i * 3 + 2] = new float4(0, 0, 1, radius);
                }

                renderablePositions[i] = smoothPositions[i];
            }
        }
    }
}
#endif