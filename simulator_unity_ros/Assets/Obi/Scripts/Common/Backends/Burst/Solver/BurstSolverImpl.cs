#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using UnityEngine;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Collections;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    public class BurstSolverImpl : ISolverImpl
    {
        ObiSolver m_Solver;

        public ObiSolver abstraction
        {
            get { return m_Solver; }
        }

        public int particleCount
        {
            get { return m_Solver.positions.count; }
        }

        public int activeParticleCount
        {
            get { return activeParticles.Length; }
        }

        public BurstInertialFrame inertialFrame
        {
            get { return m_InertialFrame; }
        }

        public BurstAffineTransform solverToWorld
        {
            get { return m_InertialFrame.frame; }
        }

        public BurstAffineTransform worldToSolver
        {
            get { return m_InertialFrame.frame.Inverse(); }
        }

        private const int maxBatches = 17;

        private ConstraintBatcher constraintBatcher;

        // Per-type constraints array:
        IBurstConstraintsImpl[] constraints;

        // particle contact generation:
        public ParticleGrid particleGrid;
        public NativeArray<BurstContact> particleContacts;
        public NativeArray<BatchData> particleBatchData;

        // fluid interaction generation:
        public NativeArray<FluidInteraction> fluidInteractions;
        public NativeArray<BatchData> fluidBatchData;

        // collider contact generation:
        private BurstColliderWorld colliderGrid;
        public NativeArray<BurstContact> colliderContacts;

        // misc data:
        public NativeList<int> activeParticles;
        private NativeList<int> deformableTriangles;
        private BurstInertialFrame m_InertialFrame; // local to world inertial frame.
        private int scheduledJobCounter = 0;

        // cached particle data arrays (just wrappers over raw unmanaged data held by the abstract solver)
        public NativeArray<float4> positions;
        public NativeArray<float4> restPositions;
        public NativeArray<float4> prevPositions;
        public NativeArray<float4> renderablePositions;

        public NativeArray<quaternion> orientations;
        public NativeArray<quaternion> restOrientations;
        public NativeArray<quaternion> prevOrientations;
        public NativeArray<quaternion> renderableOrientations;

        public NativeArray<float4> velocities;
        public NativeArray<float4> angularVelocities;

        public NativeArray<float> invMasses;
        public NativeArray<float> invRotationalMasses;
        public NativeArray<float4> invInertiaTensors;

        public NativeArray<float4> externalForces;
        public NativeArray<float4> externalTorques;
        public NativeArray<float4> wind;

        public NativeArray<float4> positionDeltas;
        public NativeArray<quaternion> orientationDeltas;
        public NativeArray<int> positionConstraintCounts;
        public NativeArray<int> orientationConstraintCounts;

        public NativeArray<int> collisionMaterials;
        public NativeArray<int> phases;
        public NativeArray<float4> anisotropies;
        public NativeArray<float4> principalRadii;
        public NativeArray<float4> normals;

        public NativeArray<float4> vorticities;
        public NativeArray<float4> fluidData;
        public NativeArray<float4> userData;
        public NativeArray<float> smoothingRadii;
        public NativeArray<float> buoyancies;
        public NativeArray<float> restDensities;
        public NativeArray<float> viscosities;
        public NativeArray<float> surfaceTension;
        public NativeArray<float> vortConfinement;
        public NativeArray<float> athmosphericDrag;
        public NativeArray<float> athmosphericPressure;
        public NativeArray<float> diffusion;

        public NativeArray<int4> cellCoords;

        public BurstSolverImpl(ObiSolver solver)
        {
            this.m_Solver = solver;

            // Initialize collision world:
            GetOrCreateColliderWorld();
            colliderGrid.IncreaseReferenceCount();

            activeParticles = new NativeList<int>(64, Allocator.Persistent);
            deformableTriangles = new NativeList<int>(64, Allocator.Persistent);

            // Initialize contact generation acceleration structure:
            particleGrid = new ParticleGrid();

            // Initialize constraint batcher:
            constraintBatcher = new ConstraintBatcher(maxBatches);

            // Initialize constraint arrays:
            constraints = new IBurstConstraintsImpl[Oni.ConstraintTypeCount];
            constraints[(int)Oni.ConstraintType.Tether] = new BurstTetherConstraints(this);
            constraints[(int)Oni.ConstraintType.Volume] = new BurstVolumeConstraints(this);
            constraints[(int)Oni.ConstraintType.Chain] = new BurstChainConstraints(this);
            constraints[(int)Oni.ConstraintType.Bending] = new BurstBendConstraints(this);
            constraints[(int)Oni.ConstraintType.Distance] = new BurstDistanceConstraints(this);
            constraints[(int)Oni.ConstraintType.ShapeMatching] = new BurstShapeMatchingConstraints(this);
            constraints[(int)Oni.ConstraintType.BendTwist] = new BurstBendTwistConstraints(this);
            constraints[(int)Oni.ConstraintType.StretchShear] = new BurstStretchShearConstraints(this);
            constraints[(int)Oni.ConstraintType.Pin] = new BurstPinConstraints(this);
            constraints[(int)Oni.ConstraintType.ParticleCollision] = new BurstParticleCollisionConstraints(this);
            constraints[(int)Oni.ConstraintType.Density] = new BurstDensityConstraints(this);
            constraints[(int)Oni.ConstraintType.Collision] = new BurstColliderCollisionConstraints(this);
            constraints[(int)Oni.ConstraintType.Skin] = new BurstSkinConstraints(this);
            constraints[(int)Oni.ConstraintType.Aerodynamics] = new BurstAerodynamicConstraints(this);
            constraints[(int)Oni.ConstraintType.Stitch] = new BurstStitchConstraints(this);
            constraints[(int)Oni.ConstraintType.ParticleFriction] = new BurstParticleFrictionConstraints(this);
            constraints[(int)Oni.ConstraintType.Friction] = new BurstColliderFrictionConstraints(this);

            var c = constraints[(int)Oni.ConstraintType.Collision] as BurstColliderCollisionConstraints;
            c.CreateConstraintsBatch();

            var f = constraints[(int)Oni.ConstraintType.Friction] as BurstColliderFrictionConstraints;
            f.CreateConstraintsBatch();
        }

        public void Destroy()
        {
            for (int i = 0; i < constraints.Length; ++i)
                if (constraints[i] != null)
                    constraints[i].Dispose();

            // Get rid of particle and collider grids:
            particleGrid.Dispose();

            if (colliderGrid != null)
                colliderGrid.DecreaseReferenceCount();

            constraintBatcher.Dispose();

            if (activeParticles.IsCreated)
                activeParticles.Dispose();
            if (deformableTriangles.IsCreated)
                deformableTriangles.Dispose();

            if (particleContacts.IsCreated)
                particleContacts.Dispose();
            if (particleBatchData.IsCreated)
                particleBatchData.Dispose();
            if (fluidInteractions.IsCreated)
                fluidInteractions.Dispose();
            if (fluidBatchData.IsCreated)
                fluidBatchData.Dispose();
            if (colliderContacts.IsCreated)
                colliderContacts.Dispose();

        }

        // Utility function to count scheduled jobs. Call it once per job.
        // Will JobHandle.ScheduleBatchedJobs once there's a good bunch of scheduled jobs.
        public void ScheduleBatchedJobsIfNeeded()
        {
            if (scheduledJobCounter++ > 16)
            {
                scheduledJobCounter = 0;
                JobHandle.ScheduleBatchedJobs();
            }
        }
  
        private void GetOrCreateColliderWorld()
        {
            colliderGrid = GameObject.FindObjectOfType<BurstColliderWorld>();
            if (colliderGrid == null)
            {
                var world = new GameObject("BurstCollisionWorld", typeof(BurstColliderWorld));
                colliderGrid = world.GetComponent<BurstColliderWorld>();
            }
        }

        public void InitializeFrame(Vector4 translation, Vector4 scale, Quaternion rotation)
        {
            m_InertialFrame = new BurstInertialFrame(translation, scale, rotation);
        } 

        public void UpdateFrame(Vector4 translation, Vector4 scale, Quaternion rotation, float deltaTime)
        {
            m_InertialFrame.Update(translation, scale, rotation, deltaTime);
        }

        public void ApplyFrame(float worldLinearInertiaScale, float worldAngularInertiaScale, float deltaTime)
        {
            // inverse linear part:
            float4x4 linear = float4x4.TRS(float3.zero, inertialFrame.frame.rotation, math.rcp(inertialFrame.frame.scale.xyz));
            float4x4 linearInv = math.transpose(linear);

            // non-inertial frame accelerations:
            float4 angularVel = math.mul(linearInv, math.mul(float4x4.Scale(inertialFrame.angularVelocity.xyz), linear)).diagonal();
            float4 eulerAccel = math.mul(linearInv, math.mul(float4x4.Scale(inertialFrame.angularAcceleration.xyz), linear)).diagonal();
            float4 inertialAccel = math.mul(linearInv, inertialFrame.acceleration);

            var applyInertialForces = new ApplyInertialForcesJob()
            {
                activeParticles = activeParticles,
                positions = positions,
                velocities = velocities,
                invMasses = invMasses,
                angularVel = angularVel,
                inertialAccel = inertialAccel,
                eulerAccel = eulerAccel,
                worldLinearInertiaScale = worldLinearInertiaScale,
                worldAngularInertiaScale = worldAngularInertiaScale,
                deltaTime = deltaTime,
            };

            applyInertialForces.Schedule(activeParticleCount, 64).Complete();
        }

        public int GetDeformableTriangleCount()
        {
            return deformableTriangles.Length / 3;
        }

        public void SetDeformableTriangles(int[] indices, int num, int destOffset)
        {
            if (destOffset + num >= deformableTriangles.Length / 3)
                deformableTriangles.ResizeUninitialized((destOffset + num) * 3);
            for (int i = 0; i < num * 3; ++i)
                deformableTriangles[i + destOffset * 3] = indices[i];
        }

        public void SetActiveParticles(int[] indices, int num)
        {
            int set = ClampArrayAccess(particleCount, num, 0);
            activeParticles.ResizeUninitialized(set);
            for (int i = 0; i < num; ++i)
                activeParticles[i] = indices[i];
            activeParticles.Sort();
        }

        int ClampArrayAccess(int size, int num, int offset)
        {
            return math.min(num, math.max(0, size - offset));
        }

        public int RemoveDeformableTriangles(int num, int sourceOffset)
        {
            if (deformableTriangles.IsCreated)
            {
                int amount = deformableTriangles.Length / 3;

                if (num < 0)
                {
                    deformableTriangles.Clear();
                    return amount;
                }

                int set = ClampArrayAccess(amount, num, sourceOffset);
                int end = sourceOffset + set;

                for (int i = end * 3 - 1; i >= sourceOffset * 3; --i)
                    deformableTriangles.RemoveAtSwapBack(i);

                return set;
            }
            return 0;
        }

        public JobHandle RecalculateInertiaTensors(JobHandle inputDeps)
        {
            var updateInertiaTensors = new UpdateInertiaTensorsJob()
            {
                activeParticles = activeParticles,
                inverseMasses = abstraction.invMasses.AsNativeArray<float>(),
                inverseRotationalMasses = abstraction.invRotationalMasses.AsNativeArray<float>(),
                principalRadii = abstraction.principalRadii.AsNativeArray<float4>(),
                inverseInertiaTensors = abstraction.invInertiaTensors.AsNativeArray<float4>(),
            };

            return updateInertiaTensors.Schedule(activeParticles.Length, 128, inputDeps);
        }

        public void GetBounds(ref Vector3 min, ref Vector3 max)
        {
            if (!activeParticles.IsCreated)
                return;

            int chunkSize = 4;
            NativeArray<BurstAabb> bounds = new NativeArray<BurstAabb>(activeParticles.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

            var particleBoundsJob = new ParticleToBoundsJob()
            {
                activeParticles = activeParticles,
                positions = positions,
                radii = principalRadii,
                bounds = bounds
            };

            JobHandle reduction = particleBoundsJob.Schedule(activeParticles.Length, 64);

            // parallel reduction:
            int chunks = bounds.Length;
            int stride = 1;

            while (chunks > 1)
            {
                var reductionJob = new BoundsReductionJob()
                {
                    bounds = bounds,
                    stride = stride,
                    size = chunkSize,
                };
                reduction = reductionJob.Schedule(chunks, 1, reduction);

                chunks = (int)math.ceil(chunks / (float)chunkSize);
                stride *= chunkSize;
            }

            reduction.Complete();

            // the parallel reduction leaves the final bounds in the first entry:
            if (bounds.Length > 0)
            {
                min = bounds[0].min.xyz;
                max = bounds[0].max.xyz;
            }

            bounds.Dispose();
        }

        public void ResetForces()
        {
            abstraction.externalForces.WipeToZero();
            abstraction.externalTorques.WipeToZero();
            abstraction.wind.WipeToZero();

            // We're at the end of a whole step (not a substep), so dispose of contact buffers:
            if (particleContacts.IsCreated)
                particleContacts.Dispose();
            if (particleBatchData.IsCreated)
                particleBatchData.Dispose();
            if (colliderContacts.IsCreated)
                colliderContacts.Dispose();
        }

        public int GetConstraintCount(Oni.ConstraintType type)
        {
            if ((int)type > 0 && (int)type < constraints.Length)
                return constraints[(int)type].GetConstraintCount();
            return 0;
        }

        public void GetCollisionContacts(Oni.Contact[] contacts, int count)
        {
            NativeArray<Oni.Contact>.Copy(colliderContacts.Reinterpret<Oni.Contact>(),0, contacts,0,count);
        }

        public void GetParticleCollisionContacts(Oni.Contact[] contacts, int count)
        {
            NativeArray<Oni.Contact>.Copy(particleContacts.Reinterpret<Oni.Contact>(),0, contacts,0,count);
        }

        public void SetParameters(Oni.SolverParameters parameters)
        {
            // No need to implement. This backend grabs parameters from the abstraction when it needs them.
        }

        public void SetConstraintGroupParameters(Oni.ConstraintType type, ref Oni.ConstraintParameters parameters)
        {
            // No need to implement. This backend grabs parameters from the abstraction when it needs them.
        }

        public void ParticleCountChanged(ObiSolver solver)
        {
            positions = abstraction.positions.AsNativeArray<float4>();
            restPositions = abstraction.restPositions.AsNativeArray<float4>();
            prevPositions = abstraction.prevPositions.AsNativeArray<float4>();
            renderablePositions = abstraction.renderablePositions.AsNativeArray<float4>();

            orientations = abstraction.orientations.AsNativeArray<quaternion>();
            restOrientations = abstraction.restOrientations.AsNativeArray<quaternion>();
            prevOrientations = abstraction.prevOrientations.AsNativeArray<quaternion>();
            renderableOrientations = abstraction.renderableOrientations.AsNativeArray<quaternion>();

            velocities = abstraction.velocities.AsNativeArray<float4>();
            angularVelocities = abstraction.angularVelocities.AsNativeArray<float4>();

            invMasses = abstraction.invMasses.AsNativeArray<float>();
            invRotationalMasses = abstraction.invRotationalMasses.AsNativeArray<float>();
            invInertiaTensors = abstraction.invInertiaTensors.AsNativeArray<float4>();

            externalForces = abstraction.externalForces.AsNativeArray<float4>();
            externalTorques = abstraction.externalTorques.AsNativeArray<float4>();
            wind = abstraction.wind.AsNativeArray<float4>();

            positionDeltas = abstraction.positionDeltas.AsNativeArray<float4>();
            orientationDeltas = abstraction.orientationDeltas.AsNativeArray<quaternion>();
            positionConstraintCounts = abstraction.positionConstraintCounts.AsNativeArray<int>();
            orientationConstraintCounts = abstraction.orientationConstraintCounts.AsNativeArray<int>();

            collisionMaterials = abstraction.collisionMaterials.AsNativeArray<int>();
            phases = abstraction.phases.AsNativeArray<int>();
            anisotropies = abstraction.anisotropies.AsNativeArray<float4>();
            principalRadii = abstraction.principalRadii.AsNativeArray<float4>();
            normals = abstraction.normals.AsNativeArray<float4>();

            vorticities = abstraction.vorticities.AsNativeArray<float4>();
            fluidData = abstraction.fluidData.AsNativeArray<float4>();
            userData = abstraction.userData.AsNativeArray<float4>();
            smoothingRadii = abstraction.smoothingRadii.AsNativeArray<float>();
            buoyancies = abstraction.buoyancies.AsNativeArray<float>();
            restDensities = abstraction.restDensities.AsNativeArray<float>();
            viscosities = abstraction.viscosities.AsNativeArray<float>();
            surfaceTension = abstraction.surfaceTension.AsNativeArray<float>();
            vortConfinement = abstraction.vortConfinement.AsNativeArray<float>();
            athmosphericDrag = abstraction.atmosphericDrag.AsNativeArray<float>();
            athmosphericPressure = abstraction.atmosphericPressure.AsNativeArray<float>();
            diffusion = abstraction.diffusion.AsNativeArray<float>();

            cellCoords = abstraction.cellCoords.AsNativeArray<int4>();
        }

        public void ParticleCapacityChanged(ObiSolver solver)
        {

        }

        public void SetRigidbodyArrays(ObiSolver solver)
        {
            // No need to implement. This backend grabs arrays from the abstraction when it needs them.
        }

        public IConstraintsBatchImpl CreateConstraintsBatch(Oni.ConstraintType type)
        {
            return constraints[(int)type].CreateConstraintsBatch();
        }

        public void DestroyConstraintsBatch(IConstraintsBatchImpl batch)
        {
            if (batch != null)
                constraints[(int)batch.constraintType].RemoveBatch(batch);
        }

        public IObiJobHandle CollisionDetection(float stepTime)
        {
            var fluidHandle = FindFluidParticles();

            var inertiaUpdate = RecalculateInertiaTensors(fluidHandle);

            return new BurstJobHandle(GenerateContacts(inertiaUpdate, stepTime));
        }

        protected JobHandle FindFluidParticles()
        {
            var d = constraints[(int)Oni.ConstraintType.Density] as BurstDensityConstraints;

            // Update positions:
            var findFluidJob = new FindFluidParticlesJob()
            {
                activeParticles = activeParticles,
                phases = m_Solver.phases.AsNativeArray<int>(),
                fluidParticles = d.fluidParticles,
            };

            return findFluidJob.Schedule();
        }



        protected JobHandle GenerateContacts(JobHandle inputDeps, float deltaTime)
        {
            // Dispose of previous fluid interactions.
            // We need fluid data during interpolation, for anisotropic fluid particles. For this reason,
            // we can't dispose of these arrays in ResetForces() at the end of each full step. They must use persistent allocation.

            if (fluidInteractions.IsCreated)
                fluidInteractions.Dispose();
            if (fluidBatchData.IsCreated)
                fluidBatchData.Dispose();

            // get constraint parameters for constraint types that depend on broadphases:
            var collisionParameters = m_Solver.GetConstraintParameters(Oni.ConstraintType.Collision);
            var particleCollisionParameters = m_Solver.GetConstraintParameters(Oni.ConstraintType.ParticleCollision);
            var densityParameters = m_Solver.GetConstraintParameters(Oni.ConstraintType.Density);

            // if no enabled constraints that require broadphase info, skip it entirely.
            if (collisionParameters.enabled ||
                particleCollisionParameters.enabled ||
                densityParameters.enabled)
            {

                // update particle grid structure:
                particleGrid.UpdateGrid(this, inputDeps);

                // generate particle and collider contacts in parallel:
                JobHandle generateParticleContactsHandle = inputDeps, generateContactsHandle = inputDeps;

                if (particleCollisionParameters.enabled || densityParameters.enabled)
                    generateParticleContactsHandle = particleGrid.GenerateContacts(this, deltaTime);

                if (collisionParameters.enabled)
                    generateContactsHandle = colliderGrid.GenerateContacts(this, deltaTime);

                JobHandle.CombineDependencies(generateParticleContactsHandle, generateContactsHandle).Complete();

                // allocate arrays for interactions and batch data:
                particleContacts = new NativeArray<BurstContact>(particleGrid.particleContactQueue.Count, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                particleBatchData = new NativeArray<BatchData>(maxBatches, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

                fluidInteractions = new NativeArray<FluidInteraction>(particleGrid.fluidInteractionQueue.Count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                fluidBatchData = new NativeArray<BatchData>(maxBatches, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

                colliderContacts = new NativeArray<BurstContact>(colliderGrid.colliderContactQueue.Count, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

                // dequeue contacts/interactions into temporary arrays:
                var rawParticleContacts = new NativeArray<BurstContact>(particleGrid.particleContactQueue.Count, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                var sortedParticleContacts = new NativeArray<BurstContact>(particleGrid.particleContactQueue.Count, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                var rawFluidInteractions = new NativeArray<FluidInteraction>(particleGrid.fluidInteractionQueue.Count, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

                DequeueIntoArrayJob<BurstContact> dequeueParticleContacts = new DequeueIntoArrayJob<BurstContact>()
                {
                    InputQueue = particleGrid.particleContactQueue,
                    OutputArray = rawParticleContacts
                };

                DequeueIntoArrayJob<FluidInteraction> dequeueFluidInteractions = new DequeueIntoArrayJob<FluidInteraction>()
                {
                    InputQueue = particleGrid.fluidInteractionQueue,
                    OutputArray = rawFluidInteractions
                };

                DequeueIntoArrayJob<BurstContact> dequeueColliderContacts = new DequeueIntoArrayJob<BurstContact>()
                {
                    InputQueue = colliderGrid.colliderContactQueue,
                    OutputArray = colliderContacts
                };

                var dequeueHandle = JobHandle.CombineDependencies(dequeueParticleContacts.Schedule(), dequeueFluidInteractions.Schedule(), dequeueColliderContacts.Schedule());

                // Sort particle contacts for jitter-free gauss-seidel (sequential) solving:
                dequeueHandle = ConstraintSorter.SortConstraints<BurstContact>(particleCount,rawParticleContacts, ref sortedParticleContacts, dequeueHandle);

                // batch particle contacts:
                var activeParticleBatchCount = new NativeArray<int>(1, Allocator.TempJob);
                var particleBatchHandle = constraintBatcher.BatchConstraints<BurstContact>(sortedParticleContacts, particleCount, ref particleContacts, ref particleBatchData, ref activeParticleBatchCount, dequeueHandle);

                // batch fluid interactions:
                var activeFluidBatchCount = new NativeArray<int>(1, Allocator.TempJob);
                var fluidBatchHandle = constraintBatcher.BatchConstraints<FluidInteraction>(rawFluidInteractions, particleCount, ref fluidInteractions, ref fluidBatchData, ref activeFluidBatchCount, dequeueHandle);

                JobHandle.CombineDependencies(particleBatchHandle, fluidBatchHandle).Complete();

                // Generate particle contact/friction batches:
                var pc = constraints[(int)Oni.ConstraintType.ParticleCollision] as BurstParticleCollisionConstraints;
                pc.batches.Clear();
                var pf = constraints[(int)Oni.ConstraintType.ParticleFriction] as BurstParticleFrictionConstraints;
                pf.batches.Clear();
                for (int i = 0; i < activeParticleBatchCount[0]; ++i)
                {
                    var cbatch = pc.CreateConstraintsBatch() as BurstParticleCollisionConstraintsBatch;
                    cbatch.batchData = particleBatchData[i];

                    var fbatch = pf.CreateConstraintsBatch() as BurstParticleFrictionConstraintsBatch;
                    fbatch.batchData = particleBatchData[i];
                }

                // Generate fluid interaction batches:
                var dc = constraints[(int)Oni.ConstraintType.Density] as BurstDensityConstraints;
                dc.batches.Clear();
                for (int i = 0; i < activeFluidBatchCount[0]; ++i)
                {
                    var batch = dc.CreateConstraintsBatch() as BurstDensityConstraintsBatch;
                    batch.batchData = fluidBatchData[i];
                }

                // dispose of temporary buffers:
                rawParticleContacts.Dispose();
                rawFluidInteractions.Dispose();
                sortedParticleContacts.Dispose();
                activeParticleBatchCount.Dispose();
                activeFluidBatchCount.Dispose();

            }

            return inputDeps;
        }

        public IObiJobHandle Substep(float substepTime)
        {
            // Apply aerodynamics
            JobHandle aerodynamicsHandle = constraints[(int)Oni.ConstraintType.Aerodynamics].Project(new JobHandle(), substepTime);

            // Predict positions:
            var predictPositions = new PredictPositionsJob()
            {
                activeParticles = activeParticles,
                phases = phases,
                buoyancies = buoyancies,

                externalForces = externalForces,
                inverseMasses = invMasses,
                positions = positions,
                previousPositions = prevPositions,
                velocities = velocities,

                externalTorques = externalTorques,
                inverseRotationalMasses = invRotationalMasses,
                orientations = orientations,
                previousOrientations = prevOrientations,
                angularVelocities = angularVelocities,

                gravity = new float4(m_Solver.parameters.gravity, 0),
                deltaTime = substepTime,
                is2D = abstraction.parameters.mode == Oni.SolverParameters.Mode.Mode2D
            };

            JobHandle predictPositionsHandle = predictPositions.Schedule(activeParticles.Length, 128, aerodynamicsHandle);

            // Project position constraints:
            JobHandle projectionHandle = ApplyConstraints(predictPositionsHandle, substepTime);

            // Update velocities:
            var updateVelocitiesJob = new UpdateVelocitiesJob()
            {
                activeParticles = activeParticles,

                inverseMasses = invMasses,
                previousPositions = prevPositions,
                positions = positions,
                velocities = velocities,

                inverseRotationalMasses = invRotationalMasses,
                previousOrientations = prevOrientations,
                orientations = orientations,
                angularVelocities = angularVelocities,

                deltaTime = substepTime,
                is2D = abstraction.parameters.mode == Oni.SolverParameters.Mode.Mode2D
            };

            JobHandle updateVelocitiesHandle = updateVelocitiesJob.Schedule(activeParticles.Length, 128, projectionHandle);

            // velocity constraints:
            JobHandle velocityCorrectionsHandle = ApplyVelocityCorrections(updateVelocitiesHandle, substepTime);

            // Update positions:
            var updatePositionsJob = new UpdatePositionsJob()
            {
                activeParticles = activeParticles,
                positions = positions,
                previousPositions = prevPositions,
                velocities = velocities,
                orientations = orientations,
                previousOrientations = prevOrientations,
                angularVelocities = angularVelocities,
                velocityScale = math.pow(1 - math.clamp(m_Solver.parameters.damping, 0, 1), substepTime),
                sleepThreshold = m_Solver.parameters.sleepThreshold
            };

            JobHandle updatePositionsHandle = updatePositionsJob.Schedule(activeParticles.Length, 128, velocityCorrectionsHandle);

            return new BurstJobHandle(updatePositionsHandle);
        }

        private JobHandle ApplyVelocityCorrections(JobHandle inputDeps, float deltaTime)
        {
            var densityParameters = m_Solver.GetConstraintParameters(Oni.ConstraintType.Density);

            if (densityParameters.enabled)
            {
                var d = constraints[(int)Oni.ConstraintType.Density] as BurstDensityConstraints;
                if (d != null)
                {
                    return d.ApplyVelocityCorrections(inputDeps, deltaTime);
                }
            }
            return inputDeps;
        }

        private JobHandle ApplyConstraints(JobHandle inputDeps, float deltaTime)
        {

            // calculate max amount of iterations required, and initialize constraints..
            int maxIterations = 0;
            for (int i = 0; i < Oni.ConstraintTypeCount; ++i)
            {
                var parameters = m_Solver.GetConstraintParameters((Oni.ConstraintType)i);
                if (parameters.enabled)
                {
                    maxIterations = math.max(maxIterations, parameters.iterations);
                    inputDeps = constraints[i].Initialize(inputDeps, deltaTime);
                }
            }

            // calculate iteration paddings:
            int[] padding = new int[Oni.ConstraintTypeCount];
            for (int i = 0; i < Oni.ConstraintTypeCount; ++i)
            {
                var parameters = m_Solver.GetConstraintParameters((Oni.ConstraintType)i);
                if (parameters.enabled && parameters.iterations > 0)
                    padding[i] = (int)math.ceil(maxIterations / (float)parameters.iterations);
                else
                    padding[i] = maxIterations;
            }

            // perform projection iterations:
            for (int i = 1; i < maxIterations; ++i)
            {
                for (int j = 0; j < Oni.ConstraintTypeCount; ++j)
                {
                    if (j != (int)Oni.ConstraintType.Aerodynamics)
                    {
                        var parameters = m_Solver.GetConstraintParameters((Oni.ConstraintType)j);
                        if (parameters.enabled && i % padding[j] == 0)
                            inputDeps = constraints[j].Project(inputDeps, deltaTime);
                    }
                }
            }

            // final iteration, all groups together:
            for (int i = 0; i < Oni.ConstraintTypeCount; ++i)
            {
                if (i != (int)Oni.ConstraintType.Aerodynamics)
                {
                    var parameters = m_Solver.GetConstraintParameters((Oni.ConstraintType)i);
                    if (parameters.enabled && parameters.iterations > 0)
                        inputDeps = constraints[i].Project(inputDeps, deltaTime);
                }
            }

            // Despite friction constraints being applied after collision (since coulomb friction depends on normal impulse)
            // we perform a collision iteration right at the end to ensure the final state meets the Signorini-Fichera conditions.
            var param = m_Solver.GetConstraintParameters(Oni.ConstraintType.ParticleCollision);
            if (param.enabled && param.iterations > 0)
                inputDeps = constraints[(int)Oni.ConstraintType.ParticleCollision].Project(inputDeps, deltaTime);
            param = m_Solver.GetConstraintParameters(Oni.ConstraintType.Collision);
            if (param.enabled && param.iterations > 0)
                inputDeps = constraints[(int)Oni.ConstraintType.Collision].Project(inputDeps, deltaTime);

            return inputDeps;
        }

        public void ApplyInterpolation(ObiNativeVector4List startPositions, ObiNativeQuaternionList startOrientations, float stepTime, float unsimulatedTime)
        {

            // Interpolate particle positions and orientations.
            var interpolate = new InterpolationJob()
            {
                positions = positions,
                startPositions = startPositions.AsNativeArray<float4>(),
                renderablePositions = renderablePositions,

                orientations = orientations,
                startOrientations = startOrientations.AsNativeArray<quaternion>(),
                renderableOrientations = renderableOrientations,

                deltaTime = stepTime,
                unsimulatedTime = unsimulatedTime,
                interpolationMode = m_Solver.parameters.interpolation
            };

            JobHandle jobHandle = interpolate.Schedule(m_Solver.positions.count, 128);

            // Update deformable triangle normals
            var updateNormals = new UpdateNormalsJob()
            {
                renderPositions = renderablePositions,
                deformableTriangles = deformableTriangles,
                normals = normals
            };

            jobHandle = updateNormals.Schedule(jobHandle);

            // fluid laplacian/anisotropy:
            var d = constraints[(int)Oni.ConstraintType.Density] as BurstDensityConstraints;
            if (d != null)
                jobHandle = d.CalculateAnisotropyLaplacianSmoothing(jobHandle);

            // update axis:
            var updatePrincipalAxis = new UpdatePrincipalAxisJob()
            {
                activeParticles = activeParticles,
                renderableOrientations = renderableOrientations,
                phases = phases,
                principalRadii = principalRadii,
                principalAxis = anisotropies,
            };

            jobHandle = updatePrincipalAxis.Schedule(activeParticles.Length, 128, jobHandle);

            jobHandle.Complete();

        }

        public void InterpolateDiffuseProperties(ObiNativeVector4List properties,
                                          ObiNativeVector4List diffusePositions,
                                          ObiNativeVector4List diffuseProperties,
                                          ObiNativeIntList neighbourCount,
                                          int diffuseCount)
        {
            particleGrid.InterpolateDiffuseProperties(this,
                                                      properties.AsNativeArray<float4>(),
                                                      diffusePositions.AsNativeArray<float4>(),
                                                      diffuseProperties.AsNativeArray<float4>(),
                                                      neighbourCount.AsNativeArray<int>(),
                                                      diffuseCount).Complete();
        }

        public int GetParticleGridSize()
        {
            return particleGrid.grid.usedCells.Length;
        }
        public void GetParticleGrid(ObiNativeAabbList cells)
        {
            particleGrid.GetCells(cells);
        }
    }
}
#endif