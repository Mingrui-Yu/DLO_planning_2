#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Burst;

namespace Obi
{
    public struct MovingEntity
    {
        public int4 oldCellCoord;
        public int4 newCellCoord;
        public int entity;
    }

    public class ParticleGrid : IDisposable
    {
        public NativeMultilevelGrid<int> grid;
        private NativeQueue<MovingEntity> movingParticles;
        public NativeQueue<BurstContact> particleContactQueue;
        public NativeQueue<FluidInteraction> fluidInteractionQueue;

        private NativeArray<int> previousActiveParticles;

        [BurstCompile]
        struct IdentifyMovingParticles : IJobParallelFor
        {

            [WriteOnly]
            [NativeDisableParallelForRestriction]
            public NativeQueue<MovingEntity>.ParallelWriter movingParticles;

            [ReadOnly] public NativeList<int> activeParticles;
            [ReadOnly] public NativeArray<float4> radii;
            [ReadOnly] public NativeArray<float> fluidRadii;
            [ReadOnly] public NativeArray<float4> positions;

            [ReadOnly] public NativeArray<int> particleMaterialIndices;
            [ReadOnly] public NativeArray<BurstCollisionMaterial> collisionMaterials;
            [ReadOnly] public bool is2D;

            [NativeDisableParallelForRestriction] public NativeArray<int4> cellCoord;

            public void Execute(int index)
            {
                int i = activeParticles[index];

                // Find this particle's stick distance:
                float stickDistance = 0;
                if (particleMaterialIndices[i] >= 0)
                    stickDistance = collisionMaterials[particleMaterialIndices[i]].stickDistance;

                // Use it (together with radius and fluid radius) to calculate its size in the grid.
                float size = radii[i].x * 2.2f + stickDistance;
                size = math.max(size, fluidRadii[i] * 1.1f);

                int level = NativeMultilevelGrid<int>.GridLevelForSize(size);
                float cellSize = NativeMultilevelGrid<int>.CellSizeOfLevel(level);

                // get new particle cell coordinate:
                int4 newCellCoord = new int4(GridHash.Quantize(positions[i].xyz, cellSize), level);

                // if the solver is 2D, project the particle to the z = 0 cell.
                if (is2D) newCellCoord[2] = 0;

                // if the current cell is different from the current one, the particle has changed cell. 
                if (!newCellCoord.Equals(cellCoord[i]))
                {
                    movingParticles.Enqueue(new MovingEntity()
                    {
                        oldCellCoord = cellCoord[i],
                        newCellCoord = newCellCoord,
                        entity = i
                    });
                    cellCoord[i] = newCellCoord;
                }
            }
        }

        [BurstCompile]
        public struct RemoveInactiveParticles : IJob
        {
            [ReadOnly] public NativeList<int> activeParticles;
            [ReadOnly] public NativeArray<int> previousActiveParticles;

            [NativeDisableParallelForRestriction] public NativeArray<int4> cellCoords;
            public NativeMultilevelGrid<int> grid;

            public void Execute()
            {
                int currentA = 0;
                int currentB = 0;

                int lastA = previousActiveParticles.Length;
                int lastB = activeParticles.Length;

                NativeList<int> inactive = new NativeList<int>(math.max(lastA, lastB), Allocator.Temp);

                // perform a set difference to find particles rendered inactive since last update:
                while (currentA != lastA && currentB != lastB)
                {
                    if (previousActiveParticles[currentA] < activeParticles[currentB])
                        inactive.Add(previousActiveParticles[currentA++]);
                    else if (activeParticles[currentB] < previousActiveParticles[currentA])
                        ++currentB;
                    else
                    {
                        ++currentA; ++currentB;
                    }
                }

                // copy remaining elements:
                for (int i = currentA; i < lastA; ++i)
                    inactive.Add(previousActiveParticles[i]);

                // remove these particles from their current cell:
                for (int i = 0; i < inactive.Length; ++i)
                {
                    int cellIndex;
                    if (grid.TryGetCellIndex(cellCoords[inactive[i]], out cellIndex))
                    {
                        var oldCell = grid.usedCells[cellIndex];
                        oldCell.Remove(inactive[i]);
                        grid.usedCells[cellIndex] = oldCell;

                        // set their current cell coord to an invalid value:
                        cellCoords[inactive[i]] = new int4(int.MaxValue);
                    }
                }
            }
        }

        [BurstCompile]
        struct UpdateMovingParticles : IJob
        {

            public NativeQueue<MovingEntity> movingParticles;
            public NativeMultilevelGrid<int> grid;

            public void Execute()
            {
                while (movingParticles.Count > 0)
                {
                    MovingEntity movingParticle = movingParticles.Dequeue();

                    // remove from old cell:
                    int cellIndex;
                    if (grid.TryGetCellIndex(movingParticle.oldCellCoord, out cellIndex))
                    {
                        var oldCell = grid.usedCells[cellIndex];
                        oldCell.Remove(movingParticle.entity);
                        grid.usedCells[cellIndex] = oldCell;
                    }

                    // add to new cell:
                    cellIndex = grid.GetOrCreateCell(movingParticle.newCellCoord);

                    var newCell = grid.usedCells[cellIndex];
                    newCell.Add(movingParticle.entity);
                    grid.usedCells[cellIndex] = newCell;
                }

                grid.RemoveEmpty();
            }
        }

        [BurstCompile]
        public struct GenerateParticleParticleContactsJob : IJobParallelFor
        {
            [ReadOnly] public NativeMultilevelGrid<int> grid;
            [DeallocateOnJobCompletion]
            [ReadOnly] public NativeArray<int> gridLevels;

            [ReadOnly] public NativeArray<float4> positions;
            [ReadOnly] public NativeArray<quaternion> orientations;
            [ReadOnly] public NativeArray<float4> restPositions;
            [ReadOnly] public NativeArray<float4> velocities;
            [ReadOnly] public NativeArray<float> invMasses;
            [ReadOnly] public NativeArray<float4> radii;
            [ReadOnly] public NativeArray<float> fluidRadii;
            [ReadOnly] public NativeArray<int> phases;

            [ReadOnly] public NativeArray<int> particleMaterialIndices;
            [ReadOnly] public NativeArray<BurstCollisionMaterial> collisionMaterials;

            [WriteOnly]
            [NativeDisableParallelForRestriction]
            public NativeQueue<BurstContact>.ParallelWriter contactsQueue;

            [WriteOnly]
            [NativeDisableParallelForRestriction]
            public NativeQueue<FluidInteraction>.ParallelWriter fluidInteractionsQueue;

            [ReadOnly] public float dt;

            public void Execute(int i)
            {
                // Looks for close particles in the same cell:
                IntraCellSearch(i);

                // Looks for close particles in neighboring cells, in the same level or higher levels.
                IntraLevelSearch(i);
            }

            private void IntraCellSearch(int cellIndex)
            {
                int cellLength = grid.usedCells[cellIndex].Length;

                for (int p = 0; p < cellLength; ++p)
                {
                    for (int n = p + 1; n < cellLength; ++n)
                    {
                        InteractionTest(grid.usedCells[cellIndex][p], grid.usedCells[cellIndex][n]);
                    }
                }
            }

            private void InterCellSearch(int cellIndex, int neighborCellIndex)
            {
                int cellLength = grid.usedCells[cellIndex].Length;
                int neighborCellLength = grid.usedCells[neighborCellIndex].Length;

                for (int p = 0; p < cellLength; ++p)
                {
                    for (int n = 0; n < neighborCellLength; ++n)
                    {
                        InteractionTest(grid.usedCells[cellIndex][p], grid.usedCells[neighborCellIndex][n]);
                    }
                }
            }

            private void IntraLevelSearch(int cellIndex)
            {
                int4 cellCoords = grid.usedCells[cellIndex].Coords;

                // neighboring cells in the current level:
                for (int i = 0; i < 13; ++i)
                {
                    int4 neighborCellCoords = new int4(cellCoords.xyz + GridHash.cellOffsets3D[i], cellCoords.w);

                    int neighborCellIndex;
                    if (grid.TryGetCellIndex(neighborCellCoords, out neighborCellIndex))
                    {
                        InterCellSearch(cellIndex, neighborCellIndex);
                    }
                }

                // neighboring cells in levels above the current one:
                int levelIndex = gridLevels.IndexOf<int,int>(cellCoords.w);
                if (levelIndex >= 0)
                {
                    levelIndex++;
                    for (; levelIndex < gridLevels.Length; ++levelIndex)
                    {
                        int level = gridLevels[levelIndex];

                        // calculate index of parent cell in parent level:
                        int4 parentCellCoords = NativeMultilevelGrid<int>.GetParentCellCoords(cellCoords, level);

                        // search in all neighbouring cells:
                        for (int x = -1; x <= 1; ++x)
                            for (int y = -1; y <= 1; ++y)
                                for (int z = -1; z <= 1; ++z)
                                {
                                    int4 neighborCellCoords = parentCellCoords + new int4(x, y, z, 0);

                                    int neighborCellIndex;
                                    if (grid.TryGetCellIndex(neighborCellCoords, out neighborCellIndex))
                                    {
                                        InterCellSearch(cellIndex, neighborCellIndex);
                                    }
                                }
                    }
                }
            }

            public void InteractionTest(int A, int B)
            {
                bool samePhase = (phases[A] & (int)Oni.ParticleFlags.GroupMask) == (phases[B] & (int)Oni.ParticleFlags.GroupMask);
                bool noSelfCollide = (phases[A] & (int)Oni.ParticleFlags.SelfCollide) == 0 || (phases[B] & (int)Oni.ParticleFlags.SelfCollide) == 0;

                // only particles of a different phase or set to self-collide can interact.
                if (samePhase && noSelfCollide)
                    return;

                // Predict positions at the end of the whole step:
                float4 predictedPositionA = positions[A] + velocities[A] * dt;
                float4 predictedPositionB = positions[B] + velocities[B] * dt;

                // Calculate particle center distance:
                float4 dab = predictedPositionA - predictedPositionB;
                float d2 = math.lengthsq(dab);

                // if both particles are fluid, check their smoothing radii:
                if ((phases[A] & (int)Oni.ParticleFlags.Fluid) != 0 &&
                    (phases[B] & (int)Oni.ParticleFlags.Fluid) != 0)
                {
                    float fluidDistance = math.max(fluidRadii[A], fluidRadii[B]);
                    if (d2 <= fluidDistance * fluidDistance)
                    {
                        fluidInteractionsQueue.Enqueue(new FluidInteraction { particleA = A, particleB = B });
                    }
                }
                else // at least one solid particle
                {
                    float solidDistance = radii[A].x + radii[B].x;

                    // if these particles are self-colliding (have same phase), see if they intersect at rest.
                    if (samePhase &&
                        restPositions[A].w > 0.5f &&
                        restPositions[B].w > 0.5f)
                    {
                        // if rest positions intersect, return too.
                        float sqr_rest_distance = math.lengthsq(restPositions[A] - restPositions[B]);
                        if (sqr_rest_distance < solidDistance * solidDistance)
                            return;
                    }

                    // calculate distance at which particles are able to interact:
                    int matIndexA = particleMaterialIndices[A];
                    int matIndexB = particleMaterialIndices[B];
                    float interactionDistance = solidDistance * 1.2f + (matIndexA >= 0 ? collisionMaterials[matIndexA].stickDistance : 0) +
                                                                       (matIndexB >= 0 ? collisionMaterials[matIndexB].stickDistance : 0);

                    // if the distance between their predicted positions is smaller than the interaction distance:
                    if (math.lengthsq(dab) <= interactionDistance * interactionDistance)
                    {
                        // calculate contact normal and distance:
                        float4 normal = positions[A] - positions[B];
                        float distance = math.length(normal);

                        if (distance > BurstMath.epsilon)
                        {
                            normal /= distance;

                            float rA = BurstMath.EllipsoidRadius(normal, orientations[A], radii[A].xyz);
                            float rB = BurstMath.EllipsoidRadius(normal, orientations[B], radii[B].xyz);

                            // adapt normal for one-sided particles:
                            if ((phases[A] & (int)Oni.ParticleFlags.OneSided) != 0 &&
                                (phases[B] & (int)Oni.ParticleFlags.OneSided) != 0)
                            {
                                float3 adjustment = float3.zero;
                                if (rA < rB)
                                    adjustment = math.mul(orientations[A], new float3(0, 0, -1));
                                else
                                    adjustment = math.mul(orientations[B], new float3(0, 0, 1));

                                float dot = math.dot(normal.xyz, adjustment);
                                if (dot < 0)
                                    normal -= 2 * dot * new float4(adjustment, 0);
                            }

                            contactsQueue.Enqueue(new BurstContact
                            {
                                entityA = A,
                                entityB = B,
                                point = positions[B] + normal * rB,
                                normal = normal,
                                distance = distance - (rA + rB)
                            });
                        }
                    }
                }
            }
        }

        [BurstCompile]
        public struct InterpolateDiffusePropertiesJob : IJobParallelFor
        {
            [ReadOnly] public NativeMultilevelGrid<int> grid;

            [DeallocateOnJobCompletion]
            [ReadOnly] public NativeArray<int4> cellOffsets;

            [ReadOnly] public NativeArray<float4> positions;
            [ReadOnly] public NativeArray<float4> properties;
            [ReadOnly] public NativeArray<float4> diffusePositions;
            [ReadOnly] public Poly6Kernel densityKernel;

            public NativeArray<float4> diffuseProperties;
            public NativeArray<int> neighbourCount;

            [DeallocateOnJobCompletion]
            [ReadOnly] public NativeArray<int> gridLevels;

            [ReadOnly] public BurstInertialFrame inertialFrame;
            [ReadOnly] public bool mode2D;

            public void Execute(int p)
            {
                neighbourCount[p] = 0;
                float4 diffuseProperty = float4.zero;
                float kernelSum = 0;

                int offsetCount = mode2D ? 4 : 8;

                float4 solverDiffusePosition = inertialFrame.frame.InverseTransformPoint(diffusePositions[p]);

                for (int k = 0; k < gridLevels.Length; ++k)
                {
                    int l = gridLevels[k];
                    float radius = NativeMultilevelGrid<int>.CellSizeOfLevel(l);

                    float4 cellCoords = math.floor(solverDiffusePosition / radius);

                    cellCoords[3] = 0;
                    if (mode2D)
                        cellCoords[2] = 0;

                    float4 posInCell = solverDiffusePosition - (cellCoords * radius + new float4(radius * 0.5f));
                    int4 quadrant = (int4)math.sign(posInCell);

                    quadrant[3] = l;

                    for (int i = 0; i < offsetCount; ++i)
                    {
                        int cellIndex;
                        if (grid.TryGetCellIndex((int4)cellCoords + cellOffsets[i] * quadrant, out cellIndex))
                        {
                            var cell = grid.usedCells[cellIndex];
                            for (int n = 0; n < cell.Length; ++n)
                            {
                                float4 r = solverDiffusePosition - positions[cell[n]];
                                r[3] = 0;
                                if (mode2D)
                                    r[2] = 0;

                                float d = math.length(r);
                                if (d <= radius)
                                {
                                    float w = densityKernel.W(d, radius);
                                    kernelSum += w;
                                    diffuseProperty += properties[cell[n]] * w;
                                    neighbourCount[p]++;
                                }
                            }
                        }
                    }
                }

                if (kernelSum > BurstMath.epsilon)
                    diffuseProperties[p] = diffuseProperty / kernelSum;
            }
        }

        public ParticleGrid()
        {
            this.grid = new NativeMultilevelGrid<int>(1000, Allocator.Persistent);
            this.movingParticles = new NativeQueue<MovingEntity>(Allocator.Persistent);
            this.particleContactQueue = new NativeQueue<BurstContact>(Allocator.Persistent);
            this.fluidInteractionQueue = new NativeQueue<FluidInteraction>(Allocator.Persistent);
            this.previousActiveParticles = new NativeArray<int>(0,Allocator.Persistent);
        }

        public void UpdateGrid(BurstSolverImpl solver, JobHandle inputDeps)
        {
            var identifyMoving = new IdentifyMovingParticles
            {
                activeParticles = solver.activeParticles,
                movingParticles = movingParticles.AsParallelWriter(),
                radii = solver.principalRadii,
                fluidRadii = solver.smoothingRadii,
                positions = solver.positions,
                cellCoord = solver.cellCoords,
                particleMaterialIndices = solver.abstraction.collisionMaterials.AsNativeArray<int>(),
                collisionMaterials = ObiColliderWorld.GetInstance().collisionMaterials.AsNativeArray<BurstCollisionMaterial>(),
                is2D = solver.abstraction.parameters.mode == Oni.SolverParameters.Mode.Mode2D
            };
            inputDeps = identifyMoving.Schedule(solver.activeParticleCount, 64, inputDeps);

            var removeInactive = new RemoveInactiveParticles
            {
                activeParticles = solver.activeParticles,
                previousActiveParticles = previousActiveParticles,
                cellCoords = solver.cellCoords,
                grid = grid
            };
            inputDeps = removeInactive.Schedule(inputDeps);

            var updateMoving = new UpdateMovingParticles
            {
                movingParticles = movingParticles,
                grid = grid
            };
            updateMoving.Schedule(inputDeps).Complete();

            if (previousActiveParticles.IsCreated)
                previousActiveParticles.Dispose();

            previousActiveParticles = solver.activeParticles.ToArray(Allocator.Persistent);
        }

        public JobHandle GenerateContacts(BurstSolverImpl solver, float deltaTime)
        {

            var generateParticleContactsJob = new ParticleGrid.GenerateParticleParticleContactsJob
            {
                grid = grid,
                gridLevels = grid.populatedLevels.GetKeyArray(Allocator.TempJob),

                positions = solver.positions,
                orientations = solver.orientations,
                restPositions = solver.restPositions,
                velocities = solver.velocities,
                invMasses = solver.invMasses,
                radii = solver.principalRadii,
                fluidRadii = solver.smoothingRadii,
                phases = solver.phases,

                particleMaterialIndices = solver.abstraction.collisionMaterials.AsNativeArray<int>(),
                collisionMaterials = ObiColliderWorld.GetInstance().collisionMaterials.AsNativeArray<BurstCollisionMaterial>(),

                contactsQueue = particleContactQueue.AsParallelWriter(),
                fluidInteractionsQueue = fluidInteractionQueue.AsParallelWriter(),
                dt = deltaTime
            };

            return generateParticleContactsJob.Schedule(grid.CellCount, 2);

        }

        public JobHandle InterpolateDiffuseProperties(BurstSolverImpl solver,
                                                        NativeArray<float4> properties,
                                                        NativeArray<float4> diffusePositions,
                                                        NativeArray<float4> diffuseProperties,
                                                        NativeArray<int> neighbourCount,
                                                        int diffuseCount)
        {

            NativeArray<int4> offsets = new NativeArray<int4>(8, Allocator.TempJob);
            offsets[0] = new int4(0, 0, 0, 1);
            offsets[1] = new int4(1, 0, 0, 1);
            offsets[2] = new int4(0, 1, 0, 1);
            offsets[3] = new int4(1, 1, 0, 1);
            offsets[4] = new int4(0, 0, 1, 1);
            offsets[5] = new int4(1, 0, 1, 1);
            offsets[6] = new int4(0, 1, 1, 1);
            offsets[7] = new int4(1, 1, 1, 1);

            var interpolateDiffusePropertiesJob = new InterpolateDiffusePropertiesJob
            {
                grid = grid,
                positions = solver.abstraction.positions.AsNativeArray<float4>(),
                cellOffsets = offsets,
                properties = properties,
                diffusePositions = diffusePositions,
                diffuseProperties = diffuseProperties,
                neighbourCount = neighbourCount,
                densityKernel = new Poly6Kernel(solver.abstraction.parameters.mode == Oni.SolverParameters.Mode.Mode2D),
                gridLevels = grid.populatedLevels.GetKeyArray(Allocator.TempJob),
                inertialFrame = solver.inertialFrame,
                mode2D = solver.abstraction.parameters.mode == Oni.SolverParameters.Mode.Mode2D
            };

            return interpolateDiffusePropertiesJob.Schedule(diffuseCount, 64);
        }

        public void GetCells(ObiNativeAabbList cells)
        {
            if (cells.count == grid.usedCells.Length)
            {
                for (int i = 0; i < grid.usedCells.Length; ++i)
                {
                    var cell = grid.usedCells[i];
                    float size = NativeMultilevelGrid<int>.CellSizeOfLevel(cell.Coords.w);

                    float4 min = (float4)cell.Coords * size;
                    min[3] = 0;

                    cells[i] = new Aabb(min, min + new float4(size, size, size, 0));
                }
            }
        }

        public void Dispose()
        {
            grid.Dispose();
            movingParticles.Dispose();
            particleContactQueue.Dispose();
            fluidInteractionQueue.Dispose();
            previousActiveParticles.Dispose();
        }

    }
}
#endif