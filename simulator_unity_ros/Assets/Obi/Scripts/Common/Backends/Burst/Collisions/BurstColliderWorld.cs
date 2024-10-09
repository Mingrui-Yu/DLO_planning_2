#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Burst;

namespace Obi
{


    public class BurstColliderWorld : MonoBehaviour, IColliderWorldImpl
    {
        struct MovingCollider
        {
            public BurstCellSpan oldSpan;
            public BurstCellSpan newSpan;
            public int entity;
        }

        private int refCount = 0;
        private int colliderCount = 0;

        private NativeMultilevelGrid<int> grid;
        private NativeQueue<MovingCollider> movingColliders;
        public NativeQueue<BurstContact> colliderContactQueue;

        public ObiNativeCellSpanList cellSpans;

        public void Awake()
        {

            this.grid = new NativeMultilevelGrid<int>(1000, Allocator.Persistent);
            this.movingColliders = new NativeQueue<MovingCollider>(Allocator.Persistent);
            this.colliderContactQueue = new NativeQueue<BurstContact>(Allocator.Persistent);

            this.cellSpans = new ObiNativeCellSpanList();

            ObiColliderWorld.GetInstance().RegisterImplementation(this);
        }

        public void OnDestroy()
        {
            ObiColliderWorld.GetInstance().UnregisterImplementation(this);

            grid.Dispose();
            movingColliders.Dispose();
            colliderContactQueue.Dispose();
            cellSpans.Dispose();
        }

        public void IncreaseReferenceCount()
        {
            refCount++;
        }
        public void DecreaseReferenceCount()
        {
            if (--refCount <= 0 && gameObject != null)
                DestroyImmediate(gameObject);  
        }

        public void SetColliders(ObiNativeColliderShapeList shapes, ObiNativeAabbList bounds, ObiNativeAffineTransformList transforms, int count)
        {
            colliderCount = count;

            // insert new empty cellspans at the end if needed:
            while (colliderCount > cellSpans.count)
                cellSpans.Add(new CellSpan(new VInt4(10000), new VInt4(10000)));
        }

        public void SetRigidbodies(ObiNativeRigidbodyList rigidbody)
        {
        }

        public void SetCollisionMaterials(ObiNativeCollisionMaterialList materials)
        {

        }

        public void SetTriangleMeshData(ObiNativeTriangleMeshHeaderList headers, ObiNativeBIHNodeList nodes, ObiNativeTriangleList triangles, ObiNativeVector3List vertices)
        {
        }

        public void SetEdgeMeshData(ObiNativeEdgeMeshHeaderList headers, ObiNativeBIHNodeList nodes, ObiNativeEdgeList edges, ObiNativeVector2List vertices)
        {
        }

        public void SetDistanceFieldData(ObiNativeDistanceFieldHeaderList headers, ObiNativeDFNodeList nodes) { }
        public void SetHeightFieldData(ObiNativeHeightFieldHeaderList headers, ObiNativeFloatList samples) { }

        public void UpdateWorld()
        {
            var identifyMoving = new IdentifyMovingColliders
            {
                movingColliders = movingColliders.AsParallelWriter(),
                colliders = ObiColliderWorld.GetInstance().colliderShapes.AsNativeArray<BurstColliderShape>(cellSpans.count),
                bounds = ObiColliderWorld.GetInstance().colliderAabbs.AsNativeArray<BurstAabb>(cellSpans.count),
                cellIndices = cellSpans.AsNativeArray<BurstCellSpan>(),
                colliderCount = colliderCount
            };
            JobHandle movingHandle = identifyMoving.Schedule(cellSpans.count, 128);

            var updateMoving = new UpdateMovingColliders
            {
                movingColliders = movingColliders,
                grid = grid,
                colliderCount = colliderCount
            };

            updateMoving.Schedule(movingHandle).Complete();

            // remove tail from the current spans array:
            if (colliderCount < cellSpans.count)
                cellSpans.count -= cellSpans.count - colliderCount;
        }

        [BurstCompile]
        struct IdentifyMovingColliders : IJobParallelFor
        {
            [WriteOnly]
            [NativeDisableParallelForRestriction]
            public NativeQueue<MovingCollider>.ParallelWriter movingColliders;

            [ReadOnly] public NativeArray<BurstColliderShape> colliders;
            [ReadOnly] public NativeArray<BurstAabb> bounds;
            public NativeArray<BurstCellSpan> cellIndices;

            [ReadOnly] public int colliderCount;

            // Iterate over all colliders and store those whose cell span has changed.
            public void Execute(int i)
            {
                float size = bounds[i].AverageAxisLength();
                int level = NativeMultilevelGrid<int>.GridLevelForSize(size);
                float cellSize = NativeMultilevelGrid<int>.CellSizeOfLevel(level);

                // get new collider bounds cell coordinates:
                BurstCellSpan newSpan = new BurstCellSpan(new int4(GridHash.Quantize(bounds[i].min.xyz, cellSize), level),
                                                          new int4(GridHash.Quantize(bounds[i].max.xyz, cellSize), level));

                // if the collider is 2D, project it to the z = 0 cells.
                if (colliders[i].is2D != 0)
                {
                    newSpan.min[2] = 0;
                    newSpan.max[2] = 0;
                }

                // if the collider is at the tail (removed), we will only remove it from its current cellspan.
                // if the new cellspan and the current one are different, we must remove it from its current cellspan and add it to its new one.
                if (i >= colliderCount || cellIndices[i] != newSpan)
                {
                    // Add the collider to the list of moving colliders:
                    movingColliders.Enqueue(new MovingCollider()
                    {
                        oldSpan = cellIndices[i],
                        newSpan = newSpan,
                        entity = i
                    });

                    // Update previous coords:
                    cellIndices[i] = newSpan;
                }

            }
        }

        [BurstCompile]
        struct UpdateMovingColliders : IJob
        {
            public NativeQueue<MovingCollider> movingColliders;
            public NativeMultilevelGrid<int> grid;
            [ReadOnly] public int colliderCount;

            public void Execute()
            {
                while (movingColliders.Count > 0)
                {
                    MovingCollider movingCollider = movingColliders.Dequeue();

                    // remove from old cells:
                    grid.RemoveFromCells(movingCollider.oldSpan, movingCollider.entity);

                    // insert in new cells, as long as the index is below the amount of colliders.
                    // otherwise, the collider is at the "tail" and there's no need to add it back.
                    if (movingCollider.entity < colliderCount)
                        grid.AddToCells(movingCollider.newSpan, movingCollider.entity);
                }

                // remove all empty cells from the grid:
                grid.RemoveEmpty();
            }
        }

        [BurstCompile]
        unsafe struct GenerateContactsJob : IJobParallelFor
        {
            // particle and collider grids:
            [ReadOnly] public NativeMultilevelGrid<int> particleGrid;
            [ReadOnly] public NativeMultilevelGrid<int> colliderGrid;

            [DeallocateOnJobCompletion]
            [ReadOnly] public NativeArray<int> gridLevels;

            // particle arrays:
            [ReadOnly] public NativeArray<float4> velocities;
            [ReadOnly] public NativeArray<float4> positions;
            [ReadOnly] public NativeArray<quaternion> orientations;
            [ReadOnly] public NativeArray<float> invMasses;
            [ReadOnly] public NativeArray<float4> radii;
            [ReadOnly] public NativeArray<int> phases;
            [ReadOnly] public NativeArray<int> particleMaterialIndices;

            // collider arrays:
            [ReadOnly] public NativeArray<BurstAffineTransform> transforms;
            [ReadOnly] public NativeArray<BurstColliderShape> shapes;
            [ReadOnly] public NativeArray<BurstAabb> bounds;
            [ReadOnly] public NativeArray<BurstCollisionMaterial> collisionMaterials;

            // triangle mesh arrays:
            [ReadOnly] public NativeArray<TriangleMeshHeader> triangleMeshHeaders;
            [ReadOnly] public NativeArray<BIHNode> bihNodes;
            [ReadOnly] public NativeArray<Triangle> triangles;
            [ReadOnly] public NativeArray<float3> vertices;

            // edge mesh arrays:
            [ReadOnly] public NativeArray<EdgeMeshHeader> edgeMeshHeaders;
            [ReadOnly] public NativeArray<BIHNode> edgeBihNodes;
            [ReadOnly] public NativeArray<Edge> edges;
            [ReadOnly] public NativeArray<float2> edgeVertices;

            // distance field arrays:
            [ReadOnly] public NativeArray<DistanceFieldHeader> distanceFieldHeaders;
            [ReadOnly] public NativeArray<BurstDFNode> dfNodes;

            // height field arrays:
            [ReadOnly] public NativeArray<HeightFieldHeader> heightFieldHeaders;
            [ReadOnly] public NativeArray<float> heightFieldSamples;

            // output contacts queue:
            [WriteOnly]
            [NativeDisableParallelForRestriction]
            public NativeQueue<BurstContact>.ParallelWriter contactsQueue;

            // auxiliar data:
            [ReadOnly] public BurstAffineTransform solverToWorld;
            [ReadOnly] public BurstAffineTransform worldToSolver;
            [ReadOnly] public bool is2D;
            [ReadOnly] public float deltaTime;

            public void Execute(int i)
            {
                var cell = particleGrid.usedCells[i];

                BurstAabb cellBounds = new BurstAabb(float.MaxValue, float.MinValue);

                // here we calculate cell bounds that enclose both the predicted position and the original position of all its particles,
                // for accurate continuous collision detection.
                for (int p = 0; p < cell.Length; ++p)
                {
                    int pIndex = cell[p];

                    // get collision material stick distance:
                    float stickDistance = 0;
                    int materialIndex = particleMaterialIndices[pIndex];
                    if (materialIndex >= 0)
                        stickDistance = collisionMaterials[materialIndex].stickDistance;

                    cellBounds.EncapsulateParticle(positions[pIndex],
                                                   positions[pIndex] + velocities[pIndex] * deltaTime,
                                                   radii[pIndex].x + stickDistance);
                }

                // transform the cell bounds to world space:
                cellBounds.Transform(solverToWorld);

                // get all colliders overlapped by the cell bounds, in all grid levels:
                NativeList<int> candidates = new NativeList<int>(Allocator.Temp);
                for (int l = 0; l < gridLevels.Length; ++l)
                    GetCandidatesForBoundsAtLevel(candidates, cellBounds, gridLevels[l], is2D);

                if (candidates.Length > 0)
                {
                    // make sure each candidate collider only shows up once in the array:
                    NativeArray<int> uniqueCandidates = candidates.AsArray();
                    uniqueCandidates.Sort();
                    int uniqueCount = uniqueCandidates.Unique();

                    // iterate over candidate colliders, generating contacts for each one
                    for (int c = 0; c < uniqueCount; ++c)
                    {
                        int colliderIndex = uniqueCandidates[c];
                        BurstColliderShape shape = shapes[colliderIndex];
                        BurstAabb colliderBounds = bounds[colliderIndex];
                        BurstAffineTransform colliderToSolver = worldToSolver * transforms[colliderIndex];

                        // transform collider bounds to solver space:
                        colliderBounds.Transform(worldToSolver);

                        // iterate over all particles in the cell:
                        for (int p = 0; p < cell.Length; ++p)
                        {
                            int particleIndex = cell[p];

                            // skip this pair if particle and collider have the same phase:
                            if (shape.phase == (phases[particleIndex] & (int)Oni.ParticleFlags.GroupMask))
                                continue;

                            // get collision material stick distance:
                            float stickDistance = 0;
                            int materialIndex = particleMaterialIndices[particleIndex];
                            if (materialIndex >= 0)
                                stickDistance = collisionMaterials[materialIndex].stickDistance;

                            // inflate collider bounds by particle's bounds:
                            BurstAabb inflatedColliderBounds = colliderBounds;
                            inflatedColliderBounds.Expand(radii[particleIndex].x * 1.2f + stickDistance);

                            float4 invDir = math.rcp(velocities[particleIndex] * deltaTime);

                            // We check particle trajectory ray vs inflated collider aabb
                            // instead of checking particle vs collider aabbs directly, as this reduces
                            // the amount of contacts generated for fast moving particles.
                            if (inflatedColliderBounds.IntersectsRay(positions[particleIndex], invDir, shape.is2D != 0))
                            {
                                // generate contacts for the collider:
                                GenerateContacts(shape.type,
                                                 particleIndex, colliderIndex,
                                                 positions[particleIndex], orientations[particleIndex], velocities[particleIndex], radii[particleIndex],
                                                 colliderToSolver, shape, contactsQueue, deltaTime);
                            }
                        }

                    }
                }
            }

            private void GetCandidatesForBoundsAtLevel(NativeList<int> candidates, BurstAabb cellBounds, int level, bool is2D = false, int maxSize = 10)
            {
                float cellSize = NativeMultilevelGrid<int>.CellSizeOfLevel(level);

                int3 minCell = GridHash.Quantize(cellBounds.min.xyz, cellSize);
                int3 maxCell = GridHash.Quantize(cellBounds.max.xyz, cellSize);
                maxCell = minCell + math.min(maxCell - minCell, new int3(maxSize));

                int3 size = maxCell - minCell + new int3(1);
                int cellIndex;

                for (int x = minCell[0]; x <= maxCell[0]; ++x)
                {
                    for (int y = minCell[1]; y <= maxCell[1]; ++y)
                    {
                        // for 2D mode, project each cell at z == 0 and check them too. This way we ensure 2D colliders
                        // (which are inserted in cells with z == 0) are accounted for in the broadphase.
                        if (is2D)
                        {

                            if (colliderGrid.TryGetCellIndex(new int4(x, y, 0, level), out cellIndex))
                            {
                                var colliderCell = colliderGrid.usedCells[cellIndex];
                                candidates.AddRange(colliderCell.ContentsPointer, colliderCell.Length);
                            }
                        }

                        for (int z = minCell[2]; z <= maxCell[2]; ++z)
                        {
                            if (colliderGrid.TryGetCellIndex(new int4(x, y, z, level), out cellIndex))
                            {
                                var colliderCell = colliderGrid.usedCells[cellIndex];
                                candidates.AddRange(colliderCell.ContentsPointer, colliderCell.Length);
                            }
                        }
                    }
                }
            }

            private void GenerateContacts(ColliderShape.ShapeType colliderType,
                                          int particleIndex,
                                          int colliderIndex,
                                          float4 particlePosition,
                                          quaternion particleOrientation,
                                          float4 particleVelocity,
                                          float4 particleRadii,
                                          BurstAffineTransform colliderToSolver,
                                          BurstColliderShape shape,
                                          NativeQueue<BurstContact>.ParallelWriter contacts,
                                          float dt)
            {
                switch (colliderType)
                {
                    case ColliderShape.ShapeType.Sphere:
                        BurstSphere.Contacts(particleIndex, particlePosition, particleOrientation, particleRadii,
                                             colliderIndex, colliderToSolver, shape,
                                             contacts);
                        break;
                    case ColliderShape.ShapeType.Box:
                        BurstBox.Contacts(particleIndex, particlePosition, particleOrientation, particleRadii,
                                          colliderIndex, colliderToSolver, shape,
                                          contacts);
                        break;
                    case ColliderShape.ShapeType.Capsule:
                        BurstCapsule.Contacts(particleIndex, particlePosition, particleOrientation, particleRadii,
                                              colliderIndex, colliderToSolver, shape,
                                              contacts);
                        break;
                    case ColliderShape.ShapeType.SignedDistanceField:
                        BurstDistanceField.Contacts(particleIndex, colliderIndex,
                                                   particlePosition, particleOrientation, particleRadii,
                                                   ref dfNodes, distanceFieldHeaders[shape.dataIndex], colliderToSolver, shape, contacts);
                        break;
                    case ColliderShape.ShapeType.Heightmap:
                        BurstHeightField.Contacts(particleIndex, colliderIndex,
                                                  particlePosition, particleOrientation, particleRadii,
                                                  ref heightFieldSamples, heightFieldHeaders[shape.dataIndex], colliderToSolver, shape, contacts);
                        break;
                    case ColliderShape.ShapeType.TriangleMesh:
                        BurstTriangleMesh.Contacts(particleIndex, colliderIndex,
                                                   particlePosition, particleOrientation, particleVelocity, particleRadii, dt,
                                                   ref bihNodes, ref triangles, ref vertices, triangleMeshHeaders[shape.dataIndex],
                                                   ref colliderToSolver, ref shape, contacts);
                        break;
                    case ColliderShape.ShapeType.EdgeMesh:
                        BurstEdgeMesh.Contacts(particleIndex, colliderIndex,
                                                   particlePosition, particleOrientation, particleVelocity, particleRadii, dt,
                                                   ref edgeBihNodes, ref edges, ref edgeVertices, edgeMeshHeaders[shape.dataIndex],
                                                   ref colliderToSolver, ref shape, contacts);
                        break;
                }
            }

        }


        public JobHandle GenerateContacts(BurstSolverImpl solver, float deltaTime)
        {
            var world = ObiColliderWorld.GetInstance();

            var generateColliderContactsJob = new GenerateContactsJob
            {
                particleGrid = solver.particleGrid.grid,
                colliderGrid = grid,
                gridLevels = grid.populatedLevels.GetKeyArray(Allocator.TempJob),
                positions = solver.positions,
                orientations = solver.orientations,
                velocities = solver.velocities,
                invMasses = solver.invMasses,
                radii = solver.principalRadii,
                phases = solver.phases,
                particleMaterialIndices = solver.collisionMaterials,

                transforms = world.colliderTransforms.AsNativeArray<BurstAffineTransform>(),
                shapes = world.colliderShapes.AsNativeArray<BurstColliderShape>(),
                bounds = world.colliderAabbs.AsNativeArray<BurstAabb>(),
                collisionMaterials = world.collisionMaterials.AsNativeArray<BurstCollisionMaterial>(),

                triangleMeshHeaders = world.triangleMeshContainer.headers.AsNativeArray<TriangleMeshHeader>(),
                bihNodes = world.triangleMeshContainer.bihNodes.AsNativeArray<BIHNode>(),
                triangles = world.triangleMeshContainer.triangles.AsNativeArray<Triangle>(),
                vertices = world.triangleMeshContainer.vertices.AsNativeArray<float3>(),

                edgeMeshHeaders = world.edgeMeshContainer.headers.AsNativeArray<EdgeMeshHeader>(),
                edgeBihNodes = world.edgeMeshContainer.bihNodes.AsNativeArray<BIHNode>(),
                edges = world.edgeMeshContainer.edges.AsNativeArray<Edge>(),
                edgeVertices = world.edgeMeshContainer.vertices.AsNativeArray<float2>(),

                distanceFieldHeaders = world.distanceFieldContainer.headers.AsNativeArray<DistanceFieldHeader>(),
                dfNodes = world.distanceFieldContainer.dfNodes.AsNativeArray<BurstDFNode>(),

                heightFieldHeaders = world.heightFieldContainer.headers.AsNativeArray<HeightFieldHeader>(),
                heightFieldSamples = world.heightFieldContainer.samples.AsNativeArray<float>(),

                contactsQueue = colliderContactQueue.AsParallelWriter(),
                solverToWorld = solver.solverToWorld,
                worldToSolver = solver.worldToSolver,
                is2D = solver.abstraction.parameters.mode == Oni.SolverParameters.Mode.Mode2D,
                deltaTime = deltaTime
            };

            return generateColliderContactsJob.Schedule(solver.particleGrid.grid.CellCount, 16);

        }

    }
}
#endif