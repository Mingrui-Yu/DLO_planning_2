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
    public static class BurstEdgeMesh
    {
        public static void Contacts(int particleIndex,
                                    int colliderIndex,
                                    float4 particlePosition,
                                    quaternion particleOrientation,
                                    float4 particleVelocity,
                                    float4 particleRadii,
                                    float deltaTime,
                                    ref NativeArray<BIHNode> bihNodes,
                                    ref NativeArray<Edge> edges,
                                    ref NativeArray<float2> vertices,
                                    EdgeMeshHeader header,
                                    ref BurstAffineTransform colliderToSolver,
                                    ref BurstColliderShape shape,
                                    NativeQueue<BurstContact>.ParallelWriter contacts)
        {
            float4 colliderSpacePosition = colliderToSolver.InverseTransformPoint(particlePosition);
            float4 colliderSpaceVel = colliderToSolver.InverseTransformVector(particleVelocity * deltaTime);

            BurstAabb particleBounds = new BurstAabb(colliderSpacePosition,
                                                     colliderSpacePosition + colliderSpaceVel,
                                                     particleRadii.x / math.cmax(colliderToSolver.scale));

            colliderSpacePosition *= colliderToSolver.scale;

            BIHTraverse(particleIndex, colliderIndex,
                        colliderSpacePosition, particleOrientation, colliderSpaceVel, particleRadii, ref particleBounds,
                        0, ref bihNodes, ref edges, ref vertices, ref header, ref colliderToSolver, ref shape, contacts);
        }

        private static void BIHTraverse(int particleIndex,
                                        int colliderIndex,
                                        float4 particlePosition,
                                        quaternion particleOrientation,
                                        float4 particleVelocity,
                                        float4 particleRadii,
                                        ref BurstAabb particleBounds,
                                        int nodeIndex,
                                        ref NativeArray<BIHNode> bihNodes,
                                        ref NativeArray<Edge> edges,
                                        ref NativeArray<float2> vertices,
                                        ref EdgeMeshHeader header,
                                        ref BurstAffineTransform colliderToSolver,
                                        ref BurstColliderShape shape,
                                        NativeQueue<BurstContact>.ParallelWriter contacts)
        {
            var node = bihNodes[header.firstNode + nodeIndex];

            // amount by which we should inflate aabbs:
            float offset = shape.contactOffset + particleRadii.x;

            if (node.firstChild >= 0)
            {
                // visit min node:
                if (particleBounds.min[node.axis] - offset <= node.min)
                    BIHTraverse(particleIndex, colliderIndex,
                                particlePosition, particleOrientation, particleVelocity, particleRadii, ref particleBounds,
                                node.firstChild, ref bihNodes, ref edges, ref vertices, ref header,
                                ref colliderToSolver, ref shape, contacts);

                // visit max node:
                if (particleBounds.max[node.axis] + offset >= node.max)
                    BIHTraverse(particleIndex, colliderIndex,
                                particlePosition, particleOrientation, particleVelocity, particleRadii, ref particleBounds,
                                node.firstChild + 1, ref bihNodes, ref edges, ref vertices, ref header,
                                ref colliderToSolver, ref shape, contacts);
            }
            else
            {

                // precalculate inverse of velocity vector for ray/aabb intersections:
                float4 invDir = math.rcp(particleVelocity);

                // contacts against all triangles:
                for (int i = node.start; i < node.start + node.count; ++i)
                {
                    Edge t = edges[header.firstEdge + i];

                    float4 v1 = new float4(vertices[header.firstVertex + t.i1],0,0) * colliderToSolver.scale;
                    float4 v2 = new float4(vertices[header.firstVertex + t.i2],0,0) * colliderToSolver.scale;

                    BurstAabb aabb = new BurstAabb(v1, v2, 0.01f);
                    aabb.Expand(new float4(offset));

                    // only generate a contact if the particle trajectory intersects its inflated aabb:
                    if (aabb.IntersectsRay(particlePosition, invDir, true))
                    {

                        float4 point = BurstMath.NearestPointOnEdge(v1, v2, particlePosition);
                        float4 pointToTri = particlePosition - point;
                        float distance = math.length(pointToTri);

                        if (distance > BurstMath.epsilon)
                        {
                            BurstContact c = new BurstContact()
                            {
                                entityA = particleIndex,
                                entityB = colliderIndex,
                                point = colliderToSolver.TransformPointUnscaled(point),
                                normal = colliderToSolver.TransformDirection(pointToTri / distance),
                            };

                            c.distance = distance - (shape.contactOffset + BurstMath.EllipsoidRadius(c.normal, particleOrientation, particleRadii.xyz));

                            contacts.Enqueue(c);
                        }
                    }
                }
            }
        }

    }

}
#endif