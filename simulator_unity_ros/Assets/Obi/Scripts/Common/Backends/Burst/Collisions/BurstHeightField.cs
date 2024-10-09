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
    public static class BurstHeightField
    {
        public static void Contacts(int particleIndex,
                                    int colliderIndex,
                                    float4 position,
                                    quaternion orientation,
                                    float4 radii,
                                    ref NativeArray<float> heightMap,
                                    HeightFieldHeader header,
                                    BurstAffineTransform colliderToSolver,
                                    BurstColliderShape shape,
                                    NativeQueue<BurstContact>.ParallelWriter contacts)
        {
            float4 pos = colliderToSolver.InverseTransformPoint(position);

            BurstContact c = new BurstContact
            {
                entityA = particleIndex,
                entityB = colliderIndex,
            };

            int resolutionU = (int)shape.center.x;
            int resolutionV = (int)shape.center.y;

            // calculate terrain cell size:
            float cellWidth = shape.size.x / (resolutionU - 1);
            float cellHeight = shape.size.z / (resolutionV - 1);

            // calculate particle bounds min/max cells:
            int2 min = new int2((int)math.floor((pos[0] - radii[0]) / cellWidth), (int)math.floor((pos[2] - radii[0]) / cellHeight));
            int2 max = new int2((int)math.floor((pos[0] + radii[0]) / cellWidth), (int)math.floor((pos[2] + radii[0]) / cellHeight));

            for (int su = min[0]; su <= max[0]; ++su)
            {
                if (su >= 0 && su < resolutionU - 1)
                {
                    for (int sv = min[1]; sv <= max[1]; ++sv)
                    {
                        if (sv >= 0 && sv < resolutionV - 1)
                        {
                            // calculate neighbor sample indices:
                            int csu1 = math.clamp(su + 1, 0, resolutionU - 1);
                            int csv1 = math.clamp(sv + 1, 0, resolutionV - 1);

                            // sample heights:
                            float h1 = heightMap[header.firstSample + sv * resolutionU + su] * shape.size.y;
                            float h2 = heightMap[header.firstSample + sv * resolutionU + csu1] * shape.size.y;
                            float h3 = heightMap[header.firstSample + csv1 * resolutionU + su] * shape.size.y;
                            float h4 = heightMap[header.firstSample + csv1 * resolutionU + csu1] * shape.size.y;

                            float min_x = su * shape.size.x / (resolutionU - 1);
                            float max_x = csu1 * shape.size.x / (resolutionU - 1);
                            float min_z = sv * shape.size.z / (resolutionV - 1);
                            float max_z = csv1 * shape.size.z / (resolutionV - 1);

                            // contact with the first triangle:
                            float4 pointOnTri = BurstMath.NearestPointOnTri(new float4(min_x, h3, max_z, 0),
                                                                             new float4(max_x, h4, max_z, 0),
                                                                             new float4(min_x, h1, min_z, 0),
                                                                             pos);
                            float4 normal = pos - pointOnTri;
                            float distance = math.length(normal);

                            if (distance > BurstMath.epsilon)
                            {
                                c.normal = normal / distance;
                                c.point = pointOnTri;

                                c.normal = colliderToSolver.TransformDirection(c.normal);
                                c.point = colliderToSolver.TransformPoint(c.point);

                                c.distance = distance - (shape.contactOffset + BurstMath.EllipsoidRadius(c.normal, orientation, radii.xyz));
                                contacts.Enqueue(c);
                            }

                            // contact with the second triangle:
                            pointOnTri = BurstMath.NearestPointOnTri(new float4(min_x, h1, min_z, 0),
                                                                     new float4(max_x, h4, max_z, 0),
                                                                     new float4(max_x, h2, min_z, 0),
                                                                     pos);
                            normal = pos - pointOnTri;
                            distance = math.length(normal);

                            if (distance > BurstMath.epsilon)
                            {
                                c.normal = normal / distance;
                                c.point = pointOnTri;

                                c.normal = colliderToSolver.TransformDirection(c.normal);
                                c.point = colliderToSolver.TransformPoint(c.point);

                                c.distance = distance - (shape.contactOffset + BurstMath.EllipsoidRadius(c.normal, orientation, radii.xyz));
                                contacts.Enqueue(c);
                            }
                        }
                    }
                }
            }
        }

    }

}
#endif