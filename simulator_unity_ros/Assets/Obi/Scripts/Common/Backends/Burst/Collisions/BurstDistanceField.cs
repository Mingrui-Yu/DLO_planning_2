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
    public static class BurstDistanceField
    {
        public static void Contacts(int particleIndex,
                                    int colliderIndex,
                                    float4 position,
                                    quaternion orientation,
                                    float4 radii,
                                    ref NativeArray<BurstDFNode> dfNodes,
                                    DistanceFieldHeader header,
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

            float4 sample = DFTraverse(pos, 0, ref header, ref dfNodes);

            c.normal = new float4(math.normalize(sample.xyz),0);
            c.point = pos - c.normal * sample[3];

            c.normal = colliderToSolver.TransformDirection(c.normal);
            c.point = colliderToSolver.TransformPoint(c.point);

            c.distance = sample[3] * math.cmax(colliderToSolver.scale.xyz) - (shape.contactOffset + BurstMath.EllipsoidRadius(c.normal, orientation, radii.xyz));
            contacts.Enqueue(c);
        }

        private static float4 DFTraverse(float4 particlePosition,
                                         int nodeIndex,
                                         ref DistanceFieldHeader header,
                                         ref NativeArray<BurstDFNode> dfNodes)
        {
            var node = dfNodes[header.firstNode + nodeIndex];

            // if the child node exists, recurse down the df octree:
            if (node.firstChild >= 0)
            {
                int octant = node.GetOctant(particlePosition);
                return DFTraverse(particlePosition, node.firstChild + octant, ref header, ref dfNodes);

            }
            else
            {
                return node.SampleWithGradient(particlePosition);
            }
        }

    }

}
#endif