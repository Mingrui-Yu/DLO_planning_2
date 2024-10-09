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
    public static class BurstBox
    {
        public static void Contacts(int particleIndex,
                                    float4 position,
                                    quaternion orientation,
                                    float4 radii,
                                    int colliderIndex,
                                    BurstAffineTransform transform,
                                    BurstColliderShape shape,
                                    NativeQueue<BurstContact>.ParallelWriter contacts)
        {
            BurstContact c = new BurstContact()
            {
                entityA = particleIndex,
                entityB = colliderIndex,
            };

            float4 center = shape.center * transform.scale;
            float4 size = shape.size * transform.scale * 0.5f;

            position = transform.InverseTransformPointUnscaled(position) - center;

            // Get minimum distance for each axis:
            float4 distances = size - math.abs(position);

            // if we are inside the box:
            if (distances.x >= 0 && distances.y >= 0 && distances.z >= 0)
            {
                // find minimum distance in all three axes and the axis index:
                float min = float.MaxValue;
                int axis = 0;
                for (int i = 0; i < 3; ++i)
                {
                    if (distances[i] < min)
                    {
                        min = distances[i];
                        axis = i;
                    }
                }

                c.normal = float4.zero;
                c.point = position;

                c.distance = -distances[axis];
                c.normal[axis] = position[axis] > 0 ? 1 : -1;
                c.point[axis] = size[axis] * c.normal[axis];
            }
            else // we are outside the box:
            {
                // clamp point to be inside the box:
                c.point = math.clamp(position, -size, size);

                // find distance and direction to clamped point:
                float4 diff = position - c.point;
                c.distance = math.length(diff);
                c.normal = diff / (c.distance + math.FLT_MIN_NORMAL);
            }

            c.point += center;
            c.point = transform.TransformPointUnscaled(c.point);
            c.normal = transform.TransformDirection(c.normal);

            c.distance -= shape.contactOffset + BurstMath.EllipsoidRadius(c.normal, orientation, radii.xyz);

            contacts.Enqueue(c);
        }

    }

}
#endif