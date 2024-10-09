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
    public static class BurstSphere
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
            float4 center = shape.center * transform.scale;
            position = transform.InverseTransformPointUnscaled(position) - center;

            float radius = shape.size.x * math.cmax(transform.scale.xyz);
            float distanceToCenter = math.length(position);

            float4 normal = position / distanceToCenter;

            BurstContact c = new BurstContact
            {
                entityA = particleIndex,
                entityB = colliderIndex,
                point = center + normal * radius,
                normal = normal,
            };

            c.point = transform.TransformPointUnscaled(c.point);
            c.normal = transform.TransformDirection(c.normal);

            c.distance = distanceToCenter - radius - (shape.contactOffset + BurstMath.EllipsoidRadius(c.normal, orientation, radii.xyz));

            contacts.Enqueue(c);
        }
    }

}
#endif