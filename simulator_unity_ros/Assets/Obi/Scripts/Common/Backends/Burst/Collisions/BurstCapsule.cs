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
    public static class BurstCapsule
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

            position = transform.InverseTransformPointUnscaled(position) - center;

            int direction = (int)shape.size.z;
            float radius = shape.size.x * math.max(transform.scale[(direction + 1) % 3], transform.scale[(direction + 2) % 3]);
            float height = math.max(radius, shape.size.y * 0.5f * transform.scale[direction]);
            float d = position[direction];
            float4 axisProj = float4.zero;
            float4 cap = float4.zero;

            axisProj[direction] = d;
            cap[direction] = height - radius;

            float4 centerToPoint;
            float centerToPointNorm;

            if (d > height - radius)
            { //one cap

                centerToPoint = position - cap;
                centerToPointNorm = math.length(centerToPoint);

                c.distance = centerToPointNorm - radius;
                c.normal = (centerToPoint / (centerToPointNorm + math.FLT_MIN_NORMAL));
                c.point = cap + c.normal * radius;

            }
            else if (d < -height + radius)
            { // other cap

                centerToPoint = position + cap;
                centerToPointNorm = math.length(centerToPoint);

                c.distance = centerToPointNorm - radius;
                c.normal = (centerToPoint / (centerToPointNorm + math.FLT_MIN_NORMAL));
                c.point = -cap + c.normal * radius;

            }
            else
            {//cylinder

                centerToPoint = position - axisProj;
                centerToPointNorm = math.length(centerToPoint);

                c.distance = centerToPointNorm - radius;
                c.normal = (centerToPoint / (centerToPointNorm + math.FLT_MIN_NORMAL));
                c.point = axisProj + c.normal * radius;

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