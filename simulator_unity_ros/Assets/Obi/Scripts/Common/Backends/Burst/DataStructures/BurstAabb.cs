#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using UnityEngine;
using Unity.Mathematics;
using System.Collections;
using System;

namespace Obi
{
    public struct BurstAabb
    {
        public float4 min;
        public float4 max;

        public float4 size
        {
            get { return max - min; }
        }

        public BurstAabb(float4 min, float4 max)
        {
            this.min = min;
            this.max = max;
        }

        public BurstAabb(float4 v1, float4 v2, float4 v3, float margin)
        {
            min = math.min(math.min(v1, v2), v3) - new float4(margin, margin, margin, 0);
            max = math.max(math.max(v1, v2), v3) + new float4(margin, margin, margin, 0);
        }

        public BurstAabb(float2 v1, float2 v2, float margin)
        {
            min = new float4(math.min(v1, v2) - new float2(margin, margin),0,0);
            max = new float4(math.max(v1, v2) + new float2(margin, margin),0,0);
        }

        public BurstAabb(float4 previousPosition, float4 position, float radius)
        {
            min = math.min(position - radius, previousPosition - radius);
            max = math.max(position + radius, previousPosition + radius);
        }

        public float AverageAxisLength()
        {
            float4 d = max - min;
            return (d.x + d.y + d.z) * 0.33f;
        }

        public void EncapsulateParticle(float4 previousPosition, float4 position, float radius)
        {
            min = math.min(math.min(min, position - radius), previousPosition - radius);
            max = math.max(math.max(max, position + radius), previousPosition + radius);
        }

        public void EncapsulateBounds(BurstAabb bounds)
        {
            min = math.min(min,bounds.min);
            max = math.max(max,bounds.max);
        }

        public void Expand(float4 amount)
        {
            min -= amount;
            max += amount;
        }

        public void Transform(BurstAffineTransform transform)
        {
            float3x3 matrix = math.mul(new float3x3(transform.rotation),float3x3.Scale(transform.scale.xyz));

            float3 xa = matrix.c0 * min.x;
            float3 xb = matrix.c0 * max.x;

            float3 ya = matrix.c1 * min.y;
            float3 yb = matrix.c1 * max.y;

            float3 za = matrix.c2 * min.z;
            float3 zb = matrix.c2 * max.z;

            min = new float4(math.min(xa,xb) + math.min(ya,yb) + math.min(za,zb) + transform.translation.xyz, 0);
            max = new float4(math.max(xa,xb) + math.max(ya,yb) + math.max(za,zb) + transform.translation.xyz, 0);
        }

        public bool IntersectsRay(float4 origin, float4 inv_dir, bool in2D = false) 
        {
            float4 t1 = (min - origin) * inv_dir;
            float4 t2 = (max - origin) * inv_dir;

            float4 tmin1 = math.min(t1,t2);
            float4 tmax1 = math.max(t1,t2);

            float tmin, tmax;
        
            if (in2D) 
            {
                tmin = math.cmax(tmin1.xy);
                tmax = math.cmin(tmax1.xy);
            }
            else
            {
                tmin = math.cmax(tmin1.xyz);
                tmax = math.cmin(tmax1.xyz);
            }
        
            return tmax >= math.max(0, tmin) && tmin <= 1;
        }
    }
}
#endif