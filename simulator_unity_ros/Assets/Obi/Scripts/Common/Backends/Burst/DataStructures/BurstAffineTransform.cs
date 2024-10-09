#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using UnityEngine;
using Unity.Mathematics;
using System.Collections;

namespace Obi
{
    public struct BurstAffineTransform
    {
        public float4 translation;
        public float4 scale;
        public quaternion rotation;

        public BurstAffineTransform(float4 translation, quaternion rotation, float4 scale)
        {
            // make sure there are good values in the 4th component:
            translation[3] = 0;
            scale[3] = 1;

            this.translation = translation;
            this.rotation = rotation;
            this.scale = scale;
        }

        public static BurstAffineTransform operator *(BurstAffineTransform a, BurstAffineTransform b)
        {
            return new BurstAffineTransform(a.TransformPoint(b.translation),
                                            math.mul(a.rotation,b.rotation),
                                            a.scale * b.scale);
        }

        public BurstAffineTransform Inverse()
        {
            return new BurstAffineTransform(new float4(math.rotate(math.conjugate(rotation),(translation / -scale).xyz),0),
                                            math.conjugate(rotation),
                                            1 / scale);
        }

        public BurstAffineTransform Interpolate(BurstAffineTransform other, float translationalMu, float rotationalMu, float scaleMu)
        {
            return new BurstAffineTransform(math.lerp(translation, other.translation, translationalMu),
                                            math.slerp(rotation, other.rotation, rotationalMu),
                                            math.lerp(scale, other.scale, scaleMu));
        }

        public float4 TransformPoint(float4 point)
        {
            return new float4(translation.xyz + math.rotate(rotation, (point * scale).xyz),0);
        }

        public float4 InverseTransformPoint(float4 point)
        {
            return new float4(math.rotate(math.conjugate(rotation),(point - translation).xyz) / scale.xyz , 0);
        }

        public float4 TransformPointUnscaled(float4 point)
        {
            return new float4(translation.xyz + math.rotate(rotation,point.xyz), 0);
        }

        public float4 InverseTransformPointUnscaled(float4 point)
        {
            return new float4(math.rotate(math.conjugate(rotation), (point - translation).xyz), 0);
        }


        public float4 TransformDirection(float4 direction)
        {
            return new float4(math.rotate(rotation, direction.xyz), 0);
        }

        public float4 InverseTransformDirection(float4 direction)
        {
            return new float4(math.rotate(math.conjugate(rotation), direction.xyz), 0);
        }

        public float4 TransformVector(float4 vector)
        {
            return new float4(math.rotate(rotation, (vector * scale).xyz), 0);
        }

        public float4 InverseTransformVector(float4 vector)
        {
            return new float4(math.rotate(math.conjugate(rotation),vector.xyz) / scale.xyz, 0);
        }
    }
}
#endif