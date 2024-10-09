#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using UnityEngine;
using Unity.Mathematics;
using System.Collections;

namespace Obi
{
    public struct BurstInertialFrame
    {
        public BurstAffineTransform frame;
        public BurstAffineTransform prevFrame;

        public float4 velocity;
        public float4 angularVelocity;

        public float4 acceleration;
        public float4 angularAcceleration;

        public BurstInertialFrame(float4 position, float4 scale, quaternion rotation)
        {
            this.frame = new BurstAffineTransform(position, rotation, scale);
            this.prevFrame = frame;

            velocity = float4.zero;
            angularVelocity = float4.zero;
            acceleration = float4.zero;
            angularAcceleration = float4.zero;
        }

        public BurstInertialFrame(BurstAffineTransform frame)
        {
            this.frame = frame;
            this.prevFrame = frame;

            velocity = float4.zero;
            angularVelocity = float4.zero;
            acceleration = float4.zero;
            angularAcceleration = float4.zero;
        }

        public void Update(float4 position, float4 scale, quaternion rotation, float dt)
        {
            prevFrame = frame;
            float4 prevVelocity = velocity;
            float4 prevAngularVelocity = angularVelocity;

            frame.translation = position;
            frame.rotation = rotation;
            frame.scale = scale;

            velocity = BurstIntegration.DifferentiateLinear(frame.translation, prevFrame.translation, dt);
            angularVelocity = BurstIntegration.DifferentiateAngular(frame.rotation, prevFrame.rotation, dt);

            acceleration = BurstIntegration.DifferentiateLinear(velocity, prevVelocity, dt);
            angularAcceleration = BurstIntegration.DifferentiateLinear(angularVelocity, prevAngularVelocity, dt);
        }
       
    }
}
#endif