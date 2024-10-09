#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;
using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;

namespace Obi
{

    public struct BurstRigidbody
    {
        public float4x4 inverseInertiaTensor;
        public float4 velocity;
        public float4 angularVelocity;
        public float4 com;
        public float inverseMass;

        private int pad0;
        private int pad1;
        private int pad2;

        public float4 GetVelocityAtPoint(float4 point, float4 linearDelta, float4 angularDelta)
        {
            return velocity + linearDelta + new float4(math.cross(angularVelocity.xyz + angularDelta.xyz, (point - com).xyz),0);
        }

        public void ApplyImpulse(float4 impulse, float4 point, ref float4 linearDelta, ref float4 angularDelta)
        {
            linearDelta += inverseMass * impulse;
            angularDelta += math.mul(inverseInertiaTensor,new float4(math.cross((point - com).xyz,impulse.xyz),0));
        }

        public void ApplyDeltaQuaternion(quaternion rotation, quaternion delta, ref float4 angularDelta, float dt)
        {
            // convert quaternion delta to angular acceleration:
            quaternion newRotation = math.normalize(new quaternion(rotation.value + delta.value));
            angularDelta += BurstIntegration.DifferentiateAngular(newRotation, rotation, dt);
        }
    }
}
#endif