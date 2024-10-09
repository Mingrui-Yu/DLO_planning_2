#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using Unity.Mathematics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace Obi
{
    public struct BurstContact : IConstraint, System.IComparable<BurstContact>
    {
        public float4 point;
        public float4 normal;
        public float4 tangent;
        public float4 bitangent;

        public float distance;

        float normalImpulse;
        float tangentImpulse;
        float bitangentImpulse;
        float stickImpulse;
        float rollingFrictionImpulse;

        public int entityA;
        public int entityB;

        public float normalInvMassA;
        public float tangentInvMassA;
        public float bitangentInvMassA;

        public float normalInvMassB;
        public float tangentInvMassB;
        public float bitangentInvMassB;

        public double pad0; // padding to ensure correct alignment to 128 bytes.

        public int GetParticleCount() { return 2; }
        public int GetParticle(int index) { return index == 0 ? entityA : entityB; }

        public override string ToString()
        {
            return entityA + "," + entityB;
        }

        public int CompareTo(BurstContact other)
        {
            int first = entityA.CompareTo(other.entityA);
            if (first == 0)
                return entityB.CompareTo(other.entityB);
            return first;
        }

        public float4 ContactPointA
        {
            get { return point + normal * distance; }
        }

        public float4 ContactPointB
        {
            get { return point; }
        }

        public float TotalNormalInvMass
        {
            get { return normalInvMassA + normalInvMassB; }
        }

        public float TotalTangentInvMass
        {
            get { return tangentInvMassA + tangentInvMassB; }
        }

        public float TotalBitangentInvMass
        {
            get { return bitangentInvMassA + bitangentInvMassB; }
        }

        public void CalculateBasis(float4 relativeVelocity)
        {
            tangent = math.normalizesafe(relativeVelocity - math.dot(relativeVelocity, normal) * normal);
            bitangent = math.normalizesafe(new float4(math.cross(normal.xyz, tangent.xyz),0));
        }

        public void CalculateContactMassesA(ref NativeArray<float> invMasses,
                                            ref NativeArray<float4> prevPositions,
                                            ref NativeArray<quaternion> orientations,
                                            ref NativeArray<float4> inverseInertiaTensors, bool rollingContacts)
        {
            // initialize inverse linear masses:
            normalInvMassA = tangentInvMassA = bitangentInvMassA = invMasses[entityA];

            if (rollingContacts)
            {
                float4 rA = ContactPointA - prevPositions[entityA];
                float4x4 solverInertiaA = BurstMath.TransformInertiaTensor(inverseInertiaTensors[entityA], orientations[entityA]);

                normalInvMassA += BurstMath.RotationalInvMass(solverInertiaA, rA, normal);
                tangentInvMassA += BurstMath.RotationalInvMass(solverInertiaA, rA, tangent);
                bitangentInvMassA += BurstMath.RotationalInvMass(solverInertiaA, rA, bitangent);
            }
        }

        public void CalculateContactMassesB(ref NativeArray<float> invMasses,
                                            ref NativeArray<float4> prevPositions,
                                            ref NativeArray<quaternion> orientations,
                                            ref NativeArray<float4> inverseInertiaTensors, bool rollingContacts)
        {
            // initialize inverse linear masses:
            normalInvMassB = tangentInvMassB = bitangentInvMassB = invMasses[entityB];

            if (rollingContacts)
            {
                float4 rB = ContactPointB - prevPositions[entityB];
                float4x4 solverInertiaB = BurstMath.TransformInertiaTensor(inverseInertiaTensors[entityB], orientations[entityB]);

                normalInvMassB += BurstMath.RotationalInvMass(solverInertiaB, rB, normal);
                tangentInvMassB += BurstMath.RotationalInvMass(solverInertiaB, rB, tangent);
                bitangentInvMassB += BurstMath.RotationalInvMass(solverInertiaB, rB, bitangent);
            }
        }

        public void CalculateContactMassesB(BurstRigidbody rigidbody, bool rollingContacts)
        {
            // initialize inverse linear masses:
            normalInvMassB = tangentInvMassB = bitangentInvMassB = rigidbody.inverseMass;

            if (rollingContacts)
            {
                float4 rB = ContactPointB - rigidbody.com;

                normalInvMassB += BurstMath.RotationalInvMass(rigidbody.inverseInertiaTensor, rB, normal);
                tangentInvMassB += BurstMath.RotationalInvMass(rigidbody.inverseInertiaTensor, rB, tangent);
                bitangentInvMassB += BurstMath.RotationalInvMass(rigidbody.inverseInertiaTensor, rB, bitangent);
            }
        }

        public float SolveAdhesion(float stickDistance, float stickiness, float dt)
        {

            if (TotalNormalInvMass <= 0 || stickDistance <= 0 || stickiness <= 0 || dt <= 0)
                return 0;

            // calculate stickiness impulse correction:
            float stickinessSpeed = stickiness * (1 - math.max(distance / stickDistance, 0));
            float newStickinessImpulse = math.max(stickImpulse + stickinessSpeed / TotalNormalInvMass, 0);

            float stickinessImpulseChange = newStickinessImpulse - stickImpulse;
            stickImpulse = newStickinessImpulse;

            return stickinessImpulseChange;
        }

        public float SolvePenetration(float4 relativeVelocity, float maxDepenetrationVelocity, float dt)
        {

            if (TotalNormalInvMass <= 0 || dt <= 0)
                return 0;

            // project relativeVelocity to normal vector:
            float relativeNormalSpeed = math.dot(relativeVelocity, normal);

            // calculate normal impulse correction:
            float velocityCorrection = relativeNormalSpeed + math.max(distance / dt, -maxDepenetrationVelocity);
            float impulseCorrection = velocityCorrection / TotalNormalInvMass;

            // accumulate impulse:
            float newImpulse = math.min(normalImpulse + impulseCorrection, 0);

            // calculate normal impulse change and update accumulated impulse:
            float impulseChange = newImpulse - normalImpulse;
            normalImpulse = newImpulse;

            return impulseChange;
        }

        public float2 SolveFriction(float4 relativeVelocity, float staticFriction, float dynamicFriction, float dt)
        {
            float2 impulseChange = float2.zero;

            if (TotalTangentInvMass <= 0 || TotalBitangentInvMass <= 0 || dt <= 0 ||
                (dynamicFriction <= 0 && staticFriction <= 0) || (normalImpulse >= 0 && stickImpulse <= 0))
                return impulseChange;

            // calculate relative frictional speed:
            float relativeTangentSpeed = math.dot(relativeVelocity,tangent);
            float relativeBitangentSpeed = math.dot(relativeVelocity,bitangent);

            // calculate friction cone (or rather, pyramid) limit:
            float frictionCone = -normalImpulse * dynamicFriction;
            float staticFrictionCone = -normalImpulse * staticFriction;

            // tangent impulse:
            float tangentImpulseCorr = -relativeTangentSpeed / TotalTangentInvMass;
            float newTangentImpulse = tangentImpulse + tangentImpulseCorr;

            if (math.abs(newTangentImpulse) > staticFrictionCone)
                newTangentImpulse = math.clamp(newTangentImpulse, -frictionCone, frictionCone);

            impulseChange[0] = newTangentImpulse - tangentImpulse;
            tangentImpulse = newTangentImpulse;

            // bitangent impulse:
            float bitangentImpulseCorr = -relativeBitangentSpeed / TotalBitangentInvMass;
            float newBitangentImpulse = bitangentImpulse + bitangentImpulseCorr;

            if (math.abs(newBitangentImpulse) > staticFrictionCone)
                newBitangentImpulse = math.clamp(newBitangentImpulse, -frictionCone, frictionCone);

            impulseChange[1] = newBitangentImpulse - bitangentImpulse;
            bitangentImpulse = newBitangentImpulse;

            return impulseChange;
        }

      
        public float SolveRollingFriction(float4 angularVelocityA,
                                   float4 angularVelocityB,
                                   float rollingFriction,
                                   float invMassA,
                                   float invMassB,
                                   ref float4 rolling_axis)
        {
            float totalInvMass = invMassA + invMassB;
            if (totalInvMass <= 0)
                return 0;
        
            rolling_axis = math.normalizesafe(angularVelocityA - angularVelocityB);

            float vel1 = math.dot(angularVelocityA,rolling_axis);
            float vel2 = math.dot(angularVelocityB,rolling_axis);

            float relativeVelocity = vel1 - vel2;

            float maxImpulse = -normalImpulse * rollingFriction;
            float newRollingImpulse = math.min(math.max(rollingFrictionImpulse - relativeVelocity / totalInvMass, -maxImpulse), maxImpulse);
            float rolling_impulse_change = newRollingImpulse - rollingFrictionImpulse;
            rollingFrictionImpulse = newRollingImpulse;
        
            return rolling_impulse_change;
        }
}
}
#endif