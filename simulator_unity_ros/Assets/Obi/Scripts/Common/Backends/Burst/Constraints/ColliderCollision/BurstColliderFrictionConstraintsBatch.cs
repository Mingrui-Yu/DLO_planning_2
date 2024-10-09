#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using UnityEngine;
using Unity.Jobs;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Burst;
using System.Collections;

namespace Obi
{
    public class BurstColliderFrictionConstraintsBatch : BurstConstraintsBatchImpl, IColliderFrictionConstraintsBatchImpl
    {
        public BurstColliderFrictionConstraintsBatch(BurstColliderFrictionConstraints constraints)
        {
            m_Constraints = constraints;
            m_ConstraintType = Oni.ConstraintType.Friction;
        }

        public override JobHandle Initialize(JobHandle inputDeps, float deltaTime)
        {
            return inputDeps;
        }

        public override JobHandle Evaluate(JobHandle inputDeps, float deltaTime)
        {
            if (!((BurstSolverImpl)constraints.solver).colliderContacts.IsCreated)
                return inputDeps;

            var projectConstraints = new FrictionConstraintsBatchJob()
            {
                positions = solverImplementation.positions,
                prevPositions = solverImplementation.prevPositions,
                orientations = solverImplementation.orientations,
                prevOrientations = solverImplementation.prevOrientations,

                invMasses = solverImplementation.invMasses,
                invInertiaTensors = solverImplementation.invInertiaTensors,
                radii = solverImplementation.principalRadii,
                particleMaterialIndices = solverImplementation.collisionMaterials,

                shapes = ObiColliderWorld.GetInstance().colliderShapes.AsNativeArray<BurstColliderShape>(),
                transforms = ObiColliderWorld.GetInstance().colliderTransforms.AsNativeArray<BurstAffineTransform>(),
                collisionMaterials = ObiColliderWorld.GetInstance().collisionMaterials.AsNativeArray<BurstCollisionMaterial>(),
                rigidbodies = ObiColliderWorld.GetInstance().rigidbodies.AsNativeArray<BurstRigidbody>(),
                rigidbodyLinearDeltas = solverImplementation.abstraction.rigidbodyLinearDeltas.AsNativeArray<float4>(),
                rigidbodyAngularDeltas = solverImplementation.abstraction.rigidbodyAngularDeltas.AsNativeArray<float4>(),

                deltas = solverImplementation.positionDeltas,
                counts = solverImplementation.positionConstraintCounts,
                orientationDeltas = solverImplementation.orientationDeltas,
                orientationCounts = solverImplementation.orientationConstraintCounts,

                contacts = ((BurstSolverImpl)constraints.solver).colliderContacts,
                inertialFrame = ((BurstSolverImpl)constraints.solver).inertialFrame,
                dt = deltaTime,
            };

            return projectConstraints.Schedule(inputDeps);
        }

        public override JobHandle Apply(JobHandle inputDeps, float deltaTime)
        {
            if (!((BurstSolverImpl)constraints.solver).colliderContacts.IsCreated)
                return inputDeps;

            var parameters = solverAbstraction.GetConstraintParameters(m_ConstraintType);

            var applyConstraints = new ApplyCollisionConstraintsBatchJob()
            {
                contacts = ((BurstSolverImpl)constraints.solver).colliderContacts,
                positions = solverImplementation.positions,
                deltas = solverImplementation.positionDeltas,
                counts = solverImplementation.positionConstraintCounts,
                orientations = solverImplementation.orientations,
                orientationDeltas = solverImplementation.orientationDeltas,
                orientationCounts = solverImplementation.orientationConstraintCounts,
                constraintParameters = parameters
            };

            return applyConstraints.Schedule(inputDeps);
        }

        [BurstCompile]
        public struct FrictionConstraintsBatchJob : IJob
        {
            [ReadOnly] public NativeArray<float4> positions;
            [ReadOnly] public NativeArray<float4> prevPositions;
            [ReadOnly] public NativeArray<quaternion> orientations;
            [ReadOnly] public NativeArray<quaternion> prevOrientations;

            [ReadOnly] public NativeArray<float> invMasses;
            [ReadOnly] public NativeArray<float4> invInertiaTensors;
            [ReadOnly] public NativeArray<float4> radii;
            [ReadOnly] public NativeArray<int> particleMaterialIndices;

            [ReadOnly] public NativeArray<BurstColliderShape> shapes;
            [ReadOnly] public NativeArray<BurstAffineTransform> transforms;
            [ReadOnly] public NativeArray<BurstCollisionMaterial> collisionMaterials;
            [ReadOnly] public NativeArray<BurstRigidbody> rigidbodies;
            public NativeArray<float4> rigidbodyLinearDeltas;
            public NativeArray<float4> rigidbodyAngularDeltas;

            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<int> counts;

            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<quaternion> orientationDeltas;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<int> orientationCounts;

            public NativeArray<BurstContact> contacts;
            [ReadOnly] public BurstInertialFrame inertialFrame;
            [ReadOnly] public float dt;

            public void Execute()
            {
                for (int i = 0; i < contacts.Length; ++i)
                {
                    var contact = contacts[i];

                    // Get the indices of the particle and collider involved in this contact:
                    int indexA = contact.entityA;
                    int indexB = contact.entityB;

                    // Skip contacts involving triggers:
                    if (shapes[indexB].flags > 0)
                        continue;

                    // Get the rigidbody index (might be < 0, in that case there's no rigidbody present)
                    int rigidbodyIndex = shapes[indexB].rigidbodyIndex;

                    // Combine collision materials:
                    BurstCollisionMaterial material = CombineCollisionMaterials(indexA, indexB);

                    // Calculate relative velocity:
                    float4 angularVelocityA = float4.zero, rA = float4.zero, rB = float4.zero;
                    float4 relativeVelocity = GetRelativeVelocity(indexA, rigidbodyIndex, ref contact, ref angularVelocityA, ref rA, ref rB, material.rollingContacts > 0);

                    // Determine impulse magnitude:
                    float2 impulses = contact.SolveFriction(relativeVelocity, material.staticFriction, material.dynamicFriction, dt);

                    if (math.abs(impulses.x) > BurstMath.epsilon || math.abs(impulses.y) > BurstMath.epsilon)
                    {
                        float4 tangentImpulse   = impulses.x * contact.tangent;
                        float4 bitangentImpulse = impulses.y * contact.bitangent;
                        float4 totalImpulse = tangentImpulse + bitangentImpulse;

                        deltas[indexA] += (tangentImpulse * contact.tangentInvMassA + bitangentImpulse * contact.bitangentInvMassA) * dt;
                        counts[indexA]++;

                        if (rigidbodyIndex >= 0)
                        {
                            var rb = rigidbodies[rigidbodyIndex];

                            float4 worldImpulse = -inertialFrame.frame.TransformVector(totalImpulse);
                            float4 worldPoint = inertialFrame.frame.TransformPoint(contact.point);

                            rigidbodyLinearDeltas[rigidbodyIndex] += rb.inverseMass * worldImpulse;
                            rigidbodyAngularDeltas[rigidbodyIndex] += math.mul(rb.inverseInertiaTensor, new float4(math.cross((worldPoint - rb.com).xyz, worldImpulse.xyz), 0));
                        }

                        // Rolling contacts:
                        if (material.rollingContacts > 0)
                        {
                            // Calculate angular velocity deltas due to friction impulse:
                            float4x4 solverInertiaA = BurstMath.TransformInertiaTensor(invInertiaTensors[indexA], orientations[indexA]);

                            float4 angVelDeltaA = math.mul(solverInertiaA, new float4(math.cross(rA.xyz, totalImpulse.xyz), 0));
                            float4 angVelDeltaB = float4.zero;

                            // Final angular velocities, after adding the deltas:
                            angularVelocityA += angVelDeltaA;
                            float4 angularVelocityB = float4.zero;

                            // Calculate weights (inverse masses):
                            float invMassA = math.length(math.mul(solverInertiaA, math.normalizesafe(angularVelocityA)));
                            float invMassB = 0;

                            if (rigidbodyIndex >= 0)
                            {
                                angVelDeltaB = math.mul(-rigidbodies[rigidbodyIndex].inverseInertiaTensor, new float4(math.cross(rB.xyz, totalImpulse.xyz), 0));
                                angularVelocityB = rigidbodies[rigidbodyIndex].angularVelocity + angVelDeltaB;
                                invMassB = math.length(math.mul(rigidbodies[rigidbodyIndex].inverseInertiaTensor, math.normalizesafe(angularVelocityB)));
                            }

                            // Calculate rolling axis and angular velocity deltas:
                            float4 rollAxis = float4.zero;
                            float rollingImpulse = contact.SolveRollingFriction(angularVelocityA, angularVelocityB, material.rollingFriction, invMassA, invMassB, ref rollAxis);
                            angVelDeltaA += rollAxis * rollingImpulse * invMassA;
                            angVelDeltaB -= rollAxis * rollingImpulse * invMassB;

                            // Apply orientation delta to particle:
                            quaternion orientationDelta = BurstIntegration.AngularVelocityToSpinQuaternion(orientations[indexA], angVelDeltaA);

                            quaternion qA = orientationDeltas[indexA];
                            qA.value += orientationDelta.value * dt;
                            orientationDeltas[indexA] = qA;
                            orientationCounts[indexA]++;

                            // Apply angular velocity delta to rigidbody:
                            if (rigidbodyIndex >= 0)
                            {
                                float4 angularDelta = rigidbodyAngularDeltas[rigidbodyIndex];
                                angularDelta += angVelDeltaB;
                                rigidbodyAngularDeltas[rigidbodyIndex] = angularDelta;
                            }
                        }
                    }

                    contacts[i] = contact;
                }
            }

            private float4 GetRelativeVelocity(int particleIndex, int rigidbodyIndex, ref BurstContact contact, ref float4 angularVelocityA, ref float4 rA, ref float4 rB, bool rollingContacts)
            {
                // Initialize with particle linear velocity:
                float4 relativeVelocity = (positions[particleIndex] - prevPositions[particleIndex]) / dt;

                // Add particle angular velocity if rolling contacts are enabled:
                if (rollingContacts)
                {
                    angularVelocityA = BurstIntegration.DifferentiateAngular(orientations[particleIndex], prevOrientations[particleIndex], dt);
                    rA = contact.ContactPointA - prevPositions[particleIndex];
                    relativeVelocity += new float4(math.cross(angularVelocityA.xyz, rA.xyz), 0);
                }

                // Subtract rigidbody velocity:
                if (rigidbodyIndex >= 0)
                {
                    // Note: unlike rA, that is expressed in solver space, rB is expressed in world space.
                    rB = inertialFrame.frame.TransformPoint(contact.ContactPointB) - rigidbodies[rigidbodyIndex].com;
                    relativeVelocity -= BurstMath.GetRigidbodyVelocityAtPoint(rigidbodies[rigidbodyIndex], contact.ContactPointB, rigidbodyLinearDeltas[rigidbodyIndex], rigidbodyAngularDeltas[rigidbodyIndex], inertialFrame.frame);
                }

                return relativeVelocity;
            }

            private BurstCollisionMaterial CombineCollisionMaterials(int entityA, int entityB)
            {
                // Combine collision materials:
                int particleMaterialIndex = particleMaterialIndices[entityA];
                int colliderMaterialIndex = shapes[entityB].materialIndex;

                if (colliderMaterialIndex >= 0 && particleMaterialIndex >= 0)
                    return BurstCollisionMaterial.CombineWith(collisionMaterials[particleMaterialIndex], collisionMaterials[colliderMaterialIndex]);
                else if (particleMaterialIndex >= 0)
                    return collisionMaterials[particleMaterialIndex];
                else if (colliderMaterialIndex >= 0)
                    return collisionMaterials[colliderMaterialIndex];

                return new BurstCollisionMaterial();
            }

        }


    }
}
#endif