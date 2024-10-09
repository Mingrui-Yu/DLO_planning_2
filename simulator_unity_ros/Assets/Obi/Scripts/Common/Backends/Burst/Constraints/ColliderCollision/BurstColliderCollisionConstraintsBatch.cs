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
    public class BurstColliderCollisionConstraintsBatch : BurstConstraintsBatchImpl, IColliderCollisionConstraintsBatchImpl
    {
        public BurstColliderCollisionConstraintsBatch(BurstColliderCollisionConstraints constraints)
        {
            m_Constraints = constraints;
            m_ConstraintType = Oni.ConstraintType.Collision;
        }

        public override JobHandle Initialize(JobHandle inputDeps, float deltaTime)
        {
            var updateContacts = new UpdateContactsJob()
            {
                prevPositions = solverImplementation.prevPositions,
                prevOrientations = solverImplementation.prevOrientations,
                velocities = solverImplementation.velocities,
                radii = solverImplementation.principalRadii,
                invMasses = solverImplementation.invMasses,
                invInertiaTensors = solverImplementation.invInertiaTensors,
                particleMaterialIndices = solverImplementation.collisionMaterials,
                collisionMaterials = ObiColliderWorld.GetInstance().collisionMaterials.AsNativeArray<BurstCollisionMaterial>(),

                shapes = ObiColliderWorld.GetInstance().colliderShapes.AsNativeArray<BurstColliderShape>(),
                transforms = ObiColliderWorld.GetInstance().colliderTransforms.AsNativeArray<BurstAffineTransform>(),
                rigidbodies = ObiColliderWorld.GetInstance().rigidbodies.AsNativeArray<BurstRigidbody>(),
                rigidbodyLinearDeltas = solverImplementation.abstraction.rigidbodyLinearDeltas.AsNativeArray<float4>(),
                rigidbodyAngularDeltas = solverImplementation.abstraction.rigidbodyAngularDeltas.AsNativeArray<float4>(),

                contacts = ((BurstSolverImpl)constraints.solver).colliderContacts,
                inertialFrame = ((BurstSolverImpl)constraints.solver).inertialFrame,
            };

            return updateContacts.Schedule(((BurstSolverImpl)constraints.solver).colliderContacts.Length, 128, inputDeps);
        }

        public override JobHandle Evaluate(JobHandle inputDeps, float deltaTime)
        {
            var projectConstraints = new CollisionConstraintsBatchJob()
            {
                positions = solverImplementation.positions,
                prevPositions = solverImplementation.prevPositions,
                invMasses = solverImplementation.invMasses,
                radii = solverImplementation.principalRadii,
                particleMaterialIndices = solverImplementation.collisionMaterials,

                shapes = ObiColliderWorld.GetInstance().colliderShapes.AsNativeArray<BurstColliderShape>(),
                transforms = ObiColliderWorld.GetInstance().colliderTransforms.AsNativeArray<BurstAffineTransform>(),
                collisionMaterials = ObiColliderWorld.GetInstance().collisionMaterials.AsNativeArray<BurstCollisionMaterial>(),
                rigidbodies = ObiColliderWorld.GetInstance().rigidbodies.AsNativeArray<BurstRigidbody>(),
                rigidbodyLinearDeltas = solverImplementation.abstraction.rigidbodyLinearDeltas.AsNativeArray<float4>(),
                rigidbodyAngularDeltas = solverImplementation.abstraction.rigidbodyAngularDeltas.AsNativeArray<float4>(),

                deltas = solverAbstraction.positionDeltas.AsNativeArray<float4>(),
                counts = solverAbstraction.positionConstraintCounts.AsNativeArray<int>(),

                contacts = ((BurstSolverImpl)constraints.solver).colliderContacts,
                inertialFrame = ((BurstSolverImpl)constraints.solver).inertialFrame,
                solverParameters = solverAbstraction.parameters,
                dt = deltaTime,
            };

            return projectConstraints.Schedule(inputDeps);
        }

        public override JobHandle Apply(JobHandle inputDeps, float deltaTime)
        {
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

        /**
         * Updates contact data (such as contact distance) at the beginning of each substep. This is
         * necessary because contacts are generalted only once at the beginning of each step, not every substep.
         */
        [BurstCompile]
        public struct UpdateContactsJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float4> prevPositions;
            [ReadOnly] public NativeArray<quaternion> prevOrientations;
            [ReadOnly] public NativeArray<float4> velocities;
            [ReadOnly] public NativeArray<float4> radii;
            [ReadOnly] public NativeArray<float> invMasses;
            [ReadOnly] public NativeArray<float4> invInertiaTensors;

            [ReadOnly] public NativeArray<int> particleMaterialIndices;
            [ReadOnly] public NativeArray<BurstCollisionMaterial> collisionMaterials;

            [ReadOnly] public NativeArray<BurstColliderShape> shapes;
            [ReadOnly] public NativeArray<BurstAffineTransform> transforms;
            [ReadOnly] public NativeArray<BurstRigidbody> rigidbodies;
            [ReadOnly] public NativeArray<float4> rigidbodyLinearDeltas;
            [ReadOnly] public NativeArray<float4> rigidbodyAngularDeltas;

            public NativeArray<BurstContact> contacts;
            [ReadOnly] public BurstInertialFrame inertialFrame;

            public void Execute(int i)
            {
                var contact = contacts[i];

                int aMaterialIndex = particleMaterialIndices[contact.entityA];
                bool rollingContacts = aMaterialIndex >= 0 ? collisionMaterials[aMaterialIndex].rollingContacts > 0 : false;

                int rigidbodyIndex = shapes[contact.entityB].rigidbodyIndex;
                if (rigidbodyIndex >= 0)
                {
                    // update contact basis:
                    float4 relativeVelocity = velocities[contact.entityA] - BurstMath.GetRigidbodyVelocityAtPoint(rigidbodies[rigidbodyIndex], contact.ContactPointB, rigidbodyLinearDeltas[rigidbodyIndex], rigidbodyAngularDeltas[rigidbodyIndex], inertialFrame.frame);
                    contact.CalculateBasis(relativeVelocity);

                    int bMaterialIndex = shapes[contact.entityB].materialIndex;
                    rollingContacts |= bMaterialIndex >= 0 ? collisionMaterials[bMaterialIndex].rollingContacts > 0 : false;

                    // update contact masses:
                    contact.CalculateContactMassesA(ref invMasses, ref prevPositions, ref prevOrientations, ref invInertiaTensors, rollingContacts);
                    contact.CalculateContactMassesB(rigidbodies[rigidbodyIndex], false);

                }
                else
                {
                    // update contact basis:
                    contact.CalculateBasis(velocities[contact.entityA]);

                    // update contact masses:
                    contact.CalculateContactMassesA(ref invMasses, ref prevPositions, ref prevOrientations, ref invInertiaTensors, rollingContacts);
                }

                // update contact distance
                float dAB = math.dot(prevPositions[contact.entityA] - contact.point, contact.normal);
                float dA = BurstMath.EllipsoidRadius(contact.normal, prevOrientations[contact.entityA], radii[contact.entityA].xyz);
                float dB = shapes[contact.entityB].contactOffset;

                contact.distance = dAB - (dA + dB);

                contacts[i] = contact;
            }
        }

        [BurstCompile]
        public struct CollisionConstraintsBatchJob : IJob
        {

            [ReadOnly] public NativeArray<float4> positions;
            [ReadOnly] public NativeArray<float4> prevPositions;
            [ReadOnly] public NativeArray<float> invMasses;
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

            public NativeArray<BurstContact> contacts;
            [ReadOnly] public BurstInertialFrame inertialFrame;
            [ReadOnly] public Oni.SolverParameters solverParameters;
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
                    float4 relativeVelocity = GetRelativeVelocity(indexA, rigidbodyIndex, ref contact);

                    // Determine adhesion impulse magnitude:
                    float adhesionImpulse = contact.SolveAdhesion(material.stickDistance, material.stickiness, dt);

                    // Determine depenetration impulse magnitude:
                    float depenetrationImpulse = contact.SolvePenetration(relativeVelocity, solverParameters.maxDepenetration, dt);

                    float totalImpulse = adhesionImpulse + depenetrationImpulse;

                    // Apply normal impulse to both particle and rigidbody:
                    if (math.abs(totalImpulse) > BurstMath.epsilon)
                    {
                        deltas[indexA] += totalImpulse * -contact.normal * contact.normalInvMassA * dt;
                        counts[indexA]++;

                        if (rigidbodyIndex >= 0)
                        {
                            var rb = rigidbodies[rigidbodyIndex];

                            float4 worldImpulse = inertialFrame.frame.TransformVector(totalImpulse * contact.normal);
                            float4 worldPoint   = inertialFrame.frame.TransformPoint(contact.point);

                            rigidbodyLinearDeltas[rigidbodyIndex]  += rb.inverseMass * worldImpulse;
                            rigidbodyAngularDeltas[rigidbodyIndex] += math.mul(rb.inverseInertiaTensor, new float4(math.cross((worldPoint - rb.com).xyz, worldImpulse.xyz), 0));
                        }
                    }

                    contacts[i] = contact;
                }
            }

            private float4 GetRelativeVelocity(int particleIndex, int rigidbodyIndex, ref BurstContact contact)
            {
                // Initialize with particle linear velocity:
                float4 relativeVelocity = (positions[particleIndex] - prevPositions[particleIndex]) / dt;

                // As we do not consider true ellipses for collision detection, particle contact points are never off-axis.
                // So particle angular velocity does not contribute to normal impulses, and we can skip it.

                // Subtract rigidbody velocity:
                if (rigidbodyIndex >= 0)
                    relativeVelocity -= BurstMath.GetRigidbodyVelocityAtPoint(rigidbodies[rigidbodyIndex], contact.ContactPointB, rigidbodyLinearDeltas[rigidbodyIndex], rigidbodyAngularDeltas[rigidbodyIndex], inertialFrame.frame);

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