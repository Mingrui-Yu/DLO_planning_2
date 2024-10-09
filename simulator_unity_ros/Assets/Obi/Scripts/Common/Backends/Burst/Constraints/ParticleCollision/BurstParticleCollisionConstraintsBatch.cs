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
    public class BurstParticleCollisionConstraintsBatch : BurstConstraintsBatchImpl, IParticleCollisionConstraintsBatchImpl
    {
        public BatchData batchData;

        public BurstParticleCollisionConstraintsBatch(BurstParticleCollisionConstraints constraints)
        {
            m_Constraints = constraints;
            m_ConstraintType = Oni.ConstraintType.ParticleCollision;
        }

        public BurstParticleCollisionConstraintsBatch(BatchData batchData) : base()
        {
            this.batchData = batchData;
        }

        public override JobHandle Initialize(JobHandle inputDeps, float deltaTime)
        {
            var updateContacts = new UpdateParticleContactsJob()
            {
                prevPositions = solverImplementation.prevPositions,
                prevOrientations = solverImplementation.prevOrientations,
                velocities = solverImplementation.velocities,
                radii = solverImplementation.principalRadii,
                invMasses = solverImplementation.invMasses,
                invInertiaTensors = solverImplementation.invInertiaTensors,

                particleMaterialIndices = solverImplementation.collisionMaterials,
                collisionMaterials = ObiColliderWorld.GetInstance().collisionMaterials.AsNativeArray<BurstCollisionMaterial>(),

                contacts = ((BurstSolverImpl)constraints.solver).particleContacts,
                batchData = batchData
            };

            int batchCount = batchData.isLast ? batchData.workItemCount : 1;
            return updateContacts.Schedule(batchData.workItemCount, batchCount, inputDeps);
        }

        public override JobHandle Evaluate(JobHandle inputDeps, float deltaTime)
        {
            var parameters = solverAbstraction.GetConstraintParameters(m_ConstraintType);

            var projectConstraints = new ParticleCollisionConstraintsBatchJob()
            {
                positions = solverImplementation.positions,
                prevPositions = solverImplementation.prevPositions,
                invMasses = solverImplementation.invMasses,
                radii = solverImplementation.principalRadii,
                particleMaterialIndices = solverImplementation.collisionMaterials,
                collisionMaterials = ObiColliderWorld.GetInstance().collisionMaterials.AsNativeArray<BurstCollisionMaterial>(),

                deltas = solverImplementation.positionDeltas,
                counts = solverImplementation.positionConstraintCounts,
                contacts = ((BurstSolverImpl)constraints.solver).particleContacts,
                batchData = batchData,

                constraintParameters = parameters,
                solverParameters = solverImplementation.abstraction.parameters,
                gravity = new float4(solverImplementation.abstraction.parameters.gravity, 0),
                dt = deltaTime,
            };

            int batchCount = batchData.isLast ? batchData.workItemCount : 1;
            return projectConstraints.Schedule(batchData.workItemCount, batchCount, inputDeps);
        }

        public override JobHandle Apply(JobHandle inputDeps, float deltaTime)
        {
            var parameters = solverAbstraction.GetConstraintParameters(m_ConstraintType);

            var applyConstraints = new ApplyBatchedCollisionConstraintsBatchJob()
            {
                contacts = ((BurstSolverImpl)constraints.solver).particleContacts,
                positions = solverImplementation.positions,
                deltas = solverImplementation.positionDeltas,
                counts = solverImplementation.positionConstraintCounts,
                orientations = solverImplementation.orientations,
                orientationDeltas = solverImplementation.orientationDeltas,
                orientationCounts = solverImplementation.orientationConstraintCounts,
                batchData = batchData,
                constraintParameters = parameters,
            };

            int batchCount = batchData.isLast ? batchData.workItemCount : 1;
            return applyConstraints.Schedule(batchData.workItemCount, batchCount, inputDeps);
        }

        /**
         * Updates contact data (contact distance and frame) at the beginning of each substep. This is
         * necessary because contacts are generated only once at the beginning of each step, not every substep.
         */
        [BurstCompile]
        public struct UpdateParticleContactsJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float4> prevPositions;
            [ReadOnly] public NativeArray<quaternion> prevOrientations;
            [ReadOnly] public NativeArray<float4> velocities;
            [ReadOnly] public NativeArray<float4> radii;
            [ReadOnly] public NativeArray<float> invMasses;
            [ReadOnly] public NativeArray<float4> invInertiaTensors;

            [ReadOnly] public NativeArray<int> particleMaterialIndices;
            [ReadOnly] public NativeArray<BurstCollisionMaterial> collisionMaterials;

            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<BurstContact> contacts;

            [ReadOnly] public BatchData batchData;

            public void Execute(int workItemIndex)
            {
                int start, end;
                batchData.GetConstraintRange(workItemIndex, out start, out end);

                for (int i = start; i < end; ++i)
                {
                    var contact = contacts[i];

                    // update contact basis:
                    contact.CalculateBasis(velocities[contact.entityA] - velocities[contact.entityB]);

                    // update contact masses:
                    int aMaterialIndex = particleMaterialIndices[contact.entityA];
                    int bMaterialIndex = particleMaterialIndices[contact.entityB];
                    bool rollingContacts = (aMaterialIndex >= 0 ? collisionMaterials[aMaterialIndex].rollingContacts > 0 : false) | 
                                           (bMaterialIndex >= 0 ? collisionMaterials[bMaterialIndex].rollingContacts > 0 : false);

                    contact.CalculateContactMassesA(ref invMasses, ref prevPositions, ref prevOrientations, ref invInertiaTensors, rollingContacts);
                    contact.CalculateContactMassesB(ref invMasses, ref prevPositions, ref prevOrientations, ref invInertiaTensors, rollingContacts);

                    // update contact distance:
                    float dAB = math.dot(prevPositions[contact.entityA] - prevPositions[contact.entityB], contact.normal);
                    float dA = BurstMath.EllipsoidRadius(contact.normal, prevOrientations[contact.entityA], radii[contact.entityA].xyz);
                    float dB = BurstMath.EllipsoidRadius(contact.normal, prevOrientations[contact.entityB], radii[contact.entityB].xyz);
                    contact.distance = dAB - (dA + dB);

                    contacts[i] = contact;
                }
            }
        }

        [BurstCompile]
        public struct ParticleCollisionConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float4> prevPositions;
            [ReadOnly] public NativeArray<float> invMasses;
            [ReadOnly] public NativeArray<float4> radii;
            [ReadOnly] public NativeArray<int> particleMaterialIndices;
            [ReadOnly] public NativeArray<BurstCollisionMaterial> collisionMaterials;

            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> positions;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<int> counts;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<BurstContact> contacts;

            [ReadOnly] public Oni.ConstraintParameters constraintParameters;
            [ReadOnly] public Oni.SolverParameters solverParameters;
            [ReadOnly] public float4 gravity;
            [ReadOnly] public float dt;

            [ReadOnly] public BatchData batchData;

            public void Execute(int workItemIndex)
            {
                int start, end;
                batchData.GetConstraintRange(workItemIndex, out start, out end);

                for (int i = start; i < end; ++i)
                {
                    var contact = contacts[i];

                    int indexA = contact.entityA;
                    int indexB = contact.entityB;

                    // Combine collision materials:
                    BurstCollisionMaterial material = CombineCollisionMaterials(indexA, indexB);

                    // Calculate relative velocity:
                    float4 relativeVelocity = GetRelativeVelocity(indexA, indexB);

                    // Determine adhesion impulse magnitude:
                    float adhesionImpulse = contact.SolveAdhesion(material.stickDistance, material.stickiness, dt);

                    // Determine depenetration impulse magnitude:
                    float depenetrationImpulse = contact.SolvePenetration(relativeVelocity, solverParameters.maxDepenetration, dt);

                    float totalImpulse = depenetrationImpulse + adhesionImpulse;

                    // Apply normal impulse to both particles (w/ shock propagation):
                    if (math.abs(totalImpulse) > BurstMath.epsilon)
                    {
                        float shock = solverParameters.shockPropagation * math.dot(contact.normal, math.normalizesafe(gravity));
                        float4 delta = totalImpulse * dt * -contact.normal;
                        deltas[indexA] += delta * contact.normalInvMassA * (1 - shock);
                        deltas[indexB] -= delta * contact.normalInvMassB * (1 + shock);
                        counts[indexA]++;
                        counts[indexB]++;
                    }

                    // Apply position deltas immediately, if using sequential evaluation:
                    if (constraintParameters.evaluationOrder == Oni.ConstraintParameters.EvaluationOrder.Sequential)
                    {
                        ApplyPositionDelta(indexA, constraintParameters.SORFactor, ref positions, ref deltas, ref counts);
                        ApplyPositionDelta(indexB, constraintParameters.SORFactor, ref positions, ref deltas, ref counts);
                    }

                    contacts[i] = contact;
                }
            }

            private float4 GetRelativeVelocity(int particleIndexA, int particleIndexB)
            {
                // Initialize with particle linear velocity:
                float4 velA = (positions[particleIndexA] - prevPositions[particleIndexA]) / dt;
                float4 velB = (positions[particleIndexB] - prevPositions[particleIndexB]) / dt;

                // As we do not consider true ellipses for collision detection, particle contact points are never off-axis.
                // So particle angular velocity does not contribute to normal impulses, and we can skip it.

                return velA - velB;
            }

            private BurstCollisionMaterial CombineCollisionMaterials(int entityA, int entityB)
            {
                // Combine collision materials:
                int aMaterialIndex = particleMaterialIndices[entityA];
                int bMaterialIndex = particleMaterialIndices[entityB];

                if (aMaterialIndex >= 0 && bMaterialIndex >= 0)
                    return BurstCollisionMaterial.CombineWith(collisionMaterials[aMaterialIndex], collisionMaterials[bMaterialIndex]);
                else if (aMaterialIndex >= 0)
                    return collisionMaterials[aMaterialIndex];
                else if (bMaterialIndex >= 0)
                    return collisionMaterials[bMaterialIndex];

                return new BurstCollisionMaterial();
            }
        }


    }
}
#endif