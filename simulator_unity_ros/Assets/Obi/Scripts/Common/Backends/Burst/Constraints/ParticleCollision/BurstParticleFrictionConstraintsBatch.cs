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
    public class BurstParticleFrictionConstraintsBatch : BurstConstraintsBatchImpl, IParticleFrictionConstraintsBatchImpl
    {
        public BatchData batchData;

        public BurstParticleFrictionConstraintsBatch(BurstParticleFrictionConstraints constraints)
        {
            m_Constraints = constraints;
            m_ConstraintType = Oni.ConstraintType.ParticleFriction;
        }

        public BurstParticleFrictionConstraintsBatch(BatchData batchData) : base()
        {
            this.batchData = batchData;
        }

        public override JobHandle Initialize(JobHandle inputDeps, float deltaTime)
        {
            return inputDeps;
        }

        public override JobHandle Evaluate(JobHandle inputDeps, float deltaTime)
        {
            if (!((BurstSolverImpl)constraints.solver).particleContacts.IsCreated)
                return inputDeps;

            var projectConstraints = new ParticleFrictionConstraintsBatchJob()
            {
                positions = solverImplementation.positions,
                prevPositions = solverImplementation.prevPositions,
                orientations = solverImplementation.orientations,
                prevOrientations = solverImplementation.prevOrientations,

                invMasses = solverImplementation.invMasses,
                invInertiaTensors = solverImplementation.invInertiaTensors,
                radii = solverImplementation.principalRadii,
                particleMaterialIndices = solverImplementation.collisionMaterials,
                collisionMaterials = ObiColliderWorld.GetInstance().collisionMaterials.AsNativeArray<BurstCollisionMaterial>(),

                deltas = solverImplementation.positionDeltas,
                counts = solverImplementation.positionConstraintCounts,
                orientationDeltas = solverImplementation.orientationDeltas,
                orientationCounts = solverImplementation.orientationConstraintCounts,
                contacts = ((BurstSolverImpl)constraints.solver).particleContacts,

                batchData = batchData,
                maxDepenetrationVelocity = solverAbstraction.parameters.maxDepenetration,
                dt = deltaTime,
            };

            int batchCount = batchData.isLast ? batchData.workItemCount : 1;
            return projectConstraints.Schedule(batchData.workItemCount, batchCount, inputDeps);
        }

        public override JobHandle Apply(JobHandle inputDeps, float deltaTime)
        {
            if (!((BurstSolverImpl)constraints.solver).particleContacts.IsCreated)
                return inputDeps;

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
                constraintParameters = parameters,
                batchData = batchData
            };

            int batchCount = batchData.isLast ? batchData.workItemCount : 1;
            return applyConstraints.Schedule(batchData.workItemCount, batchCount, inputDeps);
        }

        [BurstCompile]
        public struct ParticleFrictionConstraintsBatchJob : IJobParallelFor
        {

            [ReadOnly] public NativeArray<float4> positions;
            [ReadOnly] public NativeArray<float4> prevPositions;
            [ReadOnly] public NativeArray<quaternion> orientations;
            [ReadOnly] public NativeArray<quaternion> prevOrientations;

            [ReadOnly] public NativeArray<float> invMasses;
            [ReadOnly] public NativeArray<float4> invInertiaTensors;
            [ReadOnly] public NativeArray<float4> radii;
            [ReadOnly] public NativeArray<int> particleMaterialIndices;
            [ReadOnly] public NativeArray<BurstCollisionMaterial> collisionMaterials;

            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<int> counts;

            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<quaternion> orientationDeltas;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<int> orientationCounts;

            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<BurstContact> contacts;

            [ReadOnly] public BatchData batchData;
            [ReadOnly] public float maxDepenetrationVelocity;
            [ReadOnly] public float dt;

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
                    BurstCollisionMaterial material = CombineCollisionMaterials(contact.entityA, contact.entityB);

                    // Calculate relative velocity:
                    float4 angularVelocityA = float4.zero, angularVelocityB = float4.zero, rA = float4.zero, rB = float4.zero;
                    float4 relativeVelocity = GetRelativeVelocity(indexA, indexB, ref contact, ref angularVelocityA, ref angularVelocityB, ref rA, ref rB, material.rollingContacts > 0);

                    // Calculate friction impulses (in the tangent and bitangent ddirections):
                    float2 impulses = contact.SolveFriction(relativeVelocity, material.staticFriction, material.dynamicFriction, dt);

                    // Apply friction impulses to both particles:
                    if (math.abs(impulses.x) > BurstMath.epsilon || math.abs(impulses.y) > BurstMath.epsilon)
                    {
                        float4 tangentImpulse = impulses.x * contact.tangent;
                        float4 bitangentImpulse = impulses.y * contact.bitangent;
                        float4 totalImpulse = tangentImpulse + bitangentImpulse;

                        deltas[indexA] += (tangentImpulse * contact.tangentInvMassA + bitangentImpulse * contact.bitangentInvMassA) * dt;
                        deltas[indexB] -= (tangentImpulse * contact.tangentInvMassB + bitangentImpulse * contact.bitangentInvMassB) * dt;
                        counts[indexA]++;
                        counts[indexB]++;

                        // Rolling contacts:
                        if (material.rollingContacts > 0)
                        {
                            // Calculate angular velocity deltas due to friction impulse:
                            float4x4 solverInertiaA = BurstMath.TransformInertiaTensor(invInertiaTensors[indexA], orientations[indexA]);
                            float4x4 solverInertiaB = BurstMath.TransformInertiaTensor(invInertiaTensors[indexB], orientations[indexB]);

                            float4 angVelDeltaA = math.mul(solverInertiaA, new float4(math.cross(rA.xyz, totalImpulse.xyz), 0));
                            float4 angVelDeltaB = -math.mul(solverInertiaB, new float4(math.cross(rB.xyz, totalImpulse.xyz), 0));

                            // Final angular velocities, after adding the deltas:
                            angularVelocityA += angVelDeltaA;
                            angularVelocityB += angVelDeltaB;

                            // Calculate weights (inverse masses):
                            float invMassA = math.length(math.mul(solverInertiaA, math.normalizesafe(angularVelocityA)));
                            float invMassB = math.length(math.mul(solverInertiaB, math.normalizesafe(angularVelocityB)));

                            // Calculate rolling axis and angular velocity deltas:
                            float4 rollAxis = float4.zero;
                            float rollingImpulse = contact.SolveRollingFriction(angularVelocityA, angularVelocityB, material.rollingFriction, invMassA, invMassB, ref rollAxis);
                            angVelDeltaA += rollAxis * rollingImpulse * invMassA;
                            angVelDeltaB -= rollAxis * rollingImpulse * invMassB;

                            // Apply orientation deltas to particles:
                            quaternion orientationDeltaA = BurstIntegration.AngularVelocityToSpinQuaternion(orientations[indexA], angVelDeltaA);
                            quaternion orientationDeltaB = BurstIntegration.AngularVelocityToSpinQuaternion(orientations[indexB], angVelDeltaB);

                            quaternion qA = orientationDeltas[indexA];
                            qA.value += orientationDeltaA.value * dt;
                            orientationDeltas[indexA] = qA;
                            orientationCounts[indexA]++;

                            quaternion qB = orientationDeltas[indexB];
                            qB.value += orientationDeltaB.value * dt;
                            orientationDeltas[indexB] = qB;
                            orientationCounts[indexB]++;
                        }
                    }

                    contacts[i] = contact;
                }
            }

            private float4 GetRelativeVelocity(int particleIndexA, int particleIndexB, ref BurstContact contact, ref float4 angularVelocityA, ref float4 angularVelocityB, ref float4 rA, ref float4 rB, bool rollingContacts)
            {
                // Initialize with particle linear velocity:
                float4 velA = (positions[particleIndexA] - prevPositions[particleIndexA]) / dt;
                float4 velB = (positions[particleIndexB] - prevPositions[particleIndexB]) / dt;

                // Consider angular velocities if rolling contacts are enabled:
                if (rollingContacts)
                {
                    angularVelocityA = BurstIntegration.DifferentiateAngular(orientations[particleIndexA], prevOrientations[particleIndexA], dt);
                    angularVelocityB = BurstIntegration.DifferentiateAngular(orientations[particleIndexB], prevOrientations[particleIndexB], dt);

                    rA = contact.ContactPointA - prevPositions[particleIndexA];
                    rB = contact.ContactPointB - prevPositions[particleIndexB];

                    velA += new float4(math.cross(angularVelocityA.xyz, rA.xyz), 0);
                    velB += new float4(math.cross(angularVelocityB.xyz, rB.xyz), 0);
                }

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