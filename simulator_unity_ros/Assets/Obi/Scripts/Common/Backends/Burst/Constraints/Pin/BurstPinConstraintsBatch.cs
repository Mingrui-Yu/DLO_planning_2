#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using UnityEngine;
using Unity.Jobs;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Burst;
using System.Collections.Generic;

namespace Obi
{
    public class BurstPinConstraintsBatch : BurstConstraintsBatchImpl, IPinConstraintsBatchImpl
    {
        private NativeArray<int> colliderIndices;
        private NativeArray<float4> offsets;
        private NativeArray<quaternion> restDarbouxVectors;
        private NativeArray<float2> stiffnesses;

        public BurstPinConstraintsBatch(BurstPinConstraints constraints)
        {
            m_Constraints = constraints;
            m_ConstraintType = Oni.ConstraintType.Pin;
        }

        public void SetPinConstraints(ObiNativeIntList particleIndices, ObiNativeIntList colliderIndices, ObiNativeVector4List offsets, ObiNativeQuaternionList restDarbouxVectors, ObiNativeFloatList stiffnesses, ObiNativeFloatList lambdas, int count)
        {
            this.particleIndices = particleIndices.AsNativeArray<int>();
            this.colliderIndices = colliderIndices.AsNativeArray<int>();
            this.offsets = offsets.AsNativeArray<float4>();
            this.restDarbouxVectors = restDarbouxVectors.AsNativeArray<quaternion>();
            this.stiffnesses = stiffnesses.AsNativeArray<float2>();
            this.lambdas = lambdas.AsNativeArray<float>();
            m_ConstraintCount = count;
        }

        public override JobHandle Evaluate(JobHandle inputDeps, float deltaTime)
        {
            var projectConstraints = new PinConstraintsBatchJob()
            {
                particleIndices = particleIndices,
                colliderIndices = colliderIndices,
                offsets = offsets,
                stiffnesses = stiffnesses,
                restDarboux = restDarbouxVectors,
                lambdas = lambdas.Reinterpret<float, float4>(),

                positions = solverImplementation.positions,
                invMasses = solverImplementation.invMasses,
                orientations = solverImplementation.orientations,
                invRotationalMasses = solverImplementation.invRotationalMasses,

                shapes = ObiColliderWorld.GetInstance().colliderShapes.AsNativeArray<BurstColliderShape>(),
                transforms = ObiColliderWorld.GetInstance().colliderTransforms.AsNativeArray<BurstAffineTransform>(),
                rigidbodies = ObiColliderWorld.GetInstance().rigidbodies.AsNativeArray<BurstRigidbody>(),
                rigidbodyLinearDeltas = solverImplementation.abstraction.rigidbodyLinearDeltas.AsNativeArray<float4>(),
                rigidbodyAngularDeltas = solverImplementation.abstraction.rigidbodyAngularDeltas.AsNativeArray<float4>(),

                deltas = solverImplementation.positionDeltas,
                counts = solverImplementation.positionConstraintCounts,
                orientationDeltas = solverImplementation.orientationDeltas,
                orientationCounts = solverImplementation.orientationConstraintCounts,

                inertialFrame = ((BurstSolverImpl)constraints.solver).inertialFrame,
                deltaTime = deltaTime,
                activeConstraintCount = m_ConstraintCount
            };

            return projectConstraints.Schedule(inputDeps);
        }

        public override JobHandle Apply(JobHandle inputDeps, float deltaTime)
        {
            var parameters = solverAbstraction.GetConstraintParameters(m_ConstraintType);

            var applyConstraints = new ApplyPinConstraintsBatchJob()
            {
                particleIndices = particleIndices,

                positions = solverImplementation.positions,
                deltas = solverImplementation.positionDeltas,
                counts = solverImplementation.positionConstraintCounts,

                orientations = solverImplementation.orientations,
                orientationDeltas = solverImplementation.orientationDeltas,
                orientationCounts = solverImplementation.orientationConstraintCounts,

                sorFactor = parameters.SORFactor,
                activeConstraintCount = m_ConstraintCount,
            };

            return applyConstraints.Schedule(inputDeps);
        }

        [BurstCompile]
        public unsafe struct PinConstraintsBatchJob : IJob
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] public NativeArray<int> colliderIndices;

            [ReadOnly] public NativeArray<float4> offsets;
            [ReadOnly] public NativeArray<float2> stiffnesses;
            [ReadOnly] public NativeArray<quaternion> restDarboux;
            public NativeArray<float4> lambdas;

            [ReadOnly] public NativeArray<float4> positions;
            [ReadOnly] public NativeArray<float> invMasses;
            [ReadOnly] public NativeArray<quaternion> orientations;
            [ReadOnly] public NativeArray<float> invRotationalMasses;

            [ReadOnly] public NativeArray<BurstColliderShape> shapes;
            [ReadOnly] public NativeArray<BurstAffineTransform> transforms;
            [ReadOnly] public NativeArray<BurstRigidbody> rigidbodies;
            public NativeArray<float4> rigidbodyLinearDeltas;
            public NativeArray<float4> rigidbodyAngularDeltas;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> counts;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<quaternion> orientationDeltas;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> orientationCounts;

            [ReadOnly] public BurstInertialFrame inertialFrame;
            [ReadOnly] public float deltaTime;
            [ReadOnly] public int activeConstraintCount;

            public void Execute()
            {
                for (int i = 0; i < activeConstraintCount; ++i)
                {
                    int particleIndex = particleIndices[i];
                    int colliderIndex = colliderIndices[i];

                    // no collider to pin to, so ignore the constraint.
                    if (colliderIndex < 0)
                        continue;

                    int rigidbodyIndex = shapes[colliderIndex].rigidbodyIndex;

                    // calculate time adjusted compliances
                    float2 compliances = stiffnesses[i].xy / (deltaTime * deltaTime);

                    float4 particlePosition = positions[particleIndex];

                    // express pin offset in world space:
                    float4 worldPinOffset = transforms[colliderIndex].TransformPoint(offsets[i]);
                    float4 predictedPinOffset = worldPinOffset;
                    quaternion predictedRotation = transforms[colliderIndex].rotation;

                    float rigidbodyLinearW = 0;
                    float rigidbodyAngularW = 0;

                    float4 linearRbDelta  = float4.zero;
                    float4 angularRbDelta = float4.zero;

                    if (rigidbodyIndex >= 0)
                    {
                        var rigidbody = rigidbodies[rigidbodyIndex];
                        linearRbDelta = rigidbodyLinearDeltas[rigidbodyIndex];
                        angularRbDelta = rigidbodyAngularDeltas[rigidbodyIndex];

                        // predict world-space position of offset point:
                        predictedPinOffset = BurstIntegration.IntegrateLinear(predictedPinOffset, rigidbody.GetVelocityAtPoint(worldPinOffset, linearRbDelta, angularRbDelta), deltaTime);

                        // predict rotation at the end of the step:
                        predictedRotation = BurstIntegration.IntegrateAngular(predictedRotation, rigidbody.angularVelocity + angularRbDelta, deltaTime);

                        // calculate linear and angular rigidbody weights:
                        rigidbodyLinearW = rigidbody.inverseMass;
                        rigidbodyAngularW = BurstMath.RotationalInvMass(rigidbody.inverseInertiaTensor,
                                                                        worldPinOffset - rigidbody.com,
                                                                        math.normalizesafe(inertialFrame.frame.TransformPoint(particlePosition) - predictedPinOffset));

                    }

                    // Transform pin position to solver space for constraint solving:
                    predictedPinOffset = inertialFrame.frame.InverseTransformPoint(predictedPinOffset);

                    float4 gradient = particlePosition - predictedPinOffset;
                    float constraint = math.length(gradient);
                    float4 gradientDir = gradient / (constraint + BurstMath.epsilon);

                    float4 lambda = lambdas[i];
                    float linearDLambda = (-constraint - compliances.x * lambda.w) / (invMasses[particleIndex] + rigidbodyLinearW + rigidbodyAngularW + compliances.x + BurstMath.epsilon);
                    lambda.w += linearDLambda;
                    float4 correction = linearDLambda * gradientDir;

                    deltas[particleIndex] += correction * invMasses[particleIndex];
                    counts[particleIndex]++;

                    if (rigidbodyAngularW > 0 || invRotationalMasses[particleIndex] > 0)
                    {
                        // bend/twist constraint:
                        quaternion omega = math.mul(math.conjugate(orientations[particleIndex]), predictedRotation);   //darboux vector

                        quaternion omega_plus;
                        omega_plus.value = omega.value + restDarboux[i].value;  //delta Omega with - omega_0
                        omega.value -= restDarboux[i].value;                    //delta Omega with + omega_0
                        if (math.lengthsq(omega.value) > math.lengthsq(omega_plus.value))
                            omega = omega_plus;

                        float3 dlambda = (omega.value.xyz - compliances.y * lambda.xyz) / new float3(compliances.y + invRotationalMasses[particleIndex] + rigidbodyAngularW + BurstMath.epsilon);
                        lambda.xyz += dlambda;

                        //discrete Darboux vector does not have vanishing scalar part
                        quaternion dlambdaQ = new quaternion(dlambda[0], dlambda[1], dlambda[2], 0);

                        quaternion orientDelta = orientationDeltas[particleIndex];
                        orientDelta.value += math.mul(predictedRotation, dlambdaQ).value * invRotationalMasses[particleIndex];
                        orientationDeltas[particleIndex] = orientDelta;
                        orientationCounts[particleIndex]++;

                        if (rigidbodyIndex >= 0)
                            rigidbodies[rigidbodyIndex].ApplyDeltaQuaternion(predictedRotation, math.mul(orientations[particleIndex], dlambdaQ).value * -rigidbodyAngularW, ref angularRbDelta, deltaTime);
                    }

                    if (rigidbodyIndex >= 0)
                    {
                        float4 impulse = correction / deltaTime;

                        rigidbodies[rigidbodyIndex].ApplyImpulse(-inertialFrame.frame.TransformVector(impulse) * 1, worldPinOffset, ref linearRbDelta, ref angularRbDelta);
                        rigidbodyLinearDeltas[rigidbodyIndex] = linearRbDelta;
                        rigidbodyAngularDeltas[rigidbodyIndex] = angularRbDelta;
                    }

                    lambdas[i] = lambda;
                }
            }
        }

        [BurstCompile]
        public struct ApplyPinConstraintsBatchJob : IJob
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] public float sorFactor;

            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> positions;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<int> counts;

            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<quaternion> orientations;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<quaternion> orientationDeltas;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<int> orientationCounts;

            [ReadOnly] public int activeConstraintCount;

            public void Execute()
            {
                for (int i = 0; i < activeConstraintCount; ++i)
                {
                    int p1 = particleIndices[i];

                    if (counts[p1] > 0)
                    {
                        positions[p1] += deltas[p1] * sorFactor / counts[p1];
                        deltas[p1] = float4.zero;
                        counts[p1] = 0;
                    }

                    if (orientationCounts[p1] > 0)
                    {
                        quaternion q = orientations[p1];
                        q.value += orientationDeltas[p1].value * sorFactor / orientationCounts[p1];
                        orientations[p1] = math.normalize(q);

                        orientationDeltas[p1] = new quaternion(0, 0, 0, 0);
                        orientationCounts[p1] = 0;
                    }
                }
            }
        }
    }
}
#endif