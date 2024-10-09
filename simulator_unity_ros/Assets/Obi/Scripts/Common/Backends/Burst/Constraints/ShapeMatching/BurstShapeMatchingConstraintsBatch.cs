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
    public class BurstShapeMatchingConstraintsBatch : BurstConstraintsBatchImpl, IShapeMatchingConstraintsBatchImpl
    {
        private NativeArray<int> firstIndex;
        private NativeArray<int> numIndices;
        private NativeArray<int> explicitGroup;
        private NativeArray<float> shapeMaterialParameters;
        private NativeArray<float4>     restComs;
        private NativeArray<float4>     coms;
        private NativeArray<quaternion> constraintOrientations;

        private NativeArray<float4x4> Aqq;
        private NativeArray<float4x4> deformation;

        public BurstShapeMatchingConstraintsBatch(BurstShapeMatchingConstraints constraints)
        {
            m_Constraints = constraints;
            m_ConstraintType = Oni.ConstraintType.ShapeMatching;
        }

        public void SetShapeMatchingConstraints(ObiNativeIntList particleIndices,
                                                ObiNativeIntList firstIndex,
                                                ObiNativeIntList numIndices,
                                                ObiNativeIntList explicitGroup,
                                                ObiNativeFloatList shapeMaterialParameters,
                                                ObiNativeVector4List restComs,
                                                ObiNativeVector4List coms,
                                                ObiNativeQuaternionList constraintOrientations,
                                                ObiNativeFloatList lambdas,
                                                int count)
        {
            this.particleIndices = particleIndices.AsNativeArray<int>();
            this.firstIndex = firstIndex.AsNativeArray<int>();
            this.numIndices = numIndices.AsNativeArray<int>();
            this.explicitGroup = explicitGroup.AsNativeArray<int>();
            this.shapeMaterialParameters = shapeMaterialParameters.AsNativeArray<float>();
            this.restComs = restComs.AsNativeArray<float4>();
            this.coms = coms.AsNativeArray<float4>();
            this.constraintOrientations = constraintOrientations.AsNativeArray<quaternion>();

            if (Aqq.IsCreated)
                Aqq.Dispose();
            if (deformation.IsCreated)
                deformation.Dispose();

            Aqq = new NativeArray<float4x4>(count,Allocator.Persistent);

            deformation = new NativeArray<float4x4>(count, Allocator.Persistent);
            for (int i = 0; i < count; ++i)
                deformation[i] = float4x4.identity;

            m_ConstraintCount = count;
        }

        public override void Destroy()
        {
            if (Aqq.IsCreated)
                Aqq.Dispose();
            if (deformation.IsCreated)
                deformation.Dispose();
        }

        public override JobHandle Initialize(JobHandle inputDeps, float deltaTime)
        {
            return inputDeps;
        }

        public override JobHandle Evaluate(JobHandle inputDeps, float deltaTime)
        {
            var projectConstraints = new ShapeMatchingConstraintsBatchJob()
            {
                particleIndices = particleIndices,
                firstIndex = firstIndex,
                numIndices = numIndices,
                explicitGroup = explicitGroup,
                shapeMaterialParameters = shapeMaterialParameters,
                restComs = restComs,
                coms = coms,
                constraintOrientations = constraintOrientations,
                Aqq = Aqq,
                deformation = deformation,

                positions = solverImplementation.positions,
                restPositions = solverImplementation.restPositions,
                orientations = solverImplementation.orientations,
                restOrientations = solverImplementation.restOrientations,
                invMasses = solverImplementation.invMasses,
                invRotationalMasses = solverImplementation.invRotationalMasses,
                invInertiaTensors = solverImplementation.invInertiaTensors,

                deltas = solverImplementation.positionDeltas,
                counts = solverImplementation.positionConstraintCounts,

                deltaTime = deltaTime
            };

            return projectConstraints.Schedule(m_ConstraintCount, 4, inputDeps);
        }

        public override JobHandle Apply(JobHandle inputDeps, float deltaTime)
        {
            var parameters = solverAbstraction.GetConstraintParameters(m_ConstraintType);

            var applyConstraints = new ApplyShapeMatchingConstraintsBatchJob()
            {
                particleIndices = particleIndices,
                firstIndex = firstIndex,
                numIndices = numIndices,

                positions = solverImplementation.positions,
                deltas = solverImplementation.positionDeltas,
                counts = solverImplementation.positionConstraintCounts,

                sorFactor = parameters.SORFactor
            };

            return applyConstraints.Schedule(m_ConstraintCount, 8, inputDeps);
        }

        public void CalculateRestShapeMatching()
        {
            var deps = ((BurstSolverImpl)constraints.solver).RecalculateInertiaTensors(new JobHandle());

            var calculateRest = new ShapeMatchingCalculateRestJob()
            {
                particleIndices = particleIndices,
                firstIndex = firstIndex,
                numIndices = numIndices,
                restComs = restComs,
                coms = coms,
                Aqq = Aqq,
                deformation = deformation,

                restPositions = solverAbstraction.restPositions.AsNativeArray<float4>(),
                restOrientations = solverAbstraction.restOrientations.AsNativeArray<quaternion>(),
                invMasses = solverAbstraction.invMasses.AsNativeArray<float>(),
                invInertiaTensors = solverAbstraction.invInertiaTensors.AsNativeArray<float4>(),
            };

            calculateRest.Schedule(numIndices.Length, 64, deps).Complete();
        }

        protected static void RecalculateRestData(int i,
                                                  ref NativeArray<int> particleIndices,
                                                  ref NativeArray<int> firstIndex,
                                                  ref NativeArray<float4> restComs,
                                                  ref NativeArray<float4x4> Aqq,
                                                  ref NativeArray<float4x4> deformation,
                                                  ref NativeArray<int> numIndices,
                                                  ref NativeArray<float> invMasses,
                                                  ref NativeArray<float4> restPositions,
                                                  ref NativeArray<quaternion> restOrientations,
                                                  ref NativeArray<float4> invInertiaTensors)
        {
            int k = 0;
            float maximumMass = 10000;

            // initialize rest center of mass and shape matrix:
            restComs[i] = float4.zero;
            Aqq[i] = float4x4.zero;

            float4 restCom = float4.zero;
            float4x4 _Aqq = float4x4.zero, _Rqq = float4x4.zero;

            // calculate rest center of mass, shape mass and Aqq matrix.
            for (int j = 0; j < numIndices[i]; ++j)
            {
                k = particleIndices[firstIndex[i] + j];

                float mass = maximumMass;
                if (invMasses[k] > 1.0f / maximumMass)
                    mass = 1.0f / invMasses[k];

                restCom += restPositions[k] * mass;

                float4x4 particleR = restOrientations[k].toMatrix();
                particleR[3][3] = 0;

                _Rqq += math.mul(particleR,
                                 math.mul(math.rcp(invInertiaTensors[k] + new float4(BurstMath.epsilon)).asDiagonal(),
                                 math.transpose(particleR))
                                );

                float4 restPosition = restPositions[k];
                restPosition[3] = 0;

                _Aqq += mass * BurstMath.multrnsp4(restPosition, restPosition);

            }


            if (restCom[3] < BurstMath.epsilon)
                return;

            restCom.xyz /= restCom[3];
            restComs[i] = restCom;

            restCom[3] = 0;
            _Aqq -= restComs[i][3] * BurstMath.multrnsp4(restCom, restCom);
            _Aqq[3][3] = 1; // so that the determinant is never 0 due to all-zeros row/column.

            Aqq[i] = math.inverse(_Rqq + math.mul(deformation[i], math.mul(_Aqq, math.transpose(deformation[i]))));
        }

        [BurstCompile]
        public struct ShapeMatchingCalculateRestJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] public NativeArray<int> firstIndex;
            [ReadOnly] public NativeArray<int> numIndices;
            public NativeArray<float4> restComs;
            [ReadOnly] public NativeArray<float4> coms;

            public NativeArray<float4x4> Aqq;
            [ReadOnly] public NativeArray<float4x4> deformation;

            [ReadOnly] public NativeArray<float4> restPositions;
            [ReadOnly] public NativeArray<quaternion> restOrientations;
            [ReadOnly] public NativeArray<float> invMasses;
            [ReadOnly] public NativeArray<float4> invInertiaTensors;

            public void Execute(int i)
            {
                RecalculateRestData(i,
                                    ref particleIndices,
                                    ref firstIndex,
                                    ref restComs,
                                    ref Aqq,
                                    ref deformation,
                                    ref numIndices,
                                    ref invMasses,
                                    ref restPositions,
                                    ref restOrientations,
                                    ref invInertiaTensors);
            }
        }

        [BurstCompile]
        public struct ShapeMatchingConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] public NativeArray<int> firstIndex;
            [ReadOnly] public NativeArray<int> numIndices;
            [ReadOnly] public NativeArray<int> explicitGroup;
            [ReadOnly] public NativeArray<float> shapeMaterialParameters;
             public NativeArray<float4> restComs;
             public NativeArray<float4> coms;
             public NativeArray<quaternion> constraintOrientations;

            public NativeArray<float4x4> Aqq;
            public NativeArray<float4x4> deformation;

            [ReadOnly] public NativeArray<float4> positions;
            [ReadOnly] public NativeArray<float4> restPositions;

            [ReadOnly] public NativeArray<quaternion> restOrientations;
            [ReadOnly] public NativeArray<float> invMasses;
            [ReadOnly] public NativeArray<float> invRotationalMasses;
            [ReadOnly] public NativeArray<float4> invInertiaTensors;

            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<quaternion> orientations;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> counts;

            [ReadOnly] public float deltaTime;

            public void Execute(int i)
            {
                int k = 0;
                float maximumMass = 10000;

                coms[i] = float4.zero;
                float4x4 Apq = float4x4.zero, Rpq = float4x4.zero;

                // calculate shape mass, center of mass, and moment matrix:
                for (int j = 0; j < numIndices[i]; ++j)
                {
                    k = particleIndices[firstIndex[i] + j];

                    float mass = maximumMass;
                    if (invMasses[k] > 1.0f / maximumMass)
                        mass = 1.0f / invMasses[k];

                    coms[i] += positions[k] * mass;

                    float4x4 particleR = orientations[k].toMatrix();
                    float4x4 particleRT = restOrientations[k].toMatrix();
                    particleR[3][3] = 0;
                    particleRT[3][3] = 0;

                    Rpq += math.mul(particleR,
                                    math.mul(math.rcp(invInertiaTensors[k] + new float4(BurstMath.epsilon)).asDiagonal(),
                                    math.transpose(particleRT))
                                    );

                    float4 restPosition = restPositions[k];
                    restPosition[3] = 0;

                    Apq += mass * BurstMath.multrnsp4(positions[k], restPosition);
                }

                if (restComs[i][3] < BurstMath.epsilon)
                    return;

                coms[i] /= restComs[i][3];

                // subtract global shape moment:
                float4 restCom = restComs[i];
                restCom[3] = 0;

                Apq -= restComs[i][3] * BurstMath.multrnsp4(coms[i], restCom);

                // calculate optimal transform including plastic deformation:
                float4x4 Apq_def = Rpq + math.mul(Apq , math.transpose(deformation[i]));

                // extract rotation from transform matrix, using warmstarting and few iterations:
                constraintOrientations[i] = BurstMath.ExtractRotation(Apq_def, constraintOrientations[i], 2);

                // finally, obtain rotation matrix:
                float4x4 R = constraintOrientations[i].toMatrix();
                R[3][3] = 0;

                // calculate particle orientations:
                if (explicitGroup[i] > 0)
                {
                    // if the group is explicit, set the orientation for all particles:
                    for (int j = 0; j < numIndices[i]; ++j)
                    {
                        k = particleIndices[firstIndex[i] + j];
                        orientations[k] = math.mul(constraintOrientations[i], restOrientations[k]);
                    }
                }
                else
                {
                    // set orientation of center particle only:
                    int centerIndex = particleIndices[firstIndex[i]];
                    orientations[centerIndex] = math.mul(constraintOrientations[i], restOrientations[centerIndex]);
                }

                // calculate and accumulate particle goal positions:
                float4 goal;
                float4x4 transform = math.mul(R,deformation[i]);
                for (int j = 0; j < numIndices[i]; ++j)
                {
                    k = particleIndices[firstIndex[i] + j];
                    goal = coms[i] + math.mul(transform, restPositions[k] - restComs[i]);
                    deltas[k] += (goal - positions[k]) * shapeMaterialParameters[i * 5];
                    counts[k]++;
                }

                // update plastic deformation:
                float plastic_yield = shapeMaterialParameters[i * 5 + 1];
                float plastic_creep = shapeMaterialParameters[i * 5 + 2];
                float plastic_recovery = shapeMaterialParameters[i * 5 + 3];
                float max_deform = shapeMaterialParameters[i * 5 + 4];

                // if we are allowed to absorb deformation:
                if (plastic_creep > 0)
                {
                    R[3][3] = 1;
                    Apq_def[3][3] = 1;

                    // get scale matrix (A = RS so S = Rt * A) and its deviation from the identity matrix:
                    float4x4 deform_matrix = math.mul(math.transpose(R), math.mul(Apq_def, Aqq[i])) - float4x4.identity;

                    // if the amount of deformation exceeds the yield threshold:
                    float norm = deform_matrix.frobeniusNorm();
                    if (norm > plastic_yield)
                    {
                        // deform the shape permanently:
                        deformation[i] = math.mul(float4x4.identity + plastic_creep * deform_matrix, deformation[i]);

                        // clamp deformation so that it does not exceed a percentage;
                        deform_matrix = deformation[i] - float4x4.identity;
                        norm = deform_matrix.frobeniusNorm();
                        if (norm > max_deform)
                        {
                            deformation[i] = float4x4.identity + max_deform * deform_matrix / norm;
                        }

                        // if we cannot recover from plastic deformation, recalculate rest shape now:
                        if (plastic_recovery == 0)
                            RecalculateRestData(i,
                                    ref particleIndices,
                                    ref firstIndex,
                                    ref restComs,
                                    ref Aqq,
                                    ref deformation,
                                    ref numIndices,
                                    ref invMasses,
                                    ref restPositions,
                                    ref restOrientations,
                                    ref invInertiaTensors);
                    }
                }

                // if we can recover from plastic deformation, lerp towards non-deformed shape and recalculate rest shape:
                if (plastic_recovery > 0)
                {
                    deformation[i] += (float4x4.identity - deformation[i]) * math.min(plastic_recovery * deltaTime, 1.0f);
                    RecalculateRestData(i,
                                    ref particleIndices,
                                    ref firstIndex,
                                    ref restComs,
                                    ref Aqq,
                                    ref deformation,
                                    ref numIndices,
                                    ref invMasses,
                                    ref restPositions,
                                    ref restOrientations,
                                    ref invInertiaTensors);
                }
            }
        }

        [BurstCompile]
        public struct ApplyShapeMatchingConstraintsBatchJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> particleIndices;
            [ReadOnly] public NativeArray<int> firstIndex;
            [ReadOnly] public NativeArray<int> numIndices;
            [ReadOnly] public float sorFactor;

            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> positions;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
            [NativeDisableContainerSafetyRestriction] [NativeDisableParallelForRestriction] public NativeArray<int> counts;

            public void Execute(int i)
            {
                int first = firstIndex[i];
                int last = first + numIndices[i];

                for (int k = first; k < last; ++k)
                {
                    int p = particleIndices[k];
                    if (counts[p] > 0)
                    {
                        positions[p] += deltas[p] * sorFactor / counts[p];
                        deltas[p] = float4.zero;
                        counts[p] = 0;
                    }
                }
            }
        }
    }
}
#endif