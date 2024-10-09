#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using UnityEngine;
using Unity.Burst;
using Unity.Jobs;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using System.Collections;

namespace Obi
{
    public abstract class BurstConstraintsBatchImpl : IConstraintsBatchImpl
    {
        protected IBurstConstraintsImpl m_Constraints;
        protected Oni.ConstraintType m_ConstraintType;

        protected bool m_Enabled = true;
        protected int m_ConstraintCount = 0;

        public Oni.ConstraintType constraintType
        {
            get { return m_ConstraintType; }
        }

        public bool enabled
        {
            set
            {
                if (m_Enabled != value)
                    m_Enabled = value;
            }
            get { return m_Enabled; }
        }

        public IConstraints constraints
        {
            get { return m_Constraints; }
        }

        public ObiSolver solverAbstraction
        {
            get { return ((BurstSolverImpl)m_Constraints.solver).abstraction; }
        }

        public BurstSolverImpl solverImplementation
        {
            get { return (BurstSolverImpl)m_Constraints.solver; }
        }

        protected NativeArray<int> particleIndices;
        protected NativeArray<float> lambdas;

        public virtual JobHandle Initialize(JobHandle inputDeps, float deltaTime)
        {
            if (lambdas.IsCreated)
            {
                // no need for jobs here, memclear is faster and we don't pay scheduling overhead.
                unsafe
                {
                    UnsafeUtility.MemClear(NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(lambdas),
                                           lambdas.Length * UnsafeUtility.SizeOf<float>());
                }
            }
            return inputDeps;
        }

        // implemented by concrete constraint subclasses.
        public abstract JobHandle Evaluate(JobHandle inputDeps, float deltaTime);
        public abstract JobHandle Apply(JobHandle inputDeps, float deltaTime);

        public virtual void Destroy()
        {
            // clean resources allocated by the batch, no need for a default implementation.
        }

        public void SetConstraintCount(int constraintCount)
        {
            m_ConstraintCount = constraintCount;
        }
        public int GetConstraintCount()
        {
            return m_ConstraintCount;
        }

        public static void ApplyPositionDelta(int particleIndex, float sorFactor, ref NativeArray<float4> positions, ref NativeArray<float4> deltas, ref NativeArray<int> counts)
        {
            if (counts[particleIndex] > 0)
            {
                positions[particleIndex] += deltas[particleIndex] * sorFactor / counts[particleIndex];
                deltas[particleIndex] = float4.zero;
                counts[particleIndex] = 0;
            }
        }

        public static void ApplyOrientationDelta(int particleIndex, float sorFactor, ref NativeArray<quaternion> orientations, ref NativeArray<quaternion> deltas, ref NativeArray<int> counts)
        {
            if (counts[particleIndex] > 0)
            {
                quaternion q = orientations[particleIndex];
                q.value += deltas[particleIndex].value * sorFactor / counts[particleIndex];
                orientations[particleIndex] = math.normalize(q);

                deltas[particleIndex] = new quaternion(0, 0, 0, 0);
                counts[particleIndex] = 0;
            }
        }
    }
}
#endif