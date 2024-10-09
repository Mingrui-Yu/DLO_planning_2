using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    public interface IObiConstraintsBatch
    {
        int constraintCount
        {
            get;
        }

        int activeConstraintCount
        {
            get;
            set;
        }

        int initialActiveConstraintCount
        {
            get;
            set;
        }

        Oni.ConstraintType constraintType
        {
            get;
        }

        IConstraintsBatchImpl implementation
        {
            get;
        }

        void AddToSolver(ObiSolver solver);
        void RemoveFromSolver(ObiSolver solver);

        void Merge(ObiActor actor, IObiConstraintsBatch other);

        bool DeactivateConstraint(int constraintIndex);
        bool ActivateConstraint(int constraintIndex);
        void DeactivateAllConstraints();

        void Clear();

        void GetParticlesInvolved(int index, List<int> particles);
        void ParticlesSwapped(int index, int newIndex);
    }

    public abstract class ObiConstraintsBatch : IObiConstraintsBatch
    {
        [HideInInspector] [SerializeField] protected List<int> m_IDs = new List<int>();
        [HideInInspector] [SerializeField] protected List<int> m_IDToIndex = new List<int>();         /**< maps from constraint ID to constraint index. When activating/deactivating constraints, their order changes. That makes this
                                                         map necessary. All active constraints are at the beginning of the constraint arrays, in the 0, activeConstraintCount index range.*/

        [HideInInspector] [SerializeField] protected int m_ConstraintCount = 0;
        [HideInInspector] [SerializeField] protected int m_ActiveConstraintCount = 0;
        [HideInInspector] [SerializeField] protected int m_InitialActiveConstraintCount = 0;

        [HideInInspector] public ObiNativeIntList particleIndices = new ObiNativeIntList();  /**< particle indices, amount of them per constraint can be variable. */
        [HideInInspector] public ObiNativeFloatList lambdas = new ObiNativeFloatList();      /**< constraint lambdas */

        public int constraintCount
        {
            get { return m_ConstraintCount; }
        }

        public int activeConstraintCount
        {
            get { return m_ActiveConstraintCount; }
            set { m_ActiveConstraintCount = value; }
        }

        public virtual int initialActiveConstraintCount
        {
            get { return m_InitialActiveConstraintCount; }
            set { m_InitialActiveConstraintCount = value; }
        }

        public abstract Oni.ConstraintType constraintType
        {
            get;
        }

        public abstract IConstraintsBatchImpl implementation
        {
            get;
        }

        // Merges a batch from a given actor with this one.
        public virtual void Merge(ObiActor actor, IObiConstraintsBatch other)
        {
            ObiConstraintsBatch batch = other as ObiConstraintsBatch;

            m_ConstraintCount += other.constraintCount;
            m_ActiveConstraintCount += other.activeConstraintCount;
            m_InitialActiveConstraintCount += other.initialActiveConstraintCount;
        }


        protected abstract void SwapConstraints(int sourceIndex, int destIndex);
        public abstract void GetParticlesInvolved(int index, List<int> particles);
        public abstract void AddToSolver(ObiSolver solver);
        public abstract void RemoveFromSolver(ObiSolver solver);

        protected virtual void CopyConstraint(ObiConstraintsBatch batch, int constraintIndex) { }

        private void InnerSwapConstraints(int sourceIndex, int destIndex)
        {
            m_IDToIndex[m_IDs[sourceIndex]] = destIndex;
            m_IDToIndex[m_IDs[destIndex]] = sourceIndex;
            m_IDs.Swap(sourceIndex, destIndex);
            SwapConstraints(sourceIndex, destIndex);
        }

        /**
         * Registers a new constraint. Call this before adding a new contraint to the batch, so that the constraint is given an ID 
         * and the amount of constraints increased.
         */
        protected void RegisterConstraint()
        {
            m_IDs.Add(m_ConstraintCount);
            m_IDToIndex.Add(m_ConstraintCount);
            m_ConstraintCount++;
        }

        public virtual void Clear()
        {
            m_ConstraintCount = 0;
            m_ActiveConstraintCount = 0;
            m_IDs.Clear();
            m_IDToIndex.Clear();
            particleIndices.Clear();
            lambdas.Clear();
        }

        /**
         * Given the id of a constraint, return its index in the constraint data arrays. Will return -1 if the constraint does not exist.
         */
        public int GetConstraintIndex(int constraintId)
        {
            if (constraintId < 0 || constraintId >= constraintCount)
                return -1;
            return m_IDToIndex[constraintId];
        }

        public bool IsConstraintActive(int index)
        {
            return index < m_ActiveConstraintCount;
        }

        public bool ActivateConstraint(int constraintIndex)
        {
            if (constraintIndex < m_ActiveConstraintCount)
                return false;

            InnerSwapConstraints(constraintIndex, m_ActiveConstraintCount);
            m_ActiveConstraintCount++;

            return true;
        }

        public bool DeactivateConstraint(int constraintIndex)
        {
            if (constraintIndex >= m_ActiveConstraintCount)
                return false;

            m_ActiveConstraintCount--;
            InnerSwapConstraints(constraintIndex, m_ActiveConstraintCount);

            return true;
        }

        public void DeactivateAllConstraints()
        {
            m_ActiveConstraintCount = 0;
        }

        // Swaps the constraint with the last one and reduces the amount of constraints by one.
        public void RemoveConstraint(int constraintIndex)
        {
            SwapConstraints(constraintIndex, constraintCount - 1);
            m_IDs.RemoveAt(constraintCount - 1);
            m_IDToIndex.RemoveAt(constraintCount - 1);

            m_ConstraintCount--;
            m_ActiveConstraintCount = Mathf.Min(m_ActiveConstraintCount, m_ConstraintCount); 
        }

        public void ParticlesSwapped(int index, int newIndex)
        {
            for (int i = 0; i < particleIndices.count; ++i)
            {
                if (particleIndices[i] == newIndex)
                    particleIndices[i] = index;
                else if (particleIndices[i] == index)
                    particleIndices[i] = newIndex;
            }
        }

    }
}
