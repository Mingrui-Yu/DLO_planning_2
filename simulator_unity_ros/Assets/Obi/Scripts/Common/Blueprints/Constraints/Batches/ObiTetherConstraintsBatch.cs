using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

namespace Obi
{
    [Serializable]
    public class ObiTetherConstraintsBatch : ObiConstraintsBatch
    {
        protected ITetherConstraintsBatchImpl m_BatchImpl;   

        /// <summary>
        /// 2 floats per constraint: maximum length and tether scale.
        /// </summary>
        [HideInInspector] public ObiNativeVector2List maxLengthsScales = new ObiNativeVector2List();    

        /// <summary>
        /// compliance value for each constraint.
        /// </summary>
        [HideInInspector] public ObiNativeFloatList stiffnesses = new ObiNativeFloatList();    

        public override Oni.ConstraintType constraintType
        {
            get { return Oni.ConstraintType.Tether; }
        }

        public override IConstraintsBatchImpl implementation
        {
            get { return m_BatchImpl; }
        }

        public ObiTetherConstraintsBatch(ObiTetherConstraintsData constraints = null) : base()
        {
        }

        public void AddConstraint(Vector2Int indices, float maxLength, float scale)
        {
            RegisterConstraint();

            particleIndices.Add(indices[0]);
            particleIndices.Add(indices[1]);
            maxLengthsScales.Add(new Vector2(maxLength, scale));
            stiffnesses.Add(0);
        }

        public override void Clear()
        {
            base.Clear();
            particleIndices.Clear();
            maxLengthsScales.Clear();
            stiffnesses.Clear();
        }

        public override void GetParticlesInvolved(int index, List<int> particles)
        {
            particles.Add(particleIndices[index * 2]);
            particles.Add(particleIndices[index * 2 + 1]);
        }

        protected override void SwapConstraints(int sourceIndex, int destIndex)
        {
            particleIndices.Swap(sourceIndex * 2, destIndex * 2);
            particleIndices.Swap(sourceIndex * 2 + 1, destIndex * 2 + 1);
            maxLengthsScales.Swap(sourceIndex, destIndex);
            stiffnesses.Swap(sourceIndex, destIndex);
        }

        public override void Merge(ObiActor actor, IObiConstraintsBatch other)
        {
            var batch = other as ObiTetherConstraintsBatch;
            var user = actor as ITetherConstraintsUser;

            if (batch != null && user != null)
            {
                if (!user.tetherConstraintsEnabled)
                    return;

                particleIndices.ResizeUninitialized((m_ActiveConstraintCount + batch.activeConstraintCount) * 2);
                maxLengthsScales.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                stiffnesses.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                lambdas.ResizeInitialized(m_ActiveConstraintCount + batch.activeConstraintCount);

                stiffnesses.CopyReplicate(user.tetherCompliance, m_ActiveConstraintCount, batch.activeConstraintCount);

                for (int i = 0; i < batch.activeConstraintCount * 2; ++i)
                    particleIndices[m_ActiveConstraintCount * 2 + i] = actor.solverIndices[batch.particleIndices[i]];

                for (int i = 0; i < batch.activeConstraintCount; ++i)
                    maxLengthsScales[m_ActiveConstraintCount + i] = new Vector2(batch.maxLengthsScales[i].x, user.tetherScale);

                base.Merge(actor, other);
            }
        }

        public override void AddToSolver(ObiSolver solver)
        {
            // Create distance constraints batch directly.
            m_BatchImpl = solver.implementation.CreateConstraintsBatch(constraintType) as ITetherConstraintsBatchImpl;

            if (m_BatchImpl != null)
                m_BatchImpl.SetTetherConstraints(particleIndices, maxLengthsScales, stiffnesses, lambdas, m_ActiveConstraintCount);
        }

        public override void RemoveFromSolver(ObiSolver solver)
        {
            //Remove batch:
            solver.implementation.DestroyConstraintsBatch(m_BatchImpl as IConstraintsBatchImpl);
        }

        /*public override void AddToSolver(ObiSolver solver)
        {
            // create and add the implementation:
            if (m_Constraints != null && m_Constraints.implementation != null)
            {
                m_BatchImpl = m_Constraints.implementation.CreateConstraintsBatch();
            }

            if (m_BatchImpl != null)
            {
                lambdas.Clear();
                for (int i = 0; i < stiffnesses.count; i++)
                {
                    //particleIndices[i * 2] = constraints.GetActor().solverIndices[m_Source.particleIndices[i * 2]];
                    //particleIndices[i * 2 + 1] = constraints.GetActor().solverIndices[m_Source.particleIndices[i * 2 + 1]];
                    lambdas.Add(0);
                }

                m_BatchImpl.SetTetherConstraints(particleIndices, maxLengthsScales, stiffnesses, lambdas, m_ConstraintCount);
                m_BatchImpl.SetActiveConstraints(m_ActiveConstraintCount);
            }
            
        }

        public override void RemoveFromSolver(ObiSolver solver)
        {
            if (m_Constraints != null && m_Constraints.implementation != null)
                m_Constraints.implementation.RemoveBatch(m_BatchImpl);
        }*/

        public void SetParameters(float compliance, float scale)
        {
            for (int i = 0; i < stiffnesses.count; i++)
            {
                stiffnesses[i] = compliance;
                maxLengthsScales[i] = new Vector2(maxLengthsScales[i].x, scale);
            }
        }

    }
}
