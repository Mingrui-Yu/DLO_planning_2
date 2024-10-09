using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

namespace Obi
{
    [Serializable]
    public class ObiBendConstraintsBatch : ObiConstraintsBatch
    {
        protected IBendConstraintsBatchImpl m_BatchImpl;   

        /// <summary>
        /// one float per constraint: the rest bend distance.
        /// </summary>
        [HideInInspector] public ObiNativeFloatList restBends = new ObiNativeFloatList();                

        /// <summary>
        /// two floats per constraint: max bending and compliance.
        /// </summary>
        [HideInInspector] public ObiNativeVector2List bendingStiffnesses = new ObiNativeVector2List();    

        public override Oni.ConstraintType constraintType
        {
            get { return Oni.ConstraintType.Bending; }
        }

        public override IConstraintsBatchImpl implementation
        {
            get { return m_BatchImpl; }
        }

        public ObiBendConstraintsBatch(ObiBendConstraintsData constraints = null) : base()
        {
        }

        public override void Merge(ObiActor actor, IObiConstraintsBatch other)
        {
            var batch = other as ObiBendConstraintsBatch;
            var user = actor as IBendConstraintsUser;

            if (batch != null && user != null)
            {
                if (!user.bendConstraintsEnabled)
                    return;

                particleIndices.ResizeUninitialized((m_ActiveConstraintCount + batch.activeConstraintCount) * 3);
                restBends.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                bendingStiffnesses.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                lambdas.ResizeInitialized(m_ActiveConstraintCount + batch.activeConstraintCount);

                restBends.CopyFrom(batch.restBends, 0, m_ActiveConstraintCount, batch.activeConstraintCount);
                bendingStiffnesses.CopyReplicate(new Vector2(user.maxBending, user.bendCompliance), m_ActiveConstraintCount, batch.activeConstraintCount);

                for (int i = 0; i < batch.activeConstraintCount * 3; ++i)
                    particleIndices[m_ActiveConstraintCount * 3 + i] = actor.solverIndices[batch.particleIndices[i]];

                base.Merge(actor, other);
            }
        }

        public void AddConstraint(Vector3Int indices, float restBend)
        {
            RegisterConstraint();

            particleIndices.Add(indices[0]);
            particleIndices.Add(indices[1]);
            particleIndices.Add(indices[2]);
            restBends.Add(restBend);
            bendingStiffnesses.Add(Vector2.zero);
        }

        public override void Clear()
        {
            base.Clear();
            restBends.Clear();
            bendingStiffnesses.Clear();
        }

        public override void GetParticlesInvolved(int index, List<int> particles)
        {
            particles.Add(particleIndices[index * 3]);
            particles.Add(particleIndices[index * 3 + 1]);
            particles.Add(particleIndices[index * 3 + 2]);
        }

        protected override void SwapConstraints(int sourceIndex, int destIndex)
        {
            particleIndices.Swap(sourceIndex * 3, destIndex * 3);
            particleIndices.Swap(sourceIndex * 3 + 1 , destIndex * 3 + 1);
            particleIndices.Swap(sourceIndex * 3 + 2, destIndex * 3 + 2);
            restBends.Swap(sourceIndex, destIndex);
            bendingStiffnesses.Swap(sourceIndex, destIndex);
        }

        public override void AddToSolver(ObiSolver solver)
        {
            // Create distance constraints batch directly.
            m_BatchImpl = solver.implementation.CreateConstraintsBatch(constraintType) as IBendConstraintsBatchImpl;

            if (m_BatchImpl != null)
                m_BatchImpl.SetBendConstraints(particleIndices, restBends, bendingStiffnesses, lambdas, m_ActiveConstraintCount);
            
        }

        public override void RemoveFromSolver(ObiSolver solver)
        {
            //Remove batch:
            solver.implementation.DestroyConstraintsBatch(m_BatchImpl as IConstraintsBatchImpl);
        }
    }
}
