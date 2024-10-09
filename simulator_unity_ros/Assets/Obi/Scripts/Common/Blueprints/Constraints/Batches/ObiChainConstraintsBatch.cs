using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

namespace Obi
{
    [Serializable]
    public class ObiChainConstraintsBatch : ObiConstraintsBatch
    {
        protected IChainConstraintsBatchImpl m_BatchImpl;  

        /// <summary>
        /// index of the first particle for each constraint.
        /// </summary>
        [HideInInspector] public ObiNativeIntList firstParticle = new ObiNativeIntList();       

        /// <summary>
        /// number of particles for each constraint.
        /// </summary>
        [HideInInspector] public ObiNativeIntList numParticles = new ObiNativeIntList();          

        /// <summary>
        /// min/max lenghts for each constraint.
        /// </summary>
        [HideInInspector] public ObiNativeVector2List lengths = new ObiNativeVector2List();  

        public override Oni.ConstraintType constraintType
        {
            get { return Oni.ConstraintType.Chain; }
        }

        public override IConstraintsBatchImpl implementation
        {
            get { return m_BatchImpl; }
        }

        public ObiChainConstraintsBatch(ObiChainConstraintsData constraints = null) : base()
        {
        }

        public void AddConstraint(int[] indices, float restLength, float stretchStiffness, float compressionStiffness)
        {
            RegisterConstraint();

            firstParticle.Add((int)particleIndices.count);
            numParticles.Add((int)indices.Length);
            particleIndices.AddRange(indices);
            lengths.Add(new Vector2(restLength, restLength));
        }

        public override void Clear()
        {
            base.Clear();
            particleIndices.Clear();
            firstParticle.Clear();
            numParticles.Clear();
            lengths.Clear();
        }

        public override void GetParticlesInvolved(int index, List<int> particles)
        {
            //TODO.
        }

        protected override void SwapConstraints(int sourceIndex, int destIndex)
        {
            firstParticle.Swap(sourceIndex, destIndex);
            numParticles.Swap(sourceIndex, destIndex);
            lengths.Swap(sourceIndex, destIndex);
        }

        public override void Merge(ObiActor actor, IObiConstraintsBatch other)
        {
            var batch = other as ObiChainConstraintsBatch;
            var user = actor as IChainConstraintsUser;

            if (batch != null && user != null)
            {
                if (!user.chainConstraintsEnabled)
                    return;

                int initialIndexCount = particleIndices.count;

                int numActiveIndices = 0;
                for (int i = 0; i < batch.activeConstraintCount; ++i)
                    numActiveIndices += batch.numParticles[i];

                particleIndices.ResizeUninitialized(initialIndexCount + numActiveIndices);
                firstParticle.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                numParticles.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                lengths.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                lambdas.ResizeInitialized(m_ActiveConstraintCount + batch.activeConstraintCount);

                numParticles.CopyFrom(batch.numParticles, 0, m_ActiveConstraintCount, batch.activeConstraintCount);

                for (int i = 0; i < numActiveIndices; ++i)
                    particleIndices[initialIndexCount + i] = actor.solverIndices[batch.particleIndices[i]];

                for (int i = 0; i < batch.activeConstraintCount; ++i)
                {
                    firstParticle[m_ActiveConstraintCount + i] = batch.firstParticle[i] + initialIndexCount;
                    lengths[m_ActiveConstraintCount + i] = new Vector2(batch.lengths[i].y * user.tightness, batch.lengths[i].y);
                }


                base.Merge(actor, other);
            }
        }

        public override void AddToSolver(ObiSolver solver)
        {
            // Create distance constraints batch directly.
            m_BatchImpl = solver.implementation.CreateConstraintsBatch(constraintType) as IChainConstraintsBatchImpl;

            if (m_BatchImpl != null)
                m_BatchImpl.SetChainConstraints(particleIndices, lengths, firstParticle, numParticles, m_ActiveConstraintCount);
            
        }

        public override void RemoveFromSolver(ObiSolver solver)
        {
            //Remove batch:
            solver.implementation.DestroyConstraintsBatch(m_BatchImpl as IConstraintsBatchImpl);
        }
    }
}
