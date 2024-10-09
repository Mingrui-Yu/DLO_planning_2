using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

namespace Obi
{
    [Serializable]
    public class ObiDistanceConstraintsBatch : ObiConstraintsBatch, IStructuralConstraintBatch
    {
        [NonSerialized] protected IDistanceConstraintsBatchImpl m_BatchImpl; 

        /// <summary>
        /// Rest distance for each individual constraint.
        /// </summary>
        [HideInInspector] public ObiNativeFloatList restLengths = new ObiNativeFloatList();

        /// <summary>
        /// 2 values for each constraint: compliance and slack.
        /// </summary>
        [HideInInspector] public ObiNativeVector2List stiffnesses = new ObiNativeVector2List();            

        public override Oni.ConstraintType constraintType
        {
            get { return Oni.ConstraintType.Distance; }
        }

        public override IConstraintsBatchImpl implementation
        {
            get { return m_BatchImpl; }
        }

        public ObiDistanceConstraintsBatch(int a = 0) 
        { 
        }

        public void AddConstraint(Vector2Int indices, float restLength)
        {
            RegisterConstraint();

            particleIndices.Add(indices[0]);
            particleIndices.Add(indices[1]);
            restLengths.Add(restLength);
            stiffnesses.Add(Vector2.zero);
        }

        public override void Clear()
        {
            base.Clear();
            restLengths.Clear();
            stiffnesses.Clear();
        }

        public float GetRestLength(int index)
        {
            return restLengths[index];
        }

        public void SetRestLength(int index, float restLength)
        {
            restLengths[index] = restLength;
        }

        public ParticlePair GetParticleIndices(int index)
        {
            return new ParticlePair(particleIndices[index * 2],particleIndices[index * 2 + 1]);
        }

        public override void GetParticlesInvolved(int index, List<int> particles)
        {
            particles.Add(particleIndices[index * 2]);
            particles.Add(particleIndices[index * 2 + 1]);
        }

        protected override void CopyConstraint(ObiConstraintsBatch batch, int constraintIndex)
        {
            if (batch is ObiDistanceConstraintsBatch)
            {
                var db = batch as ObiDistanceConstraintsBatch;
                RegisterConstraint();
                particleIndices.Add(batch.particleIndices[constraintIndex * 2]);
                particleIndices.Add(batch.particleIndices[constraintIndex * 2 + 1]);
                restLengths.Add(db.restLengths[constraintIndex]);
                stiffnesses.Add(db.stiffnesses[constraintIndex]);
                ActivateConstraint(constraintCount - 1);
            }
        }

        protected override void SwapConstraints(int sourceIndex, int destIndex)
        {
            particleIndices.Swap(sourceIndex * 2, destIndex * 2);
            particleIndices.Swap(sourceIndex * 2 + 1, destIndex * 2 + 1);
            restLengths.Swap(sourceIndex, destIndex);
            stiffnesses.Swap(sourceIndex, destIndex);
        }

        public override void Merge(ObiActor actor, IObiConstraintsBatch other)
        {
            var batch = other as ObiDistanceConstraintsBatch;
            var user = actor as IDistanceConstraintsUser;

            if (batch != null && user != null)
            {
                if (!user.distanceConstraintsEnabled)
                    return;

                particleIndices.ResizeUninitialized((m_ActiveConstraintCount + batch.activeConstraintCount) * 2);
                restLengths.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                stiffnesses.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                lambdas.ResizeInitialized(m_ActiveConstraintCount + batch.activeConstraintCount);

                for (int i = 0; i < batch.activeConstraintCount * 2; ++i)
                    particleIndices[m_ActiveConstraintCount * 2 + i] = actor.solverIndices[batch.particleIndices[i]];

                for (int i = 0; i < batch.activeConstraintCount; ++i)
                {
                    float restLength = batch.restLengths[i] * user.stretchingScale;
                    restLengths[m_ActiveConstraintCount + i] = restLength; // TODO: use nativelist methods?
                    stiffnesses[m_ActiveConstraintCount + i] = new Vector2(user.stretchCompliance, user.maxCompression * restLength);
                }

                base.Merge(actor, other);
            }
        }

        public override void AddToSolver(ObiSolver solver)
        {
            // Create distance constraints batch directly.
            m_BatchImpl = solver.implementation.CreateConstraintsBatch(constraintType) as IDistanceConstraintsBatchImpl;

            if (m_BatchImpl != null)
                m_BatchImpl.SetDistanceConstraints(particleIndices, restLengths, stiffnesses, lambdas, m_ActiveConstraintCount);
            
        }

        public override void RemoveFromSolver(ObiSolver solver)
        {
            //Remove batch:
            solver.implementation.DestroyConstraintsBatch(m_BatchImpl as IConstraintsBatchImpl);
        }

    }
}
