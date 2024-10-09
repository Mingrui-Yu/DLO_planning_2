using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

namespace Obi
{
    [Serializable]
    public class ObiStretchShearConstraintsBatch : ObiConstraintsBatch, IStructuralConstraintBatch
    {
        protected IStretchShearConstraintsBatchImpl m_BatchImpl;  

        /// <summary>
        /// index of particle orientation for each constraint.
        /// </summary>
        [HideInInspector] public ObiNativeIntList orientationIndices = new ObiNativeIntList();                      

        /// <summary>
        /// rest distance for each constraint.
        /// </summary>
        [HideInInspector] public ObiNativeFloatList restLengths = new ObiNativeFloatList();                        

        /// <summary>
        /// rest orientation for each constraint.
        /// </summary>
        [HideInInspector] public ObiNativeQuaternionList restOrientations = new ObiNativeQuaternionList();         

        /// <summary>
        /// 3 compliance values per constraint, one for each local axis (x,y,z).
        /// </summary>
        [HideInInspector] public ObiNativeVector3List stiffnesses = new ObiNativeVector3List();                     

        public override Oni.ConstraintType constraintType
        {
            get { return Oni.ConstraintType.StretchShear; }
        }

        public override IConstraintsBatchImpl implementation
        {
            get { return m_BatchImpl; }
        }

        public ObiStretchShearConstraintsBatch(ObiStretchShearConstraintsData constraints = null) : base()
        {
        }

        public void AddConstraint(Vector2Int indices, int orientationIndex, float restLength, Quaternion restOrientation)
        {
            RegisterConstraint();

            particleIndices.Add(indices[0]);
            particleIndices.Add(indices[1]);
            orientationIndices.Add(orientationIndex);
            restLengths.Add(restLength);
            restOrientations.Add(restOrientation);
            stiffnesses.Add(Vector3.zero);
        }

        public override void Clear()
        {
            base.Clear();
            particleIndices.Clear();
            orientationIndices.Clear();
            restLengths.Clear();
            restOrientations.Clear();
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
            return new ParticlePair(particleIndices[index * 2], particleIndices[index * 2 + 1]);
        }

        public override void GetParticlesInvolved(int index, List<int> particles)
        {
            particles.Add(particleIndices[index * 2]);
            particles.Add(particleIndices[index * 2 + 1]);
        }

        protected override void SwapConstraints(int sourceIndex, int destIndex)
        {
            particleIndices.Swap(sourceIndex * 2 , destIndex * 2);
            particleIndices.Swap(sourceIndex * 2 + 1, destIndex * 2 + 1);
            orientationIndices.Swap(sourceIndex, destIndex);
            restLengths.Swap(sourceIndex, destIndex);
            restOrientations.Swap(sourceIndex, destIndex);
            stiffnesses.Swap(sourceIndex, destIndex);
        }

        public override void Merge(ObiActor actor, IObiConstraintsBatch other)
        {
            var batch = other as ObiStretchShearConstraintsBatch;
            var user = actor as IStretchShearConstraintsUser;

            if (batch != null && user != null)
            {
                if (!user.stretchShearConstraintsEnabled)
                    return;

                particleIndices.ResizeUninitialized((m_ActiveConstraintCount + batch.activeConstraintCount) * 2);
                orientationIndices.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                restLengths.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                restOrientations.ResizeUninitialized(lambdas.count + batch.activeConstraintCount);
                stiffnesses.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                lambdas.ResizeInitialized((m_ActiveConstraintCount + batch.activeConstraintCount) * 3);

                restLengths.CopyFrom(batch.restLengths, 0, m_ActiveConstraintCount, batch.activeConstraintCount);
                restOrientations.CopyFrom(batch.restOrientations, 0, m_ActiveConstraintCount, batch.activeConstraintCount);
                stiffnesses.CopyReplicate(new Vector3(user.stretchCompliance, user.shear1Compliance, user.shear2Compliance), m_ActiveConstraintCount, batch.activeConstraintCount);

                for (int i = 0; i < batch.activeConstraintCount * 2; ++i)
                    particleIndices[m_ActiveConstraintCount * 2 + i] = actor.solverIndices[batch.particleIndices[i]];

                for (int i = 0; i < batch.activeConstraintCount; ++i)
                    orientationIndices[m_ActiveConstraintCount + i] = actor.solverIndices[batch.orientationIndices[i]];

                base.Merge(actor, other);
            }
        }

        public override void AddToSolver(ObiSolver solver)
        {
            // Create distance constraints batch directly.
            m_BatchImpl = solver.implementation.CreateConstraintsBatch(constraintType) as IStretchShearConstraintsBatchImpl;

            if (m_BatchImpl != null)
                m_BatchImpl.SetStretchShearConstraints(particleIndices, orientationIndices, restLengths, restOrientations, stiffnesses, lambdas, m_ActiveConstraintCount);
        }

        public override void RemoveFromSolver(ObiSolver solver)
        {
            //Remove batch:
            solver.implementation.DestroyConstraintsBatch(m_BatchImpl as IConstraintsBatchImpl);
        }

    }
}
