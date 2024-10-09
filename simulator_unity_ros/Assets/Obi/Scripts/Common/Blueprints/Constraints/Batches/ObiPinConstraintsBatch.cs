using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    [Serializable]
    public class ObiPinConstraintsBatch : ObiConstraintsBatch
    {
        protected IPinConstraintsBatchImpl m_BatchImpl;  

        /// <summary>
        /// for each constraint, handle of the pinned collider.
        /// </summary>
        [HideInInspector] public List<ObiColliderHandle> pinBodies = new List<ObiColliderHandle>();                        

        /// <summary>
        /// index of the pinned collider in the collider world.
        /// </summary>
        [HideInInspector] public ObiNativeIntList colliderIndices = new ObiNativeIntList();

        /// <summary>
        /// Pin position expressed in the attachment's local space.
        /// </summary>
        [HideInInspector] public ObiNativeVector4List offsets = new ObiNativeVector4List();                        

        /// <summary>
        /// Rest Darboux vector for each constraint.
        /// </summary>
        [HideInInspector] public ObiNativeQuaternionList restDarbouxVectors = new ObiNativeQuaternionList();        

        /// <summary>
        /// Compliances of pin constraits. 2 float per constraint (positional and rotational compliance).
        /// </summary>
        [HideInInspector] public ObiNativeFloatList stiffnesses = new ObiNativeFloatList();                        

        /// <summary>
        /// One float per constraint: break threshold.
        /// </summary>
        [HideInInspector] public ObiNativeFloatList breakThresholds = new ObiNativeFloatList();                     

        public override Oni.ConstraintType constraintType
        {
            get { return Oni.ConstraintType.Pin; }
        }

        public override IConstraintsBatchImpl implementation
        {
            get { return m_BatchImpl; }
        }

        public ObiPinConstraintsBatch(ObiPinConstraintsData constraints = null) : base()
        {
        }

        public void AddConstraint(int solverIndex, ObiColliderBase body, Vector3 offset, Quaternion restDarboux, float linearCompliance, float rotationalCompliance, float breakThreshold)
        {
            RegisterConstraint();

            particleIndices.Add(solverIndex);
            pinBodies.Add(body != null ? body.Handle : new ObiColliderHandle());
            colliderIndices.Add(body != null ? body.Handle.index : -1);
            offsets.Add(offset);
            restDarbouxVectors.Add(restDarboux);
            stiffnesses.Add(linearCompliance);
            stiffnesses.Add(rotationalCompliance);
            breakThresholds.Add(breakThreshold);
        }

        public override void Clear()
        {
            base.Clear();
            particleIndices.Clear();
            pinBodies.Clear();
            colliderIndices.Clear();
            offsets.Clear();
            restDarbouxVectors.Clear();
            stiffnesses.Clear();
        }

        public override void GetParticlesInvolved(int index, List<int> particles)
        {
            particles.Add(particleIndices[index]);
        }

        protected override void SwapConstraints(int sourceIndex, int destIndex)
        {
            particleIndices.Swap(sourceIndex, destIndex);
            pinBodies.Swap(sourceIndex, destIndex);
            colliderIndices.Swap(sourceIndex, destIndex);
            offsets.Swap(sourceIndex, destIndex);
            restDarbouxVectors.Swap(sourceIndex, destIndex);
            stiffnesses.Swap(sourceIndex * 2, destIndex * 2);
            stiffnesses.Swap(sourceIndex * 2 + 1, destIndex * 2 + 1);
        }

        public override void Merge(ObiActor actor, IObiConstraintsBatch other)
        {
            var batch = other as ObiPinConstraintsBatch;

            if (batch != null)
            {

                particleIndices.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);

                colliderIndices.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                offsets.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                restDarbouxVectors.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                stiffnesses.ResizeUninitialized((m_ActiveConstraintCount + batch.activeConstraintCount) * 2);
                breakThresholds.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                lambdas.ResizeInitialized((m_ActiveConstraintCount + batch.activeConstraintCount) * 4);

                offsets.CopyFrom(batch.offsets, 0, m_ActiveConstraintCount, batch.activeConstraintCount);
                restDarbouxVectors.CopyFrom(batch.restDarbouxVectors, 0, m_ActiveConstraintCount, batch.activeConstraintCount);
                stiffnesses.CopyFrom(batch.stiffnesses, 0, m_ActiveConstraintCount * 2, batch.activeConstraintCount * 2);
                breakThresholds.CopyFrom(batch.breakThresholds, 0, m_ActiveConstraintCount, batch.activeConstraintCount);

                for (int i = 0; i < batch.activeConstraintCount; ++i)
                {
                    particleIndices[m_ActiveConstraintCount + i] = batch.particleIndices[i];
                    colliderIndices[m_ActiveConstraintCount + i] = batch.pinBodies[i] != null ? batch.pinBodies[i].index : -1;
                }

                base.Merge(actor, other);
            }
        }

        public override void AddToSolver(ObiSolver solver)
        {
            if (solver != null && solver.implementation != null)
            {
                m_BatchImpl = solver.implementation.CreateConstraintsBatch(constraintType) as IPinConstraintsBatchImpl;

                if (m_BatchImpl != null)
                    m_BatchImpl.SetPinConstraints(particleIndices, colliderIndices, offsets, restDarbouxVectors, stiffnesses, lambdas, m_ActiveConstraintCount);
            }
        }

        public override void RemoveFromSolver(ObiSolver solver)
        {
            if (solver != null && solver.implementation != null)
                solver.implementation.DestroyConstraintsBatch(m_BatchImpl as IConstraintsBatchImpl);
        }

    }
}
