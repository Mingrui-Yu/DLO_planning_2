using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

namespace Obi
{
    [Serializable]
    public class ObiSkinConstraintsBatch : ObiConstraintsBatch
    {
        protected ISkinConstraintsBatchImpl m_BatchImpl;   


        /// <summary>
        /// skin constraint anchor points, in solver space.
        /// </summary>
        [HideInInspector] public ObiNativeVector4List skinPoints = new ObiNativeVector4List();             

        /// <summary>
        /// normal vector for each skin constraint, in solver space.
        /// </summary>
        [HideInInspector] public ObiNativeVector4List skinNormals = new ObiNativeVector4List();            

        /// <summary>
        ///  3 floats per constraint: skin radius, backstop sphere radius, and backstop sphere distance.
        /// </summary>
        [HideInInspector] public ObiNativeFloatList skinRadiiBackstop = new ObiNativeFloatList();          

        /// <summary>
        /// one compliance value per skin constraint.
        /// </summary>
        [HideInInspector] public ObiNativeFloatList skinCompliance = new ObiNativeFloatList();              

        public override Oni.ConstraintType constraintType
        {
            get { return Oni.ConstraintType.Skin; }
        }

        public override IConstraintsBatchImpl implementation
        {
            get { return m_BatchImpl; }
        }

        public ObiSkinConstraintsBatch(ObiSkinConstraintsData constraints = null) : base()
        {
        }

        public void AddConstraint(int index, Vector4 point, Vector4 normal, float radius, float collisionRadius, float backstop, float stiffness)
        {
            RegisterConstraint();

            particleIndices.Add(index);
            skinPoints.Add(point);
            skinNormals.Add(normal);
            skinRadiiBackstop.Add(radius);
            skinRadiiBackstop.Add(collisionRadius);
            skinRadiiBackstop.Add(backstop);
            skinCompliance.Add(stiffness);
        }

        public override void Clear()
        {
            base.Clear();
            particleIndices.Clear();
            skinPoints.Clear();
            skinNormals.Clear();
            skinRadiiBackstop.Clear();
            skinCompliance.Clear();
        }

        public override void GetParticlesInvolved(int index, List<int> particles)
        {
            particles.Add(particleIndices[index]);
        }

        protected override void SwapConstraints(int sourceIndex, int destIndex)
        {
            particleIndices.Swap(sourceIndex, destIndex);
            skinPoints.Swap(sourceIndex, destIndex);
            skinNormals.Swap(sourceIndex, destIndex);
            skinRadiiBackstop.Swap(sourceIndex * 3, destIndex * 3);
            skinRadiiBackstop.Swap(sourceIndex * 3+1, destIndex * 3+1);
            skinRadiiBackstop.Swap(sourceIndex * 3+2, destIndex * 3+2);
            skinCompliance.Swap(sourceIndex, destIndex);
        }

        public override void Merge(ObiActor actor, IObiConstraintsBatch other)
        {
            var batch = other as ObiSkinConstraintsBatch;

            if (batch != null)
            {

                particleIndices.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                skinPoints.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                skinNormals.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                skinRadiiBackstop.ResizeUninitialized((m_ActiveConstraintCount + batch.activeConstraintCount) * 3);
                skinCompliance.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                lambdas.ResizeInitialized(m_ActiveConstraintCount + batch.activeConstraintCount);

                skinPoints.CopyFrom(batch.skinPoints, 0, m_ActiveConstraintCount, batch.activeConstraintCount);
                skinNormals.CopyFrom(batch.skinNormals, 0, m_ActiveConstraintCount, batch.activeConstraintCount);
                skinCompliance.CopyFrom(batch.skinCompliance, 0, m_ActiveConstraintCount, batch.activeConstraintCount);
                skinRadiiBackstop.CopyFrom(batch.skinRadiiBackstop, 0, m_ActiveConstraintCount*3, batch.activeConstraintCount*3);

                for (int i = 0; i < batch.activeConstraintCount; ++i)
                    particleIndices[m_ActiveConstraintCount + i] = actor.solverIndices[batch.particleIndices[i]];

                base.Merge(actor, other);
            }
        }

        public override void AddToSolver(ObiSolver solver)
        {
            // Create distance constraints batch directly.
            m_BatchImpl = solver.implementation.CreateConstraintsBatch(constraintType) as ISkinConstraintsBatchImpl;

            if (m_BatchImpl != null)
                m_BatchImpl.SetSkinConstraints(particleIndices, skinPoints, skinNormals, skinRadiiBackstop, skinCompliance, lambdas, m_ActiveConstraintCount);
        }

        public override void RemoveFromSolver(ObiSolver solver)
        {
            //Remove batch:
            solver.implementation.DestroyConstraintsBatch(m_BatchImpl as IConstraintsBatchImpl);
        }

    }
}
