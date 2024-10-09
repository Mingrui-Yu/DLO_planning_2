using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

namespace Obi
{
    [Serializable]
    public class ObiShapeMatchingConstraintsBatch : ObiConstraintsBatch
    {
        protected IShapeMatchingConstraintsBatchImpl m_BatchImpl;  

        /// <summary>
        /// index of the first particle in each constraint.
        /// </summary>
        public ObiNativeIntList firstIndex = new ObiNativeIntList();           

        /// <summary>
        /// amount of particles in each constraint.
        /// </summary>
        public ObiNativeIntList numIndices = new ObiNativeIntList();            

        /// <summary>
        /// whether the constraint is implicit (0) or explicit (>0).
        /// </summary>
        public ObiNativeIntList explicitGroup = new ObiNativeIntList();         

        /// <summary>
        /// 5 floats per constraint: stiffness, plastic yield, creep, recovery and max deformation.
        /// </summary>
        public ObiNativeFloatList materialParameters = new ObiNativeFloatList(); 


        /// <summary>
        /// rest center of mass for each constraint.
        /// </summary>
        public ObiNativeVector4List restComs = new ObiNativeVector4List();      

        /// <summary>
        /// current center of mass for each constraint.
        /// </summary>
        public ObiNativeVector4List coms = new ObiNativeVector4List();       

        /// <summary>
        /// current best-match orientation for each constraint.
        /// </summary>
        public ObiNativeQuaternionList orientations = new ObiNativeQuaternionList();   

        public override Oni.ConstraintType constraintType
        {
            get { return Oni.ConstraintType.ShapeMatching; }
        }

        public override IConstraintsBatchImpl implementation
        {
            get { return m_BatchImpl; }
        }

        public ObiShapeMatchingConstraintsBatch(ObiShapeMatchingConstraintsData constraints = null) : base()
        {
        }

        public void AddConstraint(int[] indices, bool isExplicit)
        {
            RegisterConstraint();

            firstIndex.Add((int)particleIndices.count);
            numIndices.Add((int)indices.Length);
            explicitGroup.Add(isExplicit ? 1 : 0);
            particleIndices.AddRange(indices);
            materialParameters.AddRange(new float[] { 1, 1, 1, 1, 1 });
        }

        public override void Clear()
        {
            base.Clear();
            firstIndex.Clear();
            numIndices.Clear();
            explicitGroup.Clear();
            particleIndices.Clear();
            materialParameters.Clear();
        }

        public override void GetParticlesInvolved(int index, List<int> particles)
        {
            int first = firstIndex[index];
            int num = numIndices[index];
            for (int i = first; i < first + num; ++i) 
                particles.Add(particleIndices[i]);
        }

        protected override void SwapConstraints(int sourceIndex, int destIndex)
        {
            firstIndex.Swap(sourceIndex, destIndex);
            numIndices.Swap(sourceIndex, destIndex);
            explicitGroup.Swap(sourceIndex, destIndex);

            for (int i = 0; i < 5; ++i)
                materialParameters.Swap(sourceIndex * 5 + i, destIndex * 5 + i);

            restComs.Swap(sourceIndex, destIndex);
            coms.Swap(sourceIndex, destIndex);
            orientations.Swap(sourceIndex, destIndex);
        }

        public override void Merge(ObiActor actor, IObiConstraintsBatch other)
        {
            var batch = other as ObiShapeMatchingConstraintsBatch;
            var user = actor as IShapeMatchingConstraintsUser;

            if (batch != null && user != null)
            {
                if (!user.shapeMatchingConstraintsEnabled)
                    return;

                int initialIndexCount = particleIndices.count;

                // shape matching constraint particle indices are not reordered when deactivating constraints,
                // so instead of using batch.activeConstraintCount, batch.constraintCount. We need all of them.
                int numActiveIndices = 0;
                for (int i = 0; i < batch.constraintCount; ++i) 
                    numActiveIndices += batch.numIndices[i];

                particleIndices.ResizeUninitialized(initialIndexCount + numActiveIndices);
                firstIndex.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                numIndices.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                explicitGroup.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                materialParameters.ResizeUninitialized((m_ActiveConstraintCount + batch.activeConstraintCount) * 5);

                restComs.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                coms.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                orientations.ResizeUninitialized(m_ActiveConstraintCount + batch.activeConstraintCount);
                lambdas.ResizeInitialized(m_ActiveConstraintCount + batch.activeConstraintCount);

                numIndices.CopyFrom(batch.numIndices, 0, m_ActiveConstraintCount, batch.activeConstraintCount);
                explicitGroup.CopyFrom(batch.explicitGroup, 0, m_ActiveConstraintCount, batch.activeConstraintCount);
                orientations.CopyReplicate(actor.actorLocalToSolverMatrix.rotation, m_ActiveConstraintCount, batch.activeConstraintCount);

                for (int i = 0; i < numActiveIndices; ++i)
                    particleIndices[initialIndexCount + i] = actor.solverIndices[batch.particleIndices[i]];

                for (int i = 0; i < batch.activeConstraintCount; ++i)
                {
                    firstIndex[m_ActiveConstraintCount + i] = batch.firstIndex[i] + initialIndexCount;
                    materialParameters[(m_ActiveConstraintCount + i) * 5] = user.deformationResistance;
                    materialParameters[(m_ActiveConstraintCount + i) * 5 + 1] = user.plasticYield;
                    materialParameters[(m_ActiveConstraintCount + i) * 5 + 2] = user.plasticCreep;
                    materialParameters[(m_ActiveConstraintCount + i) * 5 + 3] = user.plasticRecovery;
                    materialParameters[(m_ActiveConstraintCount + i) * 5 + 4] = user.maxDeformation;
                }

                base.Merge(actor, other);
            }
        }

        public override void AddToSolver(ObiSolver solver)
        {
            // Create distance constraints batch directly.
            m_BatchImpl = solver.implementation.CreateConstraintsBatch(constraintType) as IShapeMatchingConstraintsBatchImpl;

            if (m_BatchImpl != null)
            {
                m_BatchImpl.SetShapeMatchingConstraints(particleIndices, firstIndex, numIndices, explicitGroup,
                                                        materialParameters, restComs, coms, orientations, lambdas, m_ActiveConstraintCount);

                m_BatchImpl.CalculateRestShapeMatching();
            }
        }

        public override void RemoveFromSolver(ObiSolver solver)
        {
            //Remove batch:
            solver.implementation.DestroyConstraintsBatch(m_BatchImpl as IConstraintsBatchImpl);
        }

        public void RecalculateRestShapeMatching()
        {
            if (m_BatchImpl != null)
                m_BatchImpl.CalculateRestShapeMatching();
        }
    }
}
