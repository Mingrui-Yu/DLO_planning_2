#if (OBI_ONI_SUPPORTED)
using System;
using System.Collections;
using System.Runtime.InteropServices;

namespace Obi
{
    public interface IOniConstraintsImpl : IConstraints
    {
        IConstraintsBatchImpl CreateConstraintsBatch();
        void RemoveBatch(IConstraintsBatchImpl batch);
    }

    public abstract class OniConstraintsImpl : IOniConstraintsImpl
    {
        protected OniSolverImpl m_Solver;
        protected Oni.ConstraintType m_ConstraintType;

        public ISolverImpl solver
        {
            get { return m_Solver; }
        }

        public Oni.ConstraintType constraintType
        {
            get { return m_ConstraintType; }
        }

        public OniConstraintsImpl(OniSolverImpl solver, Oni.ConstraintType constraintType)
        {
            m_ConstraintType = constraintType;
            m_Solver = solver;
        }

        public abstract IConstraintsBatchImpl CreateConstraintsBatch();

        public abstract void RemoveBatch(IConstraintsBatchImpl batch);

        public int GetConstraintCount()
        {
            return Oni.GetConstraintCount(m_Solver.oniSolver, (int)m_ConstraintType);
        }
    }
}
#endif