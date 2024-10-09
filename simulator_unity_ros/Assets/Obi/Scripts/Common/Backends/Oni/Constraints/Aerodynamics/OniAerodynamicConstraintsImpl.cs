#if (OBI_ONI_SUPPORTED)
using System;
using System.Collections;
using System.Runtime.InteropServices;

namespace Obi
{
    public class OniAerodynamicConstraintsImpl : OniConstraintsImpl
    {

        public OniAerodynamicConstraintsImpl(OniSolverImpl solver) : base(solver, Oni.ConstraintType.Aerodynamics)
        {
        }

        public override IConstraintsBatchImpl CreateConstraintsBatch()
        {
            var batch =  new OniAerodynamicConstraintsBatchImpl(this);
            Oni.AddBatch(((OniSolverImpl)solver).oniSolver, ((OniConstraintsBatchImpl)batch).oniBatch);
            return batch;
        }

        public override void RemoveBatch(IConstraintsBatchImpl batch)
        {
            Oni.RemoveBatch(((OniSolverImpl)solver).oniSolver, ((OniConstraintsBatchImpl)batch).oniBatch);
        }
    }
}
#endif