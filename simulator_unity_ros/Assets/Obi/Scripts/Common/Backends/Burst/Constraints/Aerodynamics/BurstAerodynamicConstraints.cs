#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;

namespace Obi
{
    public class BurstAerodynamicConstraints : BurstConstraintsImpl<BurstAerodynamicConstraintsBatch>
    {
        public BurstAerodynamicConstraints(BurstSolverImpl solver) : base(solver, Oni.ConstraintType.Aerodynamics)
        {
        }

        public override IConstraintsBatchImpl CreateConstraintsBatch()
        {
            var dataBatch = new BurstAerodynamicConstraintsBatch(this);
            batches.Add(dataBatch);
            return dataBatch;
        }

        public override void RemoveBatch(IConstraintsBatchImpl batch)
        {
            batches.Remove(batch as BurstAerodynamicConstraintsBatch);
            batch.Destroy();
        }
    }
}
#endif
