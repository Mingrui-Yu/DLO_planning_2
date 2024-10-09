#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;

namespace Obi
{
    public class BurstTetherConstraints : BurstConstraintsImpl<BurstTetherConstraintsBatch>
    {
        public BurstTetherConstraints(BurstSolverImpl solver) : base(solver, Oni.ConstraintType.Distance)
        {
        }

        public override IConstraintsBatchImpl CreateConstraintsBatch()
        {
            var dataBatch = new BurstTetherConstraintsBatch(this);
            batches.Add(dataBatch);
            return dataBatch;
        }

        public override void RemoveBatch(IConstraintsBatchImpl batch)
        {
            batches.Remove(batch as BurstTetherConstraintsBatch);
            batch.Destroy();
        }
    }
}
#endif