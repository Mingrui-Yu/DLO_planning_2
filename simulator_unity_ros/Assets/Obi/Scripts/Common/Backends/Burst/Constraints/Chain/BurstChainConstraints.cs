#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;

namespace Obi
{
    public class BurstChainConstraints : BurstConstraintsImpl<BurstChainConstraintsBatch>
    {
        public BurstChainConstraints(BurstSolverImpl solver) : base(solver, Oni.ConstraintType.Chain)
        {
        }

        public override IConstraintsBatchImpl CreateConstraintsBatch()
        {
            var dataBatch = new BurstChainConstraintsBatch(this);
            batches.Add(dataBatch);
            return dataBatch;
        }

        public override void RemoveBatch(IConstraintsBatchImpl batch)
        {
            batches.Remove(batch as BurstChainConstraintsBatch);
            batch.Destroy();
        }
    }
}
#endif