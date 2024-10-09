#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;

namespace Obi
{
    public class BurstPinConstraints : BurstConstraintsImpl<BurstPinConstraintsBatch>
    {
        public BurstPinConstraints(BurstSolverImpl solver) : base(solver, Oni.ConstraintType.Pin)
        {
        }

        public override IConstraintsBatchImpl CreateConstraintsBatch()
        {
            var dataBatch = new BurstPinConstraintsBatch(this);
            batches.Add(dataBatch);
            return dataBatch;
        }

        public override void RemoveBatch(IConstraintsBatchImpl batch)
        {
            batches.Remove(batch as BurstPinConstraintsBatch);
            batch.Destroy();
        }
    }
}
#endif