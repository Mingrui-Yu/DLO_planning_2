#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;

namespace Obi
{
    public class BurstStitchConstraints : BurstConstraintsImpl<BurstStitchConstraintsBatch>
    {
        public BurstStitchConstraints(BurstSolverImpl solver) : base(solver, Oni.ConstraintType.Stitch)
        {
        }

        public override IConstraintsBatchImpl CreateConstraintsBatch()
        {
            var dataBatch = new BurstStitchConstraintsBatch(this);
            batches.Add(dataBatch);
            return dataBatch;
        }

        public override void RemoveBatch(IConstraintsBatchImpl batch)
        {
            batches.Remove(batch as BurstStitchConstraintsBatch);
            batch.Destroy();
        }
    }
}
#endif