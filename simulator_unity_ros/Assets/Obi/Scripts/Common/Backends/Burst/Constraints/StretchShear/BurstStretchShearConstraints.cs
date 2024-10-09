#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;

namespace Obi
{
    public class BurstStretchShearConstraints : BurstConstraintsImpl<BurstStretchShearConstraintsBatch>
    {
        public BurstStretchShearConstraints(BurstSolverImpl solver) : base(solver, Oni.ConstraintType.StretchShear)
        {
        }

        public override IConstraintsBatchImpl CreateConstraintsBatch()
        {
            var dataBatch = new BurstStretchShearConstraintsBatch(this);
            batches.Add(dataBatch);
            return dataBatch;
        }

        public override void RemoveBatch(IConstraintsBatchImpl batch)
        {
            batches.Remove(batch as BurstStretchShearConstraintsBatch);
            batch.Destroy();
        }
    }
}
#endif