#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;

namespace Obi
{
    public class BurstDistanceConstraints : BurstConstraintsImpl<BurstDistanceConstraintsBatch>
    {
        public BurstDistanceConstraints(BurstSolverImpl solver) : base(solver, Oni.ConstraintType.Distance)
        {
        }

        public override IConstraintsBatchImpl CreateConstraintsBatch()
        {
            var dataBatch = new BurstDistanceConstraintsBatch(this);
            batches.Add(dataBatch);
            return dataBatch;
        }

        public override void RemoveBatch(IConstraintsBatchImpl batch)
        {
            batches.Remove(batch as BurstDistanceConstraintsBatch);
            batch.Destroy();
        }
    }
}
#endif