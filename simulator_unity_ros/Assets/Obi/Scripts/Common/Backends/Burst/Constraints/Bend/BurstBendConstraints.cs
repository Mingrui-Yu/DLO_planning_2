#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;

namespace Obi
{
    public class BurstBendConstraints : BurstConstraintsImpl<BurstBendConstraintsBatch>
    {
        public BurstBendConstraints(BurstSolverImpl solver) : base(solver, Oni.ConstraintType.Bending)
        {
        }

        public override IConstraintsBatchImpl CreateConstraintsBatch()
        {
            var dataBatch = new BurstBendConstraintsBatch(this);
            batches.Add(dataBatch);
            return dataBatch;
        }

        public override void RemoveBatch(IConstraintsBatchImpl batch)
        {
            batches.Remove(batch as BurstBendConstraintsBatch);
            batch.Destroy();
        }
    }
}
#endif