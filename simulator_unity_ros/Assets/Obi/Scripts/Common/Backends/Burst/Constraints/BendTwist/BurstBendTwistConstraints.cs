#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;

namespace Obi
{
    public class BurstBendTwistConstraints : BurstConstraintsImpl<BurstBendTwistConstraintsBatch>
    {
        public BurstBendTwistConstraints(BurstSolverImpl solver) : base(solver, Oni.ConstraintType.BendTwist)
        {
        }

        public override IConstraintsBatchImpl CreateConstraintsBatch()
        {
            var dataBatch = new BurstBendTwistConstraintsBatch(this);
            batches.Add(dataBatch);
            return dataBatch;
        }

        public override void RemoveBatch(IConstraintsBatchImpl batch)
        {
            batches.Remove(batch as BurstBendTwistConstraintsBatch);
            batch.Destroy();
        }
    }
}
#endif