#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;

namespace Obi
{
    public class BurstSkinConstraints : BurstConstraintsImpl<BurstSkinConstraintsBatch>
    {
        public BurstSkinConstraints(BurstSolverImpl solver) : base(solver, Oni.ConstraintType.Skin)
        {
        }

        public override IConstraintsBatchImpl CreateConstraintsBatch()
        {
            var dataBatch = new BurstSkinConstraintsBatch(this);
            batches.Add(dataBatch);
            return dataBatch;
        }

        public override void RemoveBatch(IConstraintsBatchImpl batch)
        {
            batches.Remove(batch as BurstSkinConstraintsBatch);
            batch.Destroy();
        }
    }
}
#endif