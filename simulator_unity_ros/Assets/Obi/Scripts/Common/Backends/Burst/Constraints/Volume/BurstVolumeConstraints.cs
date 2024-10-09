#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;

namespace Obi
{
    public class BurstVolumeConstraints : BurstConstraintsImpl<BurstVolumeConstraintsBatch>
    {
        public BurstVolumeConstraints(BurstSolverImpl solver) : base(solver, Oni.ConstraintType.Volume)
        {
        }

        public override IConstraintsBatchImpl CreateConstraintsBatch()
        {
            var dataBatch = new BurstVolumeConstraintsBatch(this);
            batches.Add(dataBatch);
            return dataBatch;
        }

        public override void RemoveBatch(IConstraintsBatchImpl batch)
        {
            batches.Remove(batch as BurstVolumeConstraintsBatch);
            batch.Destroy();
        }
    }
}
#endif