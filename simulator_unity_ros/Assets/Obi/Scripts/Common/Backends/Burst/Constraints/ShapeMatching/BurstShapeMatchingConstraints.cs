#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;

namespace Obi
{
    public class BurstShapeMatchingConstraints : BurstConstraintsImpl<BurstShapeMatchingConstraintsBatch>
    {
        public BurstShapeMatchingConstraints(BurstSolverImpl solver) : base(solver, Oni.ConstraintType.ShapeMatching)
        {
        }

        public override IConstraintsBatchImpl CreateConstraintsBatch()
        {
            var dataBatch = new BurstShapeMatchingConstraintsBatch(this);
            batches.Add(dataBatch);
            return dataBatch;
        }

        public override void RemoveBatch(IConstraintsBatchImpl batch)
        {
            batches.Remove(batch as BurstShapeMatchingConstraintsBatch);
            batch.Destroy();
        }
    }
}
#endif