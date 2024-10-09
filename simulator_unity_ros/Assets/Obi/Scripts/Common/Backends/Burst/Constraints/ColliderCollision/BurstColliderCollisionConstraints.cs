#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;

namespace Obi
{
    public class BurstColliderCollisionConstraints : BurstConstraintsImpl<BurstColliderCollisionConstraintsBatch>
    {
        public BurstColliderCollisionConstraints(BurstSolverImpl solver) : base(solver, Oni.ConstraintType.Collision)
        {
        }

        public override IConstraintsBatchImpl CreateConstraintsBatch()
        {
            var dataBatch = new BurstColliderCollisionConstraintsBatch(this);
            batches.Add(dataBatch);
            return dataBatch;
        }

        public override void RemoveBatch(IConstraintsBatchImpl batch)
        {
            batches.Remove(batch as BurstColliderCollisionConstraintsBatch);
            batch.Destroy();
        }

        public override int GetConstraintCount()
        {
            return ((BurstSolverImpl)solver).colliderContacts.Length;
        }
    }
}
#endif