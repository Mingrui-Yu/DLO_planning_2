#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;

namespace Obi
{
    public class BurstColliderFrictionConstraints : BurstConstraintsImpl<BurstColliderFrictionConstraintsBatch>
    {
        public BurstColliderFrictionConstraints(BurstSolverImpl solver) : base(solver, Oni.ConstraintType.Friction)
        {
        }

        public override IConstraintsBatchImpl CreateConstraintsBatch()
        {
            var dataBatch = new BurstColliderFrictionConstraintsBatch(this);
            batches.Add(dataBatch);
            return dataBatch;
        }


        public override void RemoveBatch(IConstraintsBatchImpl batch)
        {
            batches.Remove(batch as BurstColliderFrictionConstraintsBatch);
            batch.Destroy();
        }

        public override int GetConstraintCount()
        {
            return ((BurstSolverImpl)solver).colliderContacts.Length;
        }
    }
}
#endif

