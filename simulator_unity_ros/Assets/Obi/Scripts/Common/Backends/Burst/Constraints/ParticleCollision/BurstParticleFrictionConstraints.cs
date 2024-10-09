#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;

namespace Obi
{
    public class BurstParticleFrictionConstraints : BurstConstraintsImpl<BurstParticleFrictionConstraintsBatch>
    {
        public BurstParticleFrictionConstraints(BurstSolverImpl solver) : base(solver, Oni.ConstraintType.ParticleFriction)
        {
        }

        public override IConstraintsBatchImpl CreateConstraintsBatch()
        {
            var dataBatch = new BurstParticleFrictionConstraintsBatch(this);
            batches.Add(dataBatch);
            return dataBatch;
        }

        public override void RemoveBatch(IConstraintsBatchImpl batch)
        {
            batches.Remove(batch as BurstParticleFrictionConstraintsBatch);
            batch.Destroy();
        }

        public override int GetConstraintCount()
        {
            return ((BurstSolverImpl)solver).particleContacts.Length;
        }
    }
}
#endif