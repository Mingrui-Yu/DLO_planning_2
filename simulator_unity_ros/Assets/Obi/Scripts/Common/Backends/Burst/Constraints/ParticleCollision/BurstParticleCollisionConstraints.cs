#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;
using Unity.Jobs;

namespace Obi
{
    public class BurstParticleCollisionConstraints : BurstConstraintsImpl<BurstParticleCollisionConstraintsBatch>
    {
        public BurstParticleCollisionConstraints(BurstSolverImpl solver) : base(solver, Oni.ConstraintType.ParticleCollision)
        {
        }

        public override IConstraintsBatchImpl CreateConstraintsBatch()
        {
            var dataBatch = new BurstParticleCollisionConstraintsBatch(this);
            batches.Add(dataBatch);
            return dataBatch;
        }

        public override void RemoveBatch(IConstraintsBatchImpl batch)
        {
            batches.Remove(batch as BurstParticleCollisionConstraintsBatch);
            batch.Destroy();
        }

        public override int GetConstraintCount()
        {
            return ((BurstSolverImpl)solver).particleContacts.Length;
        }
    }
}
#endif