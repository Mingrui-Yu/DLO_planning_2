using UnityEngine;
using System.Collections;

namespace Obi
{

    public interface IStructuralConstraintBatch
    {
        float GetRestLength(int index);
        void SetRestLength(int index, float restLength);
        ParticlePair GetParticleIndices(int index);
    }
}
