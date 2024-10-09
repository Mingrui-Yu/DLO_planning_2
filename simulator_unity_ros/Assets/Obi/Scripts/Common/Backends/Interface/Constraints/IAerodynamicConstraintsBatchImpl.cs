using UnityEngine;
using System.Collections;

namespace Obi
{
    public interface IAerodynamicConstraintsBatchImpl : IConstraintsBatchImpl
    {
        void SetAerodynamicConstraints(ObiNativeIntList particleIndices, ObiNativeFloatList aerodynamicCoeffs, int count);
    }
}
