using UnityEngine;
using System.Collections;

namespace Obi
{
    public interface IStitchConstraintsBatchImpl : IConstraintsBatchImpl
    {
        void SetStitchConstraints(ObiNativeIntList particleIndices, ObiNativeFloatList stiffnesses, ObiNativeFloatList lambdas, int count);
    }
}
