using UnityEngine;
using System.Collections;

namespace Obi
{
    public interface ITetherConstraintsBatchImpl : IConstraintsBatchImpl
    {
        void SetTetherConstraints(ObiNativeIntList particleIndices, ObiNativeVector2List maxLengthScale, ObiNativeFloatList stiffnesses, ObiNativeFloatList lambdas, int count);
    }
}
