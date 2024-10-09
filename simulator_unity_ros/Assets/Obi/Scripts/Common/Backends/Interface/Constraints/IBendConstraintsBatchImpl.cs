using UnityEngine;
using System.Collections;

namespace Obi
{
    public interface IBendConstraintsBatchImpl : IConstraintsBatchImpl
    {
        void SetBendConstraints(ObiNativeIntList particleIndices, ObiNativeFloatList restBends, ObiNativeVector2List bendingStiffnesses, ObiNativeFloatList lambdas, int count);
    }
}
