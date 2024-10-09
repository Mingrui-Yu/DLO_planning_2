using UnityEngine;
using System.Collections;

namespace Obi
{
    public interface IBendTwistConstraintsBatchImpl : IConstraintsBatchImpl
    {
        void SetBendTwistConstraints(ObiNativeIntList orientationIndices, ObiNativeQuaternionList restDarboux, ObiNativeVector3List stiffnesses, ObiNativeFloatList lambdas, int count);
    }
}
