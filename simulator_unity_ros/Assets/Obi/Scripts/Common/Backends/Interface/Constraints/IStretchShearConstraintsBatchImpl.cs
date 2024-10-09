using UnityEngine;
using System.Collections;

namespace Obi
{
    public interface IStretchShearConstraintsBatchImpl : IConstraintsBatchImpl
    {
        void SetStretchShearConstraints(ObiNativeIntList particleIndices, ObiNativeIntList orientationIndices, ObiNativeFloatList restLengths, ObiNativeQuaternionList restOrientations, ObiNativeVector3List stiffnesses, ObiNativeFloatList lambdas, int count);
    }
}
