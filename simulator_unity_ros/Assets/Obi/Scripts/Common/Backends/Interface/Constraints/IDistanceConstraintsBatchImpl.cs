using UnityEngine;
using System.Collections;

namespace Obi
{
    public interface IDistanceConstraintsBatchImpl : IConstraintsBatchImpl
    {
        void SetDistanceConstraints(ObiNativeIntList particleIndices, ObiNativeFloatList restLengths, ObiNativeVector2List stiffnesses, ObiNativeFloatList lambdas, int count);
    }
}
