using UnityEngine;
using System.Collections;

namespace Obi
{
    public interface ISkinConstraintsBatchImpl : IConstraintsBatchImpl
    {
        void SetSkinConstraints(ObiNativeIntList particleIndices, ObiNativeVector4List skinPoints, ObiNativeVector4List skinNormals, ObiNativeFloatList skinRadiiBackstop, ObiNativeFloatList skinCompliance, ObiNativeFloatList lambdas, int count);
    }
}
