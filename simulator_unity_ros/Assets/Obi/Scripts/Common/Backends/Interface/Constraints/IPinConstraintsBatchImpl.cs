
using System.Collections.Generic;

namespace Obi
{
    public interface IPinConstraintsBatchImpl : IConstraintsBatchImpl
    {
        void SetPinConstraints(ObiNativeIntList particleIndices, ObiNativeIntList colliderIndices, ObiNativeVector4List offsets, ObiNativeQuaternionList restDarbouxVectors, ObiNativeFloatList stiffnesses, ObiNativeFloatList lambdas, int count);
    }
}
