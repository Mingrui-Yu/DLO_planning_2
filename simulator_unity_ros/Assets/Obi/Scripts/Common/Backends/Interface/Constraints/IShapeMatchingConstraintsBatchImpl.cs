using UnityEngine;
using System.Collections;

namespace Obi
{
    public interface IShapeMatchingConstraintsBatchImpl : IConstraintsBatchImpl
    {
        void SetShapeMatchingConstraints(ObiNativeIntList particleIndices,
                                         ObiNativeIntList firstIndex,
                                         ObiNativeIntList numIndices,
                                         ObiNativeIntList explicitGroup,
                                         ObiNativeFloatList shapeMaterialParameters,
                                         ObiNativeVector4List restComs,
                                         ObiNativeVector4List coms,
                                         ObiNativeQuaternionList orientations,
                                         ObiNativeFloatList lambdas,
                                         int count);

        void CalculateRestShapeMatching();
    }
}
