using UnityEngine;
using System.Collections;

namespace Obi
{
    public interface IChainConstraintsBatchImpl : IConstraintsBatchImpl
    {
        void SetChainConstraints(ObiNativeIntList particleIndices, ObiNativeVector2List restLengths, ObiNativeIntList firstIndex, ObiNativeIntList numIndex, int count);
    }
}
