#if (OBI_ONI_SUPPORTED)
using System;
using System.Collections;
using System.Runtime.InteropServices;

namespace Obi
{
    public class OniChainConstraintsBatchImpl : OniConstraintsBatchImpl, IChainConstraintsBatchImpl
    {
        public OniChainConstraintsBatchImpl(OniChainConstraintsImpl constraints) : base(constraints, Oni.ConstraintType.Chain)
        {
        }

        public void SetChainConstraints(ObiNativeIntList particleIndices, ObiNativeVector2List restLengths, ObiNativeIntList firstIndex, ObiNativeIntList numIndices, int count)
        {
            Oni.SetChainConstraints(oniBatch, particleIndices.GetIntPtr(), restLengths.GetIntPtr(), firstIndex.GetIntPtr(), numIndices.GetIntPtr(), count);
        }
    }
}
#endif