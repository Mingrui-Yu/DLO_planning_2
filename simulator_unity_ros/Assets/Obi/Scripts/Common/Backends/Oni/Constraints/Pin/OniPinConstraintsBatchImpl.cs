#if (OBI_ONI_SUPPORTED)
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace Obi
{
    public class OniPinConstraintsBatchImpl : OniConstraintsBatchImpl, IPinConstraintsBatchImpl
    {
        public OniPinConstraintsBatchImpl(OniPinConstraintsImpl constraints) : base(constraints, Oni.ConstraintType.Pin)
        {
        }

        public void SetPinConstraints(ObiNativeIntList particleIndices, ObiNativeIntList colliderIndices, ObiNativeVector4List offsets, ObiNativeQuaternionList restDarbouxVectors, ObiNativeFloatList stiffnesses, ObiNativeFloatList lambdas, int count)
        {
            Oni.SetPinConstraints(oniBatch, particleIndices.GetIntPtr() ,offsets.GetIntPtr(), restDarbouxVectors.GetIntPtr(), colliderIndices.GetIntPtr(), stiffnesses.GetIntPtr(), lambdas.GetIntPtr(), count);
        }
    }
}
#endif