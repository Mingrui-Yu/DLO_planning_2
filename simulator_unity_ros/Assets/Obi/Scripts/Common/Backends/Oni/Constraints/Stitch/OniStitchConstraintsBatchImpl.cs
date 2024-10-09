#if (OBI_ONI_SUPPORTED)
using System;
using System.Collections;
using System.Runtime.InteropServices;

namespace Obi
{
    public class OniStitchConstraintsBatchImpl : OniConstraintsBatchImpl, IStitchConstraintsBatchImpl
    {
        public OniStitchConstraintsBatchImpl(OniStitchConstraintsImpl constraints) : base(constraints, Oni.ConstraintType.Stitch)
        {
        }

        public void SetStitchConstraints(ObiNativeIntList particleIndices, ObiNativeFloatList stiffnesses, ObiNativeFloatList lambdas, int count)
        {
            Oni.SetStitchConstraints(oniBatch, particleIndices.GetIntPtr(), stiffnesses.GetIntPtr(), lambdas.GetIntPtr(), count);
        }
    }
}
#endif