#if (OBI_ONI_SUPPORTED)
using System;
using System.Collections;
using System.Runtime.InteropServices;

namespace Obi
{
    public class OniStretchShearConstraintsBatchImpl : OniConstraintsBatchImpl, IStretchShearConstraintsBatchImpl
    {
        public OniStretchShearConstraintsBatchImpl(OniStretchShearConstraintsImpl constraints) : base(constraints, Oni.ConstraintType.StretchShear)
        {
        }

        public void SetStretchShearConstraints(ObiNativeIntList particleIndices, ObiNativeIntList orientationIndices, ObiNativeFloatList restLengths, ObiNativeQuaternionList restOrientations, ObiNativeVector3List stiffnesses, ObiNativeFloatList lambdas, int count)
        {
            Oni.SetStretchShearConstraints(oniBatch, particleIndices.GetIntPtr(), orientationIndices.GetIntPtr(), restLengths.GetIntPtr(), restOrientations.GetIntPtr(), stiffnesses.GetIntPtr(), lambdas.GetIntPtr(), count); 
        }
    }
}
#endif