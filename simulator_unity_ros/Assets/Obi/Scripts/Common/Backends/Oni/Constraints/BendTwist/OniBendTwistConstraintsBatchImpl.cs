#if (OBI_ONI_SUPPORTED)
using System;
using System.Collections;
using System.Runtime.InteropServices;

namespace Obi
{
    public class OniBendTwistConstraintsBatchImpl : OniConstraintsBatchImpl, IBendTwistConstraintsBatchImpl
    {
        public OniBendTwistConstraintsBatchImpl(OniBendTwistConstraintsImpl constraints) : base(constraints, Oni.ConstraintType.BendTwist)
        {
        }

        public void SetBendTwistConstraints(ObiNativeIntList orientationIndices, ObiNativeQuaternionList restOrientations, ObiNativeVector3List stiffnesses, ObiNativeFloatList lambdas, int count)
        {
            Oni.SetBendTwistConstraints(oniBatch, orientationIndices.GetIntPtr(), restOrientations.GetIntPtr(), stiffnesses.GetIntPtr(), lambdas.GetIntPtr(), count);
        }
    }
}
#endif
