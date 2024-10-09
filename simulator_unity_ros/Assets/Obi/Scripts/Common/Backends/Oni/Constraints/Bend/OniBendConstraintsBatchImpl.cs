#if (OBI_ONI_SUPPORTED)
using System;
using System.Collections;
using System.Runtime.InteropServices;

namespace Obi
{
    public class OniBendConstraintsBatchImpl : OniConstraintsBatchImpl, IBendConstraintsBatchImpl
    {
        public OniBendConstraintsBatchImpl(OniBendConstraintsImpl constraints) : base(constraints, Oni.ConstraintType.Bending)
        {
        }

        public void SetBendConstraints(ObiNativeIntList particleIndices, ObiNativeFloatList restBends, ObiNativeVector2List bendingStiffnesses, ObiNativeFloatList lambdas, int count)
        {
            Oni.SetBendingConstraints(oniBatch, particleIndices.GetIntPtr(), restBends.GetIntPtr(), bendingStiffnesses.GetIntPtr(), lambdas.GetIntPtr(), count);
        }
    }
}
#endif