#if (OBI_ONI_SUPPORTED)
using System;
using System.Collections;
using System.Runtime.InteropServices;

namespace Obi
{
    public class OniTetherConstraintsBatchImpl : OniConstraintsBatchImpl, ITetherConstraintsBatchImpl
    {
        public OniTetherConstraintsBatchImpl(OniTetherConstraintsImpl constraints) : base(constraints, Oni.ConstraintType.Tether)
        {
        }

        public void SetTetherConstraints(ObiNativeIntList particleIndices, ObiNativeVector2List maxLengthScale, ObiNativeFloatList stiffnesses, ObiNativeFloatList lambdas, int count)
        {
            Oni.SetTetherConstraints(oniBatch, particleIndices.GetIntPtr(), maxLengthScale.GetIntPtr(), stiffnesses.GetIntPtr(), lambdas.GetIntPtr(), count);
        }
    }
}
#endif