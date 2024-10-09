#if (OBI_ONI_SUPPORTED)
using System;
using System.Collections;
using System.Runtime.InteropServices;

namespace Obi
{
    public class OniDistanceConstraintsBatchImpl : OniConstraintsBatchImpl, IDistanceConstraintsBatchImpl
    {
        public OniDistanceConstraintsBatchImpl(OniDistanceConstraintsImpl constraints) : base(constraints, Oni.ConstraintType.Distance)
        {
        }

        public void SetDistanceConstraints(ObiNativeIntList particleIndices, ObiNativeFloatList restLengths, ObiNativeVector2List stiffnesses, ObiNativeFloatList lambdas, int count)
        {
            Oni.SetDistanceConstraints(oniBatch, particleIndices.GetIntPtr(), restLengths.GetIntPtr(), stiffnesses.GetIntPtr(), lambdas.GetIntPtr(), count);   
        }
    }
}
#endif