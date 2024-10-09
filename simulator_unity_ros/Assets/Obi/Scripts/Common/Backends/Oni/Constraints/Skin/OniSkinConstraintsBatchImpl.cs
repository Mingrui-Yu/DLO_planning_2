#if (OBI_ONI_SUPPORTED)
using System;
using System.Collections;
using System.Runtime.InteropServices;

namespace Obi
{
    public class OniSkinConstraintsBatchImpl : OniConstraintsBatchImpl, ISkinConstraintsBatchImpl
    {
        public OniSkinConstraintsBatchImpl(OniSkinConstraintsImpl constraints) : base(constraints, Oni.ConstraintType.Skin)
        {
        }

        public void SetSkinConstraints(ObiNativeIntList particleIndices, ObiNativeVector4List skinPoints, ObiNativeVector4List skinNormals, ObiNativeFloatList skinRadiiBackstop, ObiNativeFloatList skinCompliance, ObiNativeFloatList lambdas, int count)
        {
            Oni.SetSkinConstraints(oniBatch, particleIndices.GetIntPtr(), skinPoints.GetIntPtr(), skinNormals.GetIntPtr(), skinRadiiBackstop.GetIntPtr(), skinCompliance.GetIntPtr(), lambdas.GetIntPtr(), count);
        }
    }
}
#endif