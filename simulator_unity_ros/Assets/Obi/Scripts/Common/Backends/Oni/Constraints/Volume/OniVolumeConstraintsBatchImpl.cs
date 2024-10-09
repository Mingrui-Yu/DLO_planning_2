#if (OBI_ONI_SUPPORTED)
using System;
using System.Collections;
using System.Runtime.InteropServices;

namespace Obi
{
    public class OniVolumeConstraintsBatchImpl : OniConstraintsBatchImpl, IVolumeConstraintsBatchImpl
    {
        public OniVolumeConstraintsBatchImpl(OniVolumeConstraintsImpl constraints) : base(constraints, Oni.ConstraintType.Volume)
        {
        }

        public void SetVolumeConstraints(ObiNativeIntList triangles,
                                      ObiNativeIntList firstTriangle,
                                      ObiNativeIntList numTriangles,
                                      ObiNativeFloatList restVolumes,
                                      ObiNativeVector2List pressureStiffness,
                                      ObiNativeFloatList lambdas,
                                      int count)
        {
            Oni.SetVolumeConstraints(oniBatch, triangles.GetIntPtr(), firstTriangle.GetIntPtr(), numTriangles.GetIntPtr(), restVolumes.GetIntPtr(), pressureStiffness.GetIntPtr(), lambdas.GetIntPtr(), count);
        }
    } 
}
#endif