using UnityEngine;
using System.Collections;

namespace Obi
{
    public interface IVolumeConstraintsBatchImpl : IConstraintsBatchImpl
    {
        void SetVolumeConstraints(ObiNativeIntList triangles,
                                  ObiNativeIntList firstTriangle,
                                  ObiNativeIntList numTriangles,
                                  ObiNativeFloatList restVolumes,
                                  ObiNativeVector2List pressureStiffness,
                                  ObiNativeFloatList lambdas,
                                  int count);
    }
}
