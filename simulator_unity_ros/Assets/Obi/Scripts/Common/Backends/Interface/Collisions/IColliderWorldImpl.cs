using UnityEngine;
using System.Collections;
using Unity.Jobs;

namespace Obi
{
    public interface IColliderWorldImpl
    {
        void UpdateWorld();

        void SetColliders(ObiNativeColliderShapeList shapes, ObiNativeAabbList bounds, ObiNativeAffineTransformList transforms, int count);
        void SetRigidbodies(ObiNativeRigidbodyList rigidbody);

        void SetCollisionMaterials(ObiNativeCollisionMaterialList materials);

        void SetTriangleMeshData(ObiNativeTriangleMeshHeaderList headers, ObiNativeBIHNodeList nodes, ObiNativeTriangleList triangles, ObiNativeVector3List vertices);
        void SetEdgeMeshData(ObiNativeEdgeMeshHeaderList headers, ObiNativeBIHNodeList nodes, ObiNativeEdgeList triangles, ObiNativeVector2List vertices);
        void SetDistanceFieldData(ObiNativeDistanceFieldHeaderList headers, ObiNativeDFNodeList nodes);
        void SetHeightFieldData(ObiNativeHeightFieldHeaderList headers, ObiNativeFloatList samples);
    }
}
