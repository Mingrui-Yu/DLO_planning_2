using UnityEngine;
using System.Collections;

namespace Obi
{
    public static class ObiMeshUtils
    {
        // Temporary vector3 values
        static Vector3 tv1, tv2, tv3, tv4;

        public static bool RayIntersectsTriangle(Vector3 origin,
                                                 Vector3 dir,
                                                 Vector3 vert0,
                                                 Vector3 vert1,
                                                 Vector3 vert2,
                                                 ref float distance,
                                                 ref Vector3 normal)
        {
            float det;

            ObiVectorMath.Subtract(vert0, vert1, ref tv1);
            ObiVectorMath.Subtract(vert0, vert2, ref tv2);

            ObiVectorMath.Cross(dir, tv2, ref tv4);
            det = Vector3.Dot(tv1, tv4);

            if (det < Mathf.Epsilon)
                return false;

            ObiVectorMath.Subtract(vert0, origin, ref tv3);

            float u = Vector3.Dot(tv3, tv4);

            if (u < 0f || u > det)
                return false;

            ObiVectorMath.Cross(tv3, tv1, ref tv4);

            float v = Vector3.Dot(dir, tv4);

            if (v < 0f || u + v > det)
                return false;

            distance = Vector3.Dot(tv2, tv4) * (1f / det);
            ObiVectorMath.Cross(tv1, tv2, ref normal);

            return true;
        }
         /**         * Find the nearest triangle intersected by InWorldRay on this mesh.  InWorldRay is in world space.         * @hit contains information about the hit point.  @distance limits how far from @InWorldRay.origin the hit         * point may be.  @cullingMode determines what face orientations are tested (Culling.Front only tests front         * faces, Culling.Back only tests back faces, and Culling.FrontBack tests both).         * Ray origin and position values are in local space.         */         public static bool WorldRaycast(Ray InWorldRay, Matrix4x4 transform, Vector3[] vertices, int[] triangles, out ObiRaycastHit hit, float distance = Mathf.Infinity)         {
            Ray ray = InWorldRay;
            if (transform != null)
            {
                Matrix4x4 inv = transform.inverse;
                ray.origin = inv.MultiplyPoint3x4(ray.origin);
                ray.direction = inv.MultiplyVector(ray.direction);
            }             return MeshRaycast(ray, vertices, triangles, out hit, distance);         }          /**         *  Cast a ray (in model space) against a mesh.         */         public static bool MeshRaycast(Ray InRay, Vector3[] vertices, int[] triangles, out ObiRaycastHit hit, float distance = Mathf.Infinity)         {
            Vector3 hitNormal = Vector3.zero;   // vars used in loop
            Vector3 vert0, vert1, vert2;
            Vector3 origin = InRay.origin, direction = InRay.direction;

            hit = new ObiRaycastHit(Mathf.Infinity,
                                    Vector3.zero,
                                    Vector3.zero,
                                    -1);
            /**
             * Iterate faces, testing for nearest hit to ray origin.
             */
            for (int CurTri = 0; CurTri < triangles.Length; CurTri += 3)
            {
                if (CurTri + 2 >= triangles.Length) continue;
                if (triangles[CurTri + 2] >= vertices.Length) continue;

                vert0 = vertices[triangles[CurTri + 0]];
                vert1 = vertices[triangles[CurTri + 1]];
                vert2 = vertices[triangles[CurTri + 2]];

                // Second pass, test intersection with triangle
                if (RayIntersectsTriangle(origin, direction, vert0, vert1, vert2, ref distance, ref hitNormal))
                {
                    if (distance < hit.distance)
                    {
                        hit.distance = distance;
                        hit.triangle = CurTri / 3;
                        hit.position = InRay.GetPoint(hit.distance);
                        hit.normal = hitNormal;
                    }
                }
            }

            return hit.triangle > -1;         }
    }

}