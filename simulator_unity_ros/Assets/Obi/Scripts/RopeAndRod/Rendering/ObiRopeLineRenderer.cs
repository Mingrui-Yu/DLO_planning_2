using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using Unity.Profiling;

namespace Obi
{
    [AddComponentMenu("Physics/Obi/Obi Rope Line Renderer", 884)]
    [ExecuteInEditMode]
    [RequireComponent(typeof(MeshRenderer))]
    [RequireComponent(typeof(MeshFilter))]
    [RequireComponent(typeof(ObiPathSmoother))]
    public class ObiRopeLineRenderer : MonoBehaviour
    {
        static ProfilerMarker m_UpdateLineRopeRendererChunksPerfMarker = new ProfilerMarker("UpdateLineRopeRenderer");

        private List<Vector3> vertices = new List<Vector3>();
        private List<Vector3> normals = new List<Vector3>();
        private List<Vector4> tangents = new List<Vector4>();
        private List<Vector2> uvs = new List<Vector2>();
        private List<Color> vertColors = new List<Color>();
        private List<int> tris = new List<int>();

        ObiRopeBase rope;
        ObiPathSmoother smoother;

#if (UNITY_2019_1_OR_NEWER)
        System.Action<ScriptableRenderContext, Camera> renderCallback;
#endif

        [HideInInspector] [NonSerialized] public Mesh lineMesh;

        [Range(0, 1)]
        public float uvAnchor = 0;                  /**< Normalized position of texture coordinate origin along rope.*/

        public Vector2 uvScale = Vector2.one;       /**< Scaling of uvs along rope.*/

        public bool normalizeV = true;

        public float thicknessScale = 0.8f;  /**< Scales section thickness.*/

        void OnEnable()
        {

            CreateMeshIfNeeded();

#if (UNITY_2019_1_OR_NEWER)
            renderCallback = new System.Action<ScriptableRenderContext, Camera>((cntxt, cam) => { UpdateRenderer(cam); });
            RenderPipelineManager.beginCameraRendering += renderCallback;
#endif
            Camera.onPreCull += UpdateRenderer;

            rope = GetComponent<ObiRopeBase>();
            smoother = GetComponent<ObiPathSmoother>();
        }

        void OnDisable()
        {

#if (UNITY_2019_1_OR_NEWER)
            RenderPipelineManager.beginCameraRendering -= renderCallback;
#endif
            Camera.onPreCull -= UpdateRenderer;

            GameObject.DestroyImmediate(lineMesh);
        }

        private void CreateMeshIfNeeded()
        {
            if (lineMesh == null)
            {
                lineMesh = new Mesh();
                lineMesh.name = "extrudedMesh";
                lineMesh.MarkDynamic();
                GetComponent<MeshFilter>().mesh = lineMesh;
            }
        }

        public void UpdateRenderer(Camera camera)
        {
            using (m_UpdateLineRopeRendererChunksPerfMarker.Auto())
            {

                if (camera == null || !rope.gameObject.activeInHierarchy)
                    return;

                CreateMeshIfNeeded();
                ClearMeshData();

                float actualToRestLengthRatio = smoother.SmoothLength / rope.restLength;

                float vCoord = -uvScale.y * rope.restLength * uvAnchor; // v texture coordinate.
                int sectionIndex = 0;

                Vector3 localSpaceCamera = rope.transform.InverseTransformPoint(camera.transform.position);
                Vector3 vertex = Vector3.zero, normal = Vector3.zero;
                Vector4 bitangent = Vector4.zero;
                Vector2 uv = Vector2.zero;

                for (int c = 0; c < smoother.smoothChunks.Count; ++c)
                {

                    ObiList<ObiPathFrame> curve = smoother.smoothChunks[c];

                    for (int i = 0; i < curve.Count; ++i)
                    {

                        // Calculate previous and next curve indices:
                        int prevIndex = Mathf.Max(i - 1, 0);

                        // advance v texcoord:
                        vCoord += uvScale.y * (Vector3.Distance(curve.Data[i].position, curve.Data[prevIndex].position) /
                                               (normalizeV ? smoother.SmoothLength : actualToRestLengthRatio));

                        // calculate section thickness (either constant, or particle radius based):
                        float sectionThickness = curve.Data[i].thickness * thicknessScale;


                        normal.x = curve.Data[i].position.x - localSpaceCamera.x;
                        normal.y = curve.Data[i].position.y - localSpaceCamera.y;
                        normal.z = curve.Data[i].position.z - localSpaceCamera.z;
                        normal.Normalize();

                        bitangent.x = -(normal.y * curve.Data[i].tangent.z - normal.z * curve.Data[i].tangent.y);
                        bitangent.y = -(normal.z * curve.Data[i].tangent.x - normal.x * curve.Data[i].tangent.z);
                        bitangent.z = -(normal.x * curve.Data[i].tangent.y - normal.y * curve.Data[i].tangent.x);
                        bitangent.w = 0;
                        bitangent.Normalize();

                        vertex.x = curve.Data[i].position.x - bitangent.x * sectionThickness;
                        vertex.y = curve.Data[i].position.y - bitangent.y * sectionThickness;
                        vertex.z = curve.Data[i].position.z - bitangent.z * sectionThickness;
                        vertices.Add(vertex);

                        vertex.x = curve.Data[i].position.x + bitangent.x * sectionThickness;
                        vertex.y = curve.Data[i].position.y + bitangent.y * sectionThickness;
                        vertex.z = curve.Data[i].position.z + bitangent.z * sectionThickness;
                        vertices.Add(vertex);

                        normals.Add(-normal);
                        normals.Add(-normal);

                        bitangent.w = 1;
                        tangents.Add(bitangent);
                        tangents.Add(bitangent);

                        vertColors.Add(curve.Data[i].color);
                        vertColors.Add(curve.Data[i].color);

                        uv.x = 0; uv.y = vCoord;
                        uvs.Add(uv);
                        uv.x = 1;
                        uvs.Add(uv);

                        if (i < curve.Count - 1)
                        {
                            tris.Add(sectionIndex * 2);
                            tris.Add((sectionIndex + 1) * 2);
                            tris.Add(sectionIndex * 2 + 1);

                            tris.Add(sectionIndex * 2 + 1);
                            tris.Add((sectionIndex + 1) * 2);
                            tris.Add((sectionIndex + 1) * 2 + 1);
                        }

                        sectionIndex++;
                    }

                }

                CommitMeshData();
            }
        }

        private void ClearMeshData()
        {
            lineMesh.Clear();
            vertices.Clear();
            normals.Clear();
            tangents.Clear();
            uvs.Clear();
            vertColors.Clear();
            tris.Clear();
        }

        private void CommitMeshData()
        {
            lineMesh.SetVertices(vertices);
            lineMesh.SetNormals(normals);
            lineMesh.SetTangents(tangents);
            lineMesh.SetColors(vertColors);
            lineMesh.SetUVs(0, uvs);
            lineMesh.SetTriangles(tris, 0, true);
        }
    }
}


