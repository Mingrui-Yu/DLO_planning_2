using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Profiling;

namespace Obi
{
    [AddComponentMenu("Physics/Obi/Obi Rope Mesh Renderer", 886)]
    [ExecuteInEditMode]
    [RequireComponent(typeof(MeshRenderer))]
    [RequireComponent(typeof(MeshFilter))]
    [RequireComponent(typeof(ObiPathSmoother))]
    public class ObiRopeMeshRenderer : MonoBehaviour
    {
        static ProfilerMarker m_UpdateMeshRopeRendererChunksPerfMarker = new ProfilerMarker("UpdateMeshRopeRenderer");

        [SerializeProperty("SourceMesh")]
        [SerializeField] private Mesh mesh;

        [SerializeProperty("SweepAxis")]
        [SerializeField] private ObiPathFrame.Axis axis;

        public float volumeScaling = 0;
        public bool stretchWithRope = true;
        public bool spanEntireLength = true;

        [SerializeProperty("Instances")]
        [SerializeField] private int instances = 1;

        [SerializeProperty("InstanceSpacing")]
        [SerializeField] private float instanceSpacing = 1;

        public float offset = 0;
        public Vector3 scale = Vector3.one;

        [HideInInspector] [SerializeField] private float meshSizeAlongAxis = 1;

        private Vector3[] inputVertices;
        private Vector3[] inputNormals;
        private Vector4[] inputTangents;

        private Vector3[] vertices;
        private Vector3[] normals;
        private Vector4[] tangents;

        private int[] orderedVertices = new int[0];

        private ObiPathSmoother smoother;

        public Mesh SourceMesh
        {
            set { mesh = value; PreprocessInputMesh(); }
            get { return mesh; }
        }

        public ObiPathFrame.Axis SweepAxis
        {
            set { axis = value; PreprocessInputMesh(); }
            get { return axis; }
        }

        public int Instances
        {
            set { instances = value; PreprocessInputMesh(); }
            get { return instances; }
        }

        public float InstanceSpacing
        {
            set { instanceSpacing = value; PreprocessInputMesh(); }
            get { return instanceSpacing; }
        }

        [HideInInspector] [NonSerialized] public Mesh deformedMesh;

        void OnEnable()
        {
            smoother = GetComponent<ObiPathSmoother>();
            smoother.OnCurveGenerated += UpdateRenderer;
            PreprocessInputMesh();
        }

        void OnDisable()
        {
            smoother.OnCurveGenerated -= UpdateRenderer;
            GameObject.DestroyImmediate(deformedMesh);
        }

        private void PreprocessInputMesh()
        {

            if (deformedMesh == null)
            {
                deformedMesh = new Mesh();
                deformedMesh.name = "deformedMesh";
                deformedMesh.MarkDynamic();
                GetComponent<MeshFilter>().mesh = deformedMesh;
            }

            deformedMesh.Clear();

            if (mesh == null)
            {
                orderedVertices = new int[0];
                return;
            }

            // Clamp instance count to a positive value.
            instances = Mathf.Max(0, instances);

            // combine all mesh instances into a single mesh:
            Mesh combinedMesh = new Mesh();
            CombineInstance[] meshInstances = new CombineInstance[instances];
            Vector3 pos = Vector3.zero;

            // initial offset for the combined mesh is half the size of its bounding box in the swept axis:
            pos[(int)axis] = mesh.bounds.extents[(int)axis];

            for (int i = 0; i < instances; ++i)
            {
                meshInstances[i].mesh = mesh;
                meshInstances[i].transform = Matrix4x4.TRS(pos, Quaternion.identity, Vector3.one);
                pos[(int)axis] = mesh.bounds.extents[(int)axis] + (i + 1) * mesh.bounds.size[(int)axis] * instanceSpacing;
            }
            combinedMesh.CombineMeshes(meshInstances, true, true);

            // get combined mesh data:
            inputVertices = combinedMesh.vertices;
            inputNormals = combinedMesh.normals;
            inputTangents = combinedMesh.tangents;

            // sort vertices along curve axis:
            float[] keys = new float[inputVertices.Length];
            orderedVertices = new int[inputVertices.Length];

            for (int i = 0; i < keys.Length; ++i)
            {
                keys[i] = inputVertices[i][(int)axis];
                orderedVertices[i] = i;
            }

            Array.Sort(keys, orderedVertices);

            // Copy the combined mesh data to deform it:
            deformedMesh.vertices = combinedMesh.vertices;
            deformedMesh.normals = combinedMesh.normals;
            deformedMesh.tangents = combinedMesh.tangents;
            deformedMesh.uv = combinedMesh.uv;
            deformedMesh.uv2 = combinedMesh.uv2;
            deformedMesh.uv3 = combinedMesh.uv3;
            deformedMesh.uv4 = combinedMesh.uv4;
            deformedMesh.colors = combinedMesh.colors;
            deformedMesh.triangles = combinedMesh.triangles;

            vertices = deformedMesh.vertices;
            normals = deformedMesh.normals;
            tangents = deformedMesh.tangents;

            // Calculate scale along swept axis so that the mesh spans the entire lenght of the rope if required.
            meshSizeAlongAxis = combinedMesh.bounds.size[(int)axis];

            // destroy combined mesh:
            GameObject.DestroyImmediate(combinedMesh);

        }

        public void UpdateRenderer(ObiActor actor)
        {
            using (m_UpdateMeshRopeRendererChunksPerfMarker.Auto())
            {

                if (mesh == null)
                    return;

                if (smoother.smoothChunks.Count == 0)
                    return;

                ObiList<ObiPathFrame> curve = smoother.smoothChunks[0];

                if (curve.Count < 2)
                    return;

                var rope = actor as ObiRopeBase;

                float actualToRestLengthRatio = stretchWithRope ? smoother.SmoothLength / rope.restLength : 1;

                // squashing factor, makes mesh thinner when stretched and thicker when compresssed.
                float squashing = Mathf.Clamp(1 + volumeScaling * (1 / Mathf.Max(actualToRestLengthRatio, 0.01f) - 1), 0.01f, 2);

                // Calculate scale along swept axis so that the mesh spans the entire lenght of the rope if required.
                Vector3 actualScale = scale;
                if (spanEntireLength)
                    actualScale[(int)axis] = rope.restLength / meshSizeAlongAxis;

                float previousVertexValue = 0;
                float meshLength = 0;
                int index = 0;
                int nextIndex = 1;
                int prevIndex = 0;
                float sectionMagnitude = Vector3.Distance(curve[index].position, curve[nextIndex].position);

                // basis matrix for deforming the mesh:
                Matrix4x4 basis = curve[0].ToMatrix(axis);

                for (int i = 0; i < orderedVertices.Length; ++i)
                {

                    int vIndex = orderedVertices[i];
                    float vertexValue = inputVertices[vIndex][(int)axis] * actualScale[(int)axis] + offset;

                    // Calculate how much we've advanced in the sort axis since the last vertex:
                    meshLength += (vertexValue - previousVertexValue) * actualToRestLengthRatio;
                    previousVertexValue = vertexValue;

                    // If we have advanced to the next section of the curve:
                    while (meshLength > sectionMagnitude && sectionMagnitude > Mathf.Epsilon)
                    {

                        meshLength -= sectionMagnitude;
                        index = Mathf.Min(index + 1, curve.Count - 1);

                        // Calculate previous and next curve indices:
                        nextIndex = Mathf.Min(index + 1, curve.Count - 1);
                        prevIndex = Mathf.Max(index - 1, 0);

                        // Calculate current tangent as the vector between previous and next curve points:
                        sectionMagnitude = Vector3.Distance(curve[index].position, curve[nextIndex].position);

                        // Update basis matrix:
                        basis = curve[index].ToMatrix(axis);

                    }

                    float sectionThickness = curve[index].thickness;

                    // calculate deformed vertex position:
                    Vector3 offsetFromCurve = Vector3.Scale(inputVertices[vIndex], actualScale * sectionThickness * squashing);
                    offsetFromCurve[(int)axis] = meshLength;

                    vertices[vIndex] = curve[index].position + basis.MultiplyVector(offsetFromCurve);
                    normals[vIndex] = basis.MultiplyVector(inputNormals[vIndex]);
                    tangents[vIndex] = basis * inputTangents[vIndex]; // avoids expensive implicit conversion from Vector4 to Vector3.
                    tangents[vIndex].w = inputTangents[vIndex].w;
                }

                CommitMeshData();
            }
        }

        private void CommitMeshData()
        {
            deformedMesh.vertices = vertices;
            deformedMesh.normals = normals;
            deformedMesh.tangents = tangents;
            deformedMesh.RecalculateBounds();
        }
    }
}


