using System;
using UnityEngine;

namespace Obi
{
    [AddComponentMenu("Physics/Obi/Obi Distance Field Renderer", 1003)]
    [ExecuteInEditMode]
    [RequireComponent(typeof(ObiCollider))]
    public class ObiDistanceFieldRenderer : MonoBehaviour
    {
        public enum Axis
        {
            X = 0,
            Y = 1,
            Z = 2,
        }

        public Axis axis;
        [Range(0, 1)]
        public float slice = 0.25f;
        public float maxDistance = 0.5f;

        private ObiCollider unityCollider;
        private Material material;
        private Mesh planeMesh;
        private Texture2D cutawayTexture;

        private float sampleSize;
        private int sampleCount;

        private Color boundsColor = new Color(1, 1, 1, 0.5f);

        public void Awake()
        {
            unityCollider = GetComponent<ObiCollider>();
        }

        public void OnEnable()
        {
            material = GameObject.Instantiate(Resources.Load<Material>("ObiMaterials/DistanceFieldRendering"));
            material.hideFlags = HideFlags.HideAndDontSave;
        }

        public void OnDisable()
        {
            Cleanup();
        }

        private void Cleanup()
        {
            GameObject.DestroyImmediate(cutawayTexture);
            GameObject.DestroyImmediate(planeMesh);
            GameObject.DestroyImmediate(material);
        }

        private void ResizeTexture()
        {

            if (cutawayTexture == null)
            {
                cutawayTexture = new Texture2D(sampleCount, sampleCount, TextureFormat.RHalf, false);
                cutawayTexture.wrapMode = TextureWrapMode.Clamp;
                cutawayTexture.hideFlags = HideFlags.HideAndDontSave;
            }
            else
                cutawayTexture.Resize(sampleCount, sampleCount);
        }

        private void CreatePlaneMesh(ObiDistanceField field)
        {

            if (field != null && planeMesh == null)
            {

                float uvBorder = (1 - field.FieldBounds.size[0] / (sampleSize * sampleCount)) * 0.5f;

                planeMesh = new Mesh();

                planeMesh.vertices = new Vector3[]{new Vector3(-0.5f,-0.5f,0),
                                                   new Vector3(0.5f,-0.5f,0),
                                                   new Vector3(-0.5f,0.5f,0),
                                                   new Vector3(0.5f,0.5f,0)};

                planeMesh.uv = new Vector2[]{new Vector2(uvBorder,uvBorder),
                                             new Vector2(1-uvBorder,uvBorder),
                                             new Vector2(uvBorder,1-uvBorder),
                                             new Vector2(1-uvBorder,1-uvBorder)};

                planeMesh.normals = new Vector3[] { -Vector3.forward, -Vector3.forward, -Vector3.forward, -Vector3.forward };
                planeMesh.triangles = new int[] { 0, 2, 1, 2, 3, 1 };
            }
        }

        private void RefreshCutawayTexture(ObiDistanceField field)
        {
            if (field == null)
                return;

            Bounds b = field.FieldBounds;
            sampleSize = field.EffectiveSampleSize;
            sampleCount = (int)(b.size[0] / sampleSize) + 1;

            CreatePlaneMesh(field);
            ResizeTexture();

            float sweep = (sampleCount * slice) * sampleSize;
            Vector3 origin = b.center - b.extents;

            for (int x = 0; x < sampleCount; ++x)
                for (int y = 0; y < sampleCount; ++y)
                {
                    Vector3 offset = Vector3.zero;
                    switch (axis)
                    {
                        case Axis.X: offset = new Vector3(sweep, y * sampleSize, x * sampleSize); break;
                        case Axis.Y: offset = new Vector3(x * sampleSize, sweep, y * sampleSize); break;
                        case Axis.Z: offset = new Vector3(x * sampleSize, y * sampleSize, sweep); break;
                    }

                    float distance = ASDF.Sample(field.nodes, origin + offset);

                    float value = ObiUtils.Remap(distance, -maxDistance, maxDistance, 0, 1);

                    cutawayTexture.SetPixel(x, y, new Color(value, 0, 0));
                }
            cutawayTexture.Apply();
        }

        private void DrawCutawayPlane(ObiDistanceField field, Matrix4x4 matrix)
        {

            if (field == null)
                return;

            RefreshCutawayTexture(field);

            material.mainTexture = cutawayTexture;
            material.SetPass(0);

            Quaternion rotation = Quaternion.identity;
            Vector3 offset = Vector3.zero;
            offset[(int)axis] = field.FieldBounds.size[0];

            if (axis == Axis.Y)
                rotation = Quaternion.Euler(90, 0, 0);
            else if (axis == Axis.X)
                rotation = Quaternion.Euler(0, -90, 0);

            Matrix4x4 sc = Matrix4x4.TRS(field.FieldBounds.center + offset * (slice - 0.5f), rotation, Vector3.one * field.FieldBounds.size[0]);
            Graphics.DrawMeshNow(planeMesh, matrix * sc);

        }

        public void OnDrawGizmos()
        {
            if (unityCollider != null && unityCollider.distanceField != null && unityCollider.distanceField.Initialized && material != null)
            {
                DrawCutawayPlane(unityCollider.distanceField, transform.localToWorldMatrix);
                Gizmos.color = boundsColor;
                Gizmos.DrawWireCube(unityCollider.distanceField.FieldBounds.center, unityCollider.distanceField.FieldBounds.size);
            }
        }

    }
}

