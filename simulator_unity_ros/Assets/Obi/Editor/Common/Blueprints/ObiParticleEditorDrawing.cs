using UnityEngine;
using UnityEditor;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    public class ObiParticleEditorDrawing : MonoBehaviour
    {
        public static Mesh particlesMesh;
        public static Material particleMaterial;

        private static void CreateParticlesMesh()
        {
            if (particlesMesh == null)
            {
                particlesMesh = new Mesh();
                particlesMesh.hideFlags = HideFlags.HideAndDontSave;
            }
        }

        private static void CreateParticleMaterials()
        {
            if (!particleMaterial)
            {
                particleMaterial = Resources.Load<Material>("EditorParticle");
            }
        }

        public static void DestroyParticlesMesh()
        {
            GameObject.DestroyImmediate(particlesMesh);
        }

        public static void DrawParticles(Camera cam, ObiActorBlueprint blueprint, bool[] facingCamera, bool[] selectionStatus, int[] sortedIndices)
        {
            CreateParticlesMesh();
            CreateParticleMaterials();

            if (!particleMaterial.SetPass(0))
                return;

            //because each vertex needs to be drawn as a quad.
            int particlesPerDrawcall = Constants.maxVertsPerMesh / 4;
            int drawcallCount = blueprint.particleCount / particlesPerDrawcall + 1;
            particlesPerDrawcall = Mathf.Min(particlesPerDrawcall, blueprint.particleCount);

            List<Vector3> vertices = new List<Vector3>(blueprint.activeParticleCount* 4);
            List<Vector3> normals = new List<Vector3>(blueprint.activeParticleCount * 4);
            List<Vector4> uvs = new List<Vector4>(blueprint.activeParticleCount * 4);
            List<Color> colors = new List<Color>(blueprint.activeParticleCount * 4);
            List<int> triangles = new List<int>(blueprint.activeParticleCount * 6);

            Color regularColor = ObiEditorSettings.GetOrCreateSettings().particleColor;
            Color selectedColor = ObiEditorSettings.GetOrCreateSettings().selectedParticleColor;

            Vector3 particleOffset0 = new Vector3(1, 1, 0);
            Vector3 particleOffset1 = new Vector3(-1, 1, 0);
            Vector3 particleOffset2 = new Vector3(-1, -1, 0);
            Vector3 particleOffset3 = new Vector3(1, -1, 0);

            Vector4 radius = new Vector4(1, 0, 0, 0.006f);

            for (int i = 0; i < drawcallCount; ++i)
            {
                //Draw all cloth vertices:      
                particlesMesh.Clear();
                vertices.Clear();
                uvs.Clear();
                normals.Clear();
                colors.Clear();
                triangles.Clear();

                int index = 0;

                // Run over all particles (not only active ones), since they're reordered based on distance to camera.
                // Then test if the sorted index is active or not, and skip inactive ones.
                int limit = Mathf.Min((i + 1) * particlesPerDrawcall, blueprint.particleCount);

                for (int j = i * particlesPerDrawcall; j < limit; ++j)
                {
                    int sortedIndex = sortedIndices[j];

                    // skip inactive ones: 
                    if (!blueprint.IsParticleActive(sortedIndex))
                        continue;

                    // get particle color:
                    Color color = selectionStatus[sortedIndex] ? selectedColor : regularColor;
                    color.a = facingCamera[sortedIndex] ? 1 : 0.15f;

                    normals.Add(particleOffset0);
                    normals.Add(particleOffset1);
                    normals.Add(particleOffset2);
                    normals.Add(particleOffset3);

                    uvs.Add(radius);
                    uvs.Add(radius);
                    uvs.Add(radius);
                    uvs.Add(radius);

                    vertices.Add(blueprint.positions[sortedIndex]);
                    vertices.Add(blueprint.positions[sortedIndex]);
                    vertices.Add(blueprint.positions[sortedIndex]);
                    vertices.Add(blueprint.positions[sortedIndex]);

                    colors.Add(color);
                    colors.Add(color);
                    colors.Add(color);
                    colors.Add(color);

                    triangles.Add(index + 2);
                    triangles.Add(index + 1);
                    triangles.Add(index);
                    triangles.Add(index + 3);
                    triangles.Add(index + 2);
                    triangles.Add(index);

                    index += 4;
                }

                particlesMesh.SetVertices(vertices);
                particlesMesh.SetNormals(normals);
                particlesMesh.SetColors(colors);
                particlesMesh.SetUVs(0, uvs);
                particlesMesh.SetTriangles(triangles,0, true);

                Graphics.DrawMeshNow(particlesMesh, Matrix4x4.identity);
            }
        }

    }

}