using UnityEngine;
using Unity.Profiling;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    public class ParticleImpostorRendering
    {
        static ProfilerMarker m_ParticlesToMeshPerfMarker = new ProfilerMarker("ParticlesToMesh");

        private List<Mesh> meshes = new List<Mesh>();

        private List<Vector3> vertices = new List<Vector3>(4000);
        private List<Vector3> normals = new List<Vector3>(4000);
        private List<Color> colors = new List<Color>(4000);
        private List<int> triangles = new List<int>(6000);

        private List<Vector4> anisotropy1 = new List<Vector4>(4000);
        private List<Vector4> anisotropy2 = new List<Vector4>(4000);
        private List<Vector4> anisotropy3 = new List<Vector4>(4000);

        int particlesPerDrawcall = 0;
        int drawcallCount;

        private Vector3 particleOffset0 = new Vector3(1, 1, 0);
        private Vector3 particleOffset1 = new Vector3(-1, 1, 0);
        private Vector3 particleOffset2 = new Vector3(-1, -1, 0);
        private Vector3 particleOffset3 = new Vector3(1, -1, 0);

        public IEnumerable<Mesh> Meshes
        {
            get { return meshes.AsReadOnly(); }
        }

        private void Apply(Mesh mesh)
        {
            mesh.Clear();
            mesh.SetVertices(vertices);
            mesh.SetNormals(normals);
            mesh.SetColors(colors);
            mesh.SetUVs(0, anisotropy1);
            mesh.SetUVs(1, anisotropy2);
            mesh.SetUVs(2, anisotropy3);
            mesh.SetTriangles(triangles, 0, true);
        }

        public void ClearMeshes()
        {
            foreach (Mesh mesh in meshes)
                GameObject.DestroyImmediate(mesh);
            meshes.Clear();
        }

        public void UpdateMeshes(IObiParticleCollection collection)
        {
            using (m_ParticlesToMeshPerfMarker.Auto())
            {

                // figure out the size of our drawcall arrays:
                particlesPerDrawcall = Constants.maxVertsPerMesh / 4;
                drawcallCount = collection.activeParticleCount / particlesPerDrawcall + 1;
                particlesPerDrawcall = Mathf.Min(particlesPerDrawcall, collection.activeParticleCount);

                // If the amount of meshes we need to draw the particles has changed:
                if (drawcallCount != meshes.Count)
                {

                    // Re-generate meshes:
                    ClearMeshes();
                    for (int i = 0; i < drawcallCount; i++)
                    {
                        Mesh mesh = new Mesh();
                        mesh.name = "Particle impostors";
                        mesh.hideFlags = HideFlags.HideAndDontSave;
                        meshes.Add(mesh);
                    }

                }

                Vector3 position;
                Vector4 basis1 = new Vector4(1, 0, 0, 0);
                Vector4 basis2 = new Vector4(0, 1, 0, 0);
                Vector4 basis3 = new Vector4(0, 0, 1, 0);
                Color color;

                //Convert particle data to mesh geometry:
                for (int i = 0; i < drawcallCount; i++)
                {

                    // Clear all arrays
                    vertices.Clear();
                    normals.Clear();
                    colors.Clear();
                    triangles.Clear();
                    anisotropy1.Clear();
                    anisotropy2.Clear();
                    anisotropy3.Clear();

                    int index = 0;
                    int limit = Mathf.Min((i + 1) * particlesPerDrawcall, collection.activeParticleCount);

                    for (int j = i * particlesPerDrawcall; j < limit; ++j)
                    {
                        int runtimeIndex = collection.GetParticleRuntimeIndex(j);
                        position = collection.GetParticlePosition(runtimeIndex);
                        collection.GetParticleAnisotropy(runtimeIndex, ref basis1, ref basis2, ref basis3);
                        color = collection.GetParticleColor(runtimeIndex);

                        vertices.Add(position);
                        vertices.Add(position);
                        vertices.Add(position);
                        vertices.Add(position);

                        normals.Add(particleOffset0);
                        normals.Add(particleOffset1);
                        normals.Add(particleOffset2);
                        normals.Add(particleOffset3);

                        colors.Add(color);
                        colors.Add(color);
                        colors.Add(color);
                        colors.Add(color);

                        anisotropy1.Add(basis1);
                        anisotropy1.Add(basis1);
                        anisotropy1.Add(basis1);
                        anisotropy1.Add(basis1);

                        anisotropy2.Add(basis2);
                        anisotropy2.Add(basis2);
                        anisotropy2.Add(basis2);
                        anisotropy2.Add(basis2);

                        anisotropy3.Add(basis3);
                        anisotropy3.Add(basis3);
                        anisotropy3.Add(basis3);
                        anisotropy3.Add(basis3);

                        triangles.Add(index + 2);
                        triangles.Add(index + 1);
                        triangles.Add(index);
                        triangles.Add(index + 3);
                        triangles.Add(index + 2);
                        triangles.Add(index);

                        index += 4;
                    }

                    Apply(meshes[i]);
                }
            }
        }
    }
}