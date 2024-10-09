using UnityEngine;
using Unity.Profiling;
using System;
using System.Threading;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{

    [AddComponentMenu("Physics/Obi/Obi Instanced Particle Renderer", 1001)]
    [ExecuteInEditMode]
    [RequireComponent(typeof(ObiActor))]
    public class ObiInstancedParticleRenderer : MonoBehaviour
    {
        static ProfilerMarker m_DrawParticlesPerfMarker = new ProfilerMarker("DrawParticles");

        public bool render = true;
        public Mesh mesh;
        public Material material;
        public Vector3 instanceScale = Vector3.one;

        private List<Matrix4x4> matrices = new List<Matrix4x4>();
        private List<Vector4> colors = new List<Vector4>();
        private MaterialPropertyBlock mpb;

        int meshesPerBatch = 0;
        int batchCount;

        public void OnEnable()
        {
            GetComponent<ObiActor>().OnInterpolate += DrawParticles;
        }

        public void OnDisable()
        {
            GetComponent<ObiActor>().OnInterpolate -= DrawParticles;
        }

        void DrawParticles(ObiActor actor)
        {
            using (m_DrawParticlesPerfMarker.Auto())
            {

                if (mesh == null || material == null || !render || !isActiveAndEnabled || !actor.isActiveAndEnabled || actor.solver == null)
                {
                    return;
                }

                ObiSolver solver = actor.solver;

                // figure out the size of our instance batches:
                meshesPerBatch = Constants.maxInstancesPerBatch;
                batchCount = actor.particleCount / meshesPerBatch + 1;
                meshesPerBatch = Mathf.Min(meshesPerBatch, actor.particleCount);

                Vector4 basis1 = new Vector4(1, 0, 0, 0);
                Vector4 basis2 = new Vector4(0, 1, 0, 0);
                Vector4 basis3 = new Vector4(0, 0, 1, 0);

                //Convert particle data to mesh instances:
                for (int i = 0; i < batchCount; i++)
                {

                    matrices.Clear();
                    colors.Clear();
                    mpb = new MaterialPropertyBlock();
                    int limit = Mathf.Min((i + 1) * meshesPerBatch, actor.activeParticleCount);

                    for (int j = i * meshesPerBatch; j < limit; ++j)
                    {
                        int solverIndex = actor.solverIndices[j];
                        actor.GetParticleAnisotropy(solverIndex, ref basis1, ref basis2, ref basis3);
                        matrices.Add(Matrix4x4.TRS(actor.GetParticlePosition(solverIndex),
                                                   actor.GetParticleOrientation(solverIndex),
                                                   Vector3.Scale(new Vector3(basis1[3], basis2[3], basis3[3]), instanceScale)));
                        colors.Add(actor.GetParticleColor(solverIndex));
                    }

                    if (colors.Count > 0)
                        mpb.SetVectorArray("_Color", colors);

                    // Send the meshes to be drawn:
                    Graphics.DrawMeshInstanced(mesh, 0, material, matrices, mpb);
                }
            }

        }

    }
}

