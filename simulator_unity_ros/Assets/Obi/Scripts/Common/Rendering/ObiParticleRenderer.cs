using UnityEngine;
using Unity.Profiling;
using System;
using System.Threading;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    [AddComponentMenu("Physics/Obi/Obi Particle Renderer", 1000)]
    [ExecuteInEditMode]
    [RequireComponent(typeof(ObiActor))]
    public class ObiParticleRenderer : MonoBehaviour
    {
        static ProfilerMarker m_DrawParticlesPerfMarker = new ProfilerMarker("DrawParticles");

        public bool render = true;
        public Shader shader;
        public Color particleColor = Color.white;
        public float radiusScale = 1;

        private Material material;
        private ParticleImpostorRendering impostors;

        public IEnumerable<Mesh> ParticleMeshes
        {
            get { return impostors.Meshes; }
        }

        public Material ParticleMaterial
        {
            get { return material; }
        }

        public void OnEnable()
        {
            impostors = new ParticleImpostorRendering();
            GetComponent<ObiActor>().OnInterpolate += DrawParticles;
        }

        public void OnDisable()
        {
            GetComponent<ObiActor>().OnInterpolate -= DrawParticles;

            if (impostors != null)
                impostors.ClearMeshes();
            DestroyImmediate(material);
        }

        void CreateMaterialIfNeeded()
        {

            if (shader != null)
            {

                if (!shader.isSupported)
                    Debug.LogWarning("Particle rendering shader not suported.");

                if (material == null || material.shader != shader)
                {
                    DestroyImmediate(material);
                    material = new Material(shader);
                    material.hideFlags = HideFlags.HideAndDontSave;
                }
            }
        }

        void DrawParticles(ObiActor actor)
        {
            using (m_DrawParticlesPerfMarker.Auto())
            {
                if (!isActiveAndEnabled || !actor.isActiveAndEnabled || actor.solver == null)
                {
                    impostors.ClearMeshes();
                    return;
                }

                CreateMaterialIfNeeded();

                impostors.UpdateMeshes(actor);

                DrawParticles();
            }
        }

        private void DrawParticles()
        {
            if (material != null)
            {

                material.SetFloat("_RadiusScale", radiusScale);
                material.SetColor("_Color", particleColor);

                // Send the meshes to be drawn:
                if (render)
                {
                    foreach (Mesh mesh in impostors.Meshes)
                        Graphics.DrawMesh(mesh, Matrix4x4.identity, material, gameObject.layer);
                }
            }

        }

    }
}

