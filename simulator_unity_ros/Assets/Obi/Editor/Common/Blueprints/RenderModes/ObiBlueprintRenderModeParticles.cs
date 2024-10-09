using UnityEngine;
using System.Collections;

namespace Obi
{
    public class ObiBlueprintRenderModeParticles : ObiBlueprintRenderMode
    {
        public override string name
        {
            get { return "Particles"; }
        }

        private Shader shader;
        private Material material;
        private ParticleImpostorRendering impostorDrawer;

        public ObiBlueprintRenderModeParticles(ObiActorBlueprintEditor editor) :base(editor)
        {
            impostorDrawer = new ParticleImpostorRendering();
            impostorDrawer.UpdateMeshes(editor.blueprint);
        }

        void CreateMaterialIfNeeded()
        {
            if (shader == null)
            {
                shader = Shader.Find("Obi/Particles");
                if (shader != null)
                {
                    if (!shader.isSupported)
                        Debug.LogWarning("Particle rendering shader not suported.");

                    if (material == null || material.shader != shader)
                    {
                        GameObject.DestroyImmediate(material);
                        material = new Material(shader);
                        material.hideFlags = HideFlags.HideAndDontSave;
                    }
                }
            }
        }

        public override void DrawWithCamera(Camera camera) 
        {
            CreateMaterialIfNeeded();
            foreach (Mesh mesh in impostorDrawer.Meshes)
                Graphics.DrawMesh(mesh, Matrix4x4.identity, material, 0);
        }

        public override void Refresh()
        {
            impostorDrawer.UpdateMeshes(editor.blueprint);
        }

        public override void OnDestroy()
        {
            GameObject.DestroyImmediate(material);
            impostorDrawer.ClearMeshes();
        }
    }
}