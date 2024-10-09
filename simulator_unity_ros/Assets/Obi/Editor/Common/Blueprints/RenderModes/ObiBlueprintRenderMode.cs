using UnityEngine;
using UnityEditor;
using System.Collections;

namespace Obi
{
    public abstract class ObiBlueprintRenderMode
    {
        protected ObiActorBlueprintEditor editor;
        public abstract string name
        {
            get;
        }
        public ObiBlueprintRenderMode(ObiActorBlueprintEditor editor)
        {
            this.editor = editor;
        }

        public virtual void DrawWithCamera(Camera camera) {}
        public virtual void OnSceneRepaint(SceneView sceneView) {}
        public virtual void Refresh(){}

        public virtual void OnDestroy() { }
    }
}