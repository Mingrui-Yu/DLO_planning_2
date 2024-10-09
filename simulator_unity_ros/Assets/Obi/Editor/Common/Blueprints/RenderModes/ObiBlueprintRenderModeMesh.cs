using UnityEngine;
using UnityEditor;
using System.Collections;

namespace Obi
{
    public class ObiBlueprintRenderModeMesh : ObiBlueprintRenderMode
    {
        public override string name
        {
            get { return "Mesh"; }
        }

        public ObiMeshBasedActorBlueprintEditor meshBasedEditor
        {
            get { return editor as ObiMeshBasedActorBlueprintEditor; }
        }

        public ObiBlueprintRenderModeMesh(ObiMeshBasedActorBlueprintEditor editor) : base(editor)
        {
        }

        public override void OnSceneRepaint(SceneView sceneView) 
        {
            if (meshBasedEditor.currentTool is ObiPaintBrushEditorTool)
            {
                ObiPaintBrushEditorTool paintTool = (ObiPaintBrushEditorTool)meshBasedEditor.currentTool;

                float[] weights = new float[editor.selectionStatus.Length];
                for (int i = 0; i < weights.Length; i++)
                {
                    if (paintTool.selectionMask && !editor.selectionStatus[i])
                        weights[i] = 0;
                    else
                        weights[i] = 1;
                }

                float[] wireframeWeights = new float[paintTool.paintBrush.weights.Length];
                for (int i = 0; i < wireframeWeights.Length; i++)
                    wireframeWeights[i] = paintTool.paintBrush.weights[i];

                meshBasedEditor.DrawGradientMesh(weights, wireframeWeights);
            }
            else
                meshBasedEditor.DrawGradientMesh();
        }
    }
}