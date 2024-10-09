using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

namespace Obi
{
    public class ObiBlueprintRenderModeShapeMatchingConstraints : ObiBlueprintRenderMode
    {
        public override string name
        {
            get { return "Shape matching constraints"; }
        }

        public ObiBlueprintRenderModeShapeMatchingConstraints(ObiActorBlueprintEditor editor) : base(editor)
        {
        }

        public override void OnSceneRepaint(SceneView sceneView)
        {

            using (new Handles.DrawingScope(Color.cyan, Matrix4x4.identity))
            {
                var constraints = editor.blueprint.GetConstraintsByType(Oni.ConstraintType.ShapeMatching) as ObiConstraints<ObiShapeMatchingConstraintsBatch>;
                if (constraints != null)
                {
                    List<Vector3> lines = new List<Vector3>();

                    foreach (var batch in constraints.batches)
                    {
                        for (int i = 0; i < batch.activeConstraintCount; ++i)
                        {
                            int first = batch.firstIndex[i];
                            Vector3 p1 = editor.blueprint.GetParticlePosition(batch.particleIndices[first]);

                            for (int j = 1; j < batch.numIndices[i]; ++j)
                            {

                                int index = first + j;
                                Vector3 p2 = editor.blueprint.GetParticlePosition(batch.particleIndices[index]);

                                lines.Add(p1);
                                lines.Add(p2);
                            }

                        }
                    }

                    Handles.DrawLines(lines.ToArray());
                }
            }
        }

    }
}