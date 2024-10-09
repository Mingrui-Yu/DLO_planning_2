using UnityEngine;
using UnityEditor;
using System;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{

    public class ObiRaycastBrush : ObiBrushBase
    {
        public Matrix4x4 raycastTransform = Matrix4x4.identity;
        public Mesh raycastTarget = null;

        private ObiRaycastHit hit;

        public ObiRaycastBrush(Mesh raycastTarget, Action onStrokeStart, Action onStrokeUpdate, Action onStrokeEnd) : base(onStrokeStart, onStrokeUpdate, onStrokeEnd)
        {
            radius = 0.1f;
            this.raycastTarget = raycastTarget;
        }

        protected override void GenerateWeights(Vector3[] positions)
        {
            if (raycastTarget != null)
            {
                Ray mouseRay = HandleUtility.GUIPointToWorldRay(Event.current.mousePosition);
                if (ObiMeshUtils.WorldRaycast(mouseRay, raycastTransform, raycastTarget.vertices, raycastTarget.triangles, out hit))
                {
                    hit.position = raycastTransform.MultiplyPoint3x4(hit.position);
                    hit.normal = raycastTransform.MultiplyVector(hit.normal);

                    for (int i = 0; i < positions.Length; i++)
                    {
                        // get distance from mouse position to particle position:
                        weights[i] = WeightFromDistance(Vector3.Distance(hit.position, positions[i]));
                    }
                }
            }
        }

		protected override void OnMouseMove(Vector3[] positions)
        {
            base.OnMouseMove(positions);
            GenerateWeights(positions);
		}

		protected override void OnRepaint()
		{
            base.OnRepaint();

            if (raycastTarget != null)
            {
                Color brushColor = ObiEditorSettings.GetOrCreateSettings().brushColor;

                if (hit != null && hit.triangle >= 0)
                {
                    Handles.color = brushColor;
                    Handles.DrawLine(hit.position, hit.position + hit.normal.normalized * radius);
                    Handles.DrawWireDisc(hit.position, hit.normal, radius);
                    Handles.DrawWireDisc(hit.position, hit.normal, innerRadius * radius);
                }
            }
		}
    }
}

