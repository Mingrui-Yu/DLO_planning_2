using UnityEngine;
using UnityEditor;
using System;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{

    public class ObiScreenSpaceBrush : ObiBrushBase
    {
        public ObiScreenSpaceBrush(Action onStrokeStart, Action onStrokeUpdate, Action onStrokeEnd) : base(onStrokeStart, onStrokeUpdate, onStrokeEnd) 
        {
            radius = 32;
        }
       
        protected override float WeightFromDistance(float distance)
        {
            // anything outside the brush should have zero weight:
            if (distance > radius)
                return 0;
            return 1;
        }

        protected override void GenerateWeights(Vector3[] positions)
        {
            for (int i = 0; i < positions.Length; i++)
            {
                // get particle position in gui space:
                Vector2 pos = HandleUtility.WorldToGUIPoint(positions[i]);

                // get distance from mouse position to particle position:
                weights[i] = WeightFromDistance(Vector3.Distance(Event.current.mousePosition, pos));
            }
        }

        protected override void OnRepaint()
        {
            base.OnRepaint();

            Camera cam = Camera.current;
            float depth = (cam.nearClipPlane + cam.farClipPlane) * 0.5f;

            float ppp = EditorGUIUtility.pixelsPerPoint;
            Vector2 mousePos = new Vector2(Event.current.mousePosition.x * ppp,
                                           cam.pixelHeight - Event.current.mousePosition.y * ppp);

            Handles.color = ObiEditorSettings.GetOrCreateSettings().brushColor;
            Vector3 point = new Vector3(mousePos.x, mousePos.y, depth);
            Vector3 wsPoint = cam.ScreenToWorldPoint(point);

            var p1 = cam.ScreenToWorldPoint(new Vector3(1, 0, depth));
            var p2 = cam.ScreenToWorldPoint(new Vector3(0, 0, depth));
            float units = Vector3.Distance(p1, p2);

            Handles.DrawWireDisc(wsPoint, cam.transform.forward, radius * units);
        }

    }
}

