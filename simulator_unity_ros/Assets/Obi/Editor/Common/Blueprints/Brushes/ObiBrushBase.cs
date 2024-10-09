using UnityEngine;
using UnityEditor;
using System;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{

    public abstract class ObiBrushBase
    {
        static int particleBrushHash = "ObiBrushHash".GetHashCode();

        public ObiBrushMode brushMode;
        public float radius = 1;
        public float innerRadius = 0.5f;
        public float opacity = 1;
        public float[] weights = new float[0];

        public float speed = 0.1f;

        protected int controlID;
        protected Action onStrokeStart;
        protected Action onStrokeUpdate;
        protected Action onStrokeEnd;

        public float SqrRadius
        {
            get{ return radius* radius; }
        }

        public ObiBrushBase(Action onStrokeStart, Action onStrokeUpdate, Action onStrokeEnd)
        {
            this.onStrokeStart = onStrokeStart;
            this.onStrokeUpdate = onStrokeUpdate;
            this.onStrokeEnd = onStrokeEnd;
        }

        protected virtual float WeightFromDistance(float distance)
        {
            // anything outside the brush should have zero weight:
            if (distance > radius)
                return 0;
            
            float t = Mathf.InverseLerp(innerRadius * radius, radius, distance);
            return Mathf.SmoothStep(1, 0, t);
        }

        protected abstract void GenerateWeights(Vector3[] positions);

        protected virtual void OnMouseDown(Vector3[] positions)
        {
            if (Event.current.button != 0 || (Event.current.modifiers & ~EventModifiers.Shift) != EventModifiers.None)
                return;

            GUIUtility.hotControl = controlID;

            GenerateWeights(positions);

            if (onStrokeStart != null) 
                onStrokeStart();

            if (brushMode != null)
                brushMode.ApplyStamps(this, (Event.current.modifiers & EventModifiers.Shift) != 0);

            if (onStrokeUpdate != null)
                onStrokeUpdate();

            Event.current.Use();
        }

        protected virtual void OnMouseMove(Vector3[] positions)
        {
            
        }

        protected virtual void OnMouseDrag(Vector3[] positions)
        {

            if (GUIUtility.hotControl == controlID)
            {

                GenerateWeights(positions);

                if (brushMode != null)
                    brushMode.ApplyStamps(this, (Event.current.modifiers & EventModifiers.Shift) != 0);

                if (onStrokeUpdate != null)
                    onStrokeUpdate();

                Event.current.Use();

            }
        }

        protected virtual void OnMouseUp(Vector3[] positions)
        {
            if (GUIUtility.hotControl == controlID)
            {

                GUIUtility.hotControl = 0;
                Event.current.Use();

                if (onStrokeEnd != null) 
                    onStrokeEnd();
            }
        }

        protected virtual void OnRepaint()
        {
        }

        public void DoBrush(Vector3[] positions)
        {

            Matrix4x4 cachedMatrix = Handles.matrix;

            controlID = GUIUtility.GetControlID(particleBrushHash, FocusType.Passive);
            Array.Resize(ref weights, positions.Length);

            switch (Event.current.GetTypeForControl(controlID))
            {

                case EventType.MouseDown:

                    OnMouseDown(positions);

                    break;

                case EventType.MouseMove:

                    OnMouseMove(positions);

                    SceneView.RepaintAll();
                    break;

                case EventType.MouseDrag:

                    OnMouseDrag(positions);

                    break;

                case EventType.MouseUp:

                    OnMouseUp(positions);

                    break;

                case EventType.Repaint:

                    Handles.matrix = Matrix4x4.identity;

                    OnRepaint();

                    Handles.matrix = cachedMatrix;

                    break;

            }
        }
    }
}

