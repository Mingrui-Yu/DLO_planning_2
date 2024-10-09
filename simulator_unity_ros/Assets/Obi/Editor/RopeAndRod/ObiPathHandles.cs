using UnityEngine;
using UnityEditor;
using System;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{

    public class ObiPathHandles
    {

        static int splineSelectorHash = "ObiPathSelectorHash".GetHashCode();
        const int minSelectionDistance = 5;

        static Vector2 startPos;
        static Vector2 currentPos;
        static bool dragging = false;
        static Rect marquee;

        public static bool SplineCPSelector(ObiPath path, bool[] selectionStatus)
        {

            int controlID = GUIUtility.GetControlID(splineSelectorHash, FocusType.Passive);
            int selectedCPIndex = -1;
            bool selectionStatusChanged = false;

            // select vertex on mouse click:
            switch (Event.current.GetTypeForControl(controlID))
            {

                case EventType.MouseDown:
                    {

                        if ((Event.current.modifiers & EventModifiers.Control) == 0 &&
                            (HandleUtility.nearestControl != controlID || Event.current.button != 0)) break;

                        startPos = Event.current.mousePosition;
                        marquee.Set(0, 0, 0, 0);

                        // If the user is pressing shift, accumulate selection.
                        if ((Event.current.modifiers & EventModifiers.Shift) == 0 && (Event.current.modifiers & EventModifiers.Alt) == 0)
                        {
                            for (int i = 0; i < selectionStatus.Length; i++)
                                selectionStatus[i] = false;
                        }

                        // If the user is holding down control, dont allow selection of other objects and use marquee tool.
                        if ((Event.current.modifiers & EventModifiers.Control) != 0)
                            GUIUtility.hotControl = controlID;

                        float minSqrDistance = System.Single.MaxValue;
                        float sqrMinSelectionDistance = minSelectionDistance * minSelectionDistance;

                        for (int i = 0; i < path.ControlPointCount; i++)
                        {

                            // get particle position in gui space:
                            Vector2 pos = HandleUtility.WorldToGUIPoint(path.points[i].position);

                            // get distance from mouse position to particle position:
                            float sqrDistance = Vector2.SqrMagnitude(startPos - pos);

                            // check if this control point is closer to the cursor that any previously considered point.
                            if (sqrDistance < sqrMinSelectionDistance && sqrDistance < minSqrDistance)
                            {
                                minSqrDistance = sqrDistance;
                                selectedCPIndex = i;
                            }

                        }

                        if (selectedCPIndex >= 0)
                        { // toggle particle selection status.

                            selectionStatus[selectedCPIndex] = !selectionStatus[selectedCPIndex];
                            selectionStatusChanged = true;

                            // Prevent spline deselection if we have selected a particle:
                            GUIUtility.hotControl = controlID;
                            Event.current.Use();

                        }
                        else if (Event.current.modifiers == EventModifiers.None)
                        { // deselect all particles:
                            for (int i = 0; i < selectionStatus.Length; i++)
                                selectionStatus[i] = false;

                            selectionStatusChanged = true;
                        }

                    }
                    break;

                case EventType.MouseDrag:

                    if (GUIUtility.hotControl == controlID)
                    {

                        currentPos = Event.current.mousePosition;
                        if (!dragging && Vector2.Distance(startPos, currentPos) > 5)
                        {
                            dragging = true;
                        }
                        else
                        {
                            GUIUtility.hotControl = controlID;
                            Event.current.Use();
                        }

                        //update marquee rect:
                        float left = Mathf.Min(startPos.x, currentPos.x);
                        float right = Mathf.Max(startPos.x, currentPos.x);
                        float bottom = Mathf.Min(startPos.y, currentPos.y);
                        float top = Mathf.Max(startPos.y, currentPos.y);

                        marquee = new Rect(left, bottom, right - left, top - bottom);

                    }

                    break;

                case EventType.MouseUp:

                    if (GUIUtility.hotControl == controlID)
                    {

                        dragging = false;

                        for (int i = 0; i < path.ControlPointCount; i++)
                        {

                            // get particle position in gui space:
                            Vector2 pos = HandleUtility.WorldToGUIPoint(path.points[i].position);

                            if (pos.x > marquee.xMin && pos.x < marquee.xMax && pos.y > marquee.yMin && pos.y < marquee.yMax)
                            {
                                selectionStatus[i] = true;
                                selectionStatusChanged = true;
                            }

                        }

                        GUIUtility.hotControl = 0;
                        Event.current.Use();
                    }

                    break;

                case EventType.Repaint:

                    if (dragging)
                    {
                        GUISkin oldSkin = GUI.skin;
                        GUI.skin = EditorGUIUtility.GetBuiltinSkin(EditorSkin.Scene);
                        Handles.BeginGUI();
                        GUI.Box(new Rect(marquee.xMin, marquee.yMin, marquee.width, marquee.height), "");
                        Handles.EndGUI();
                        GUI.skin = oldSkin;
                    }

                    break;


                case EventType.Layout:
                    {

                        float minSqrDistance = System.Single.MaxValue;
                        float sqrMinSelectionDistance = minSelectionDistance * minSelectionDistance;

                        for (int i = 0; i < path.ControlPointCount; i++)
                        {

                            // get particle position in gui space:
                            Vector2 pos = HandleUtility.WorldToGUIPoint(path.points[i].position);

                            // get distance from mouse position to particle position:
                            float sqrDistance = Vector2.SqrMagnitude(Event.current.mousePosition - pos);

                            // check if this control point is closer to the cursor that any previously considered point.
                            if (sqrDistance < sqrMinSelectionDistance && sqrDistance < minSqrDistance)
                            { //magic number 900 = 30*30, where 30 is min distance in pixels to select a particle.
                                minSqrDistance = sqrDistance;
                            }

                        }

                        HandleUtility.AddControl(controlID, Mathf.Sqrt(minSqrDistance));
                    }
                    break;

            }

            return selectionStatusChanged;
        }

        private static void DrawControlPointArcs(ObiPath path, float thicknessScale)
        {
            for (int i = 0; i < path.ControlPointCount; ++i)
            {
                Vector3 position = path.points[i].position;
                Vector3 tangent = path.points.GetTangent(i);
                Vector3 right = Vector3.Cross(tangent, path.normals[i]).normalized;
                float thickness = path.thicknesses[i] * thicknessScale + 0.05f;

                Handles.DrawWireArc(position, tangent, right, -180, thickness);
            }
        }

        private static void DrawPathPolylines(Vector3[] samples, Vector3[] leftSamples, Vector3[] rightSamples, Vector3[] upSamples, bool drawOrientation)
        {
            Handles.DrawPolyLine(samples);
            if (drawOrientation)
            {
                Handles.DrawPolyLine(leftSamples);
                Handles.DrawPolyLine(upSamples);
                Handles.DrawPolyLine(rightSamples);
            }
        }

        public static void DrawPathHandle(ObiPath path, Matrix4x4 referenceFrame, float thicknessScale, int resolution, bool drawOrientation = true)
        {

            if (path == null || path.GetSpanCount() == 0) return;

            Matrix4x4 prevMatrix = Handles.matrix;
            Handles.matrix = referenceFrame;

            // Draw the curve:
            int curveSegments = path.GetSpanCount() * resolution;
            Vector3[] samples = new Vector3[curveSegments + 1];
            Vector3[] leftSamples = new Vector3[curveSegments + 1];
            Vector3[] rightSamples = new Vector3[curveSegments + 1];
            Vector3[] upSamples = new Vector3[curveSegments + 1];

            for (int i = 0; i <= curveSegments; ++i)
            {

                float mu = i / (float)curveSegments;
                samples[i] = path.points.GetPositionAtMu(path.Closed,mu);

                if (drawOrientation)
                {
                    Vector3 tangent = path.points.GetTangentAtMu(path.Closed,mu);
                    Vector3 right = Vector3.Cross(tangent, path.normals.GetAtMu(path.Closed,mu)).normalized;
                    Vector3 up = Vector3.Cross(right, tangent).normalized;
                    float thickness = path.thicknesses.GetAtMu(path.Closed,mu) * thicknessScale + 0.05f;

                    leftSamples[i] = samples[i] - right * thickness;
                    rightSamples[i] = samples[i] + right * thickness;
                    upSamples[i] = samples[i] + up * thickness;

                    if (i % 5 == 0)
                    {
                        Handles.DrawLine(leftSamples[i], rightSamples[i]);
                        Handles.DrawLine(samples[i], samples[i] + up * thickness);
                    }
                }
            }

            if (drawOrientation)
                DrawControlPointArcs(path, thicknessScale);

            DrawPathPolylines(samples, leftSamples, rightSamples, upSamples, drawOrientation);
            DrawPathPolylines(samples, leftSamples, rightSamples, upSamples, drawOrientation);

            Handles.matrix = prevMatrix;
        }


    }
}

