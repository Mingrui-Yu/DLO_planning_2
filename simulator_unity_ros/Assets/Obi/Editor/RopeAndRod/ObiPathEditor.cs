using UnityEngine;
using UnityEditor;
using System;
using System.Collections;

namespace Obi
{
    public class ObiPathEditor
    {
        UnityEngine.Object target;
        ObiPath path;

        Quaternion prevRot = Quaternion.identity;
        Vector3 prevScale = Vector3.one;

        bool insertTool = false;
        bool removeTool = false;
        bool orientTool = false;
        bool showTangentHandles = true;
        bool showThicknessHandles = true;

        public bool needsRepaint = false;

        protected bool[] selectedStatus;
        protected int lastSelected = 0;
        protected int selectedCount = 0;
        protected Vector3 selectionAverage;
        protected bool useOrientation = false;

        protected static Color handleColor = new Color(1, 0.55f, 0.1f);

        public ObiPathEditor(UnityEngine.Object target, ObiPath path, bool useOrientation){
            this.target = target;
            this.path = path;
            this.useOrientation = useOrientation;
            selectedStatus = new bool[path.ControlPointCount];
            ResizeCPArrays();
        }

        public void ResizeCPArrays()
        {
            Array.Resize(ref selectedStatus, path.ControlPointCount);
        }

        int windowId;
        public bool OnSceneGUI(float thicknessScale, Matrix4x4 matrix)
        {
            ResizeCPArrays();

            HandleUtility.AddDefaultControl(GUIUtility.GetControlID(FocusType.Passive));

            // get a window ID:
            if (Event.current.type != EventType.Used)
                windowId  = GUIUtility.GetControlID(FocusType.Passive);

            Matrix4x4 prevMatrix = Handles.matrix;
            Handles.matrix = matrix;

            // Draw control points:
            Handles.color = handleColor;
            for (int i = 0; i < path.ControlPointCount; ++i)
            {
                needsRepaint |= DrawControlPoint(i);
            }

            // Control point selection handle:
            needsRepaint |= ObiPathHandles.SplineCPSelector(path, selectedStatus);

            // Count selected and calculate average position:
            selectionAverage = GetControlPointAverage(out lastSelected, out selectedCount);

            // Draw cp tool handles:
            needsRepaint |= SplineCPTools(matrix);

            if (showThicknessHandles)
                needsRepaint |= DoThicknessHandles(thicknessScale);

            // Sceneview GUI:
            Handles.BeginGUI();
            GUILayout.Window(windowId, new Rect(10, 28, 0, 0), DrawUIWindow, "Path editor");
            Handles.EndGUI();

            Handles.matrix = prevMatrix;

            // During edit mode, allow to add/remove control points.
            if (insertTool)
                AddControlPointsMode(matrix);

            if (removeTool)
                RemoveControlPointsMode(matrix);

            return needsRepaint;
        }

        private void AddControlPointsMode(Matrix4x4 matrix)
        {

            float mu = ScreenPointToCurveMu(path, Event.current.mousePosition, matrix);

            Vector3 pointOnSpline = matrix.MultiplyPoint3x4(path.points.GetPositionAtMu(path.Closed,mu));

            float size = HandleUtility.GetHandleSize(pointOnSpline) * 0.12f;

            Ray ray = HandleUtility.GUIPointToWorldRay(Event.current.mousePosition);
            Handles.color = Color.green;
            Handles.DrawDottedLine(pointOnSpline, ray.origin, 4);
            Handles.SphereHandleCap(0, pointOnSpline, Quaternion.identity, size, Event.current.type);


            if (Event.current.type == EventType.MouseDown && Event.current.modifiers == EventModifiers.None)
            {
                Undo.RecordObject(target, "Add");

                int newIndex = path.InsertControlPoint(mu);
                if (newIndex >= 0)
                {
                    ResizeCPArrays();
                    for (int i = 0; i < selectedStatus.Length; ++i)
                        selectedStatus[i] = false;
                    selectedStatus[newIndex] = true;
                }

                path.FlushEvents();
                Event.current.Use();
            }

            // Repaint the scene, so that the add control point helpers are updated every frame.
            SceneView.RepaintAll();

        }

        private void RemoveControlPointsMode(Matrix4x4 matrix)
        {

            float mu = ScreenPointToCurveMu(path, Event.current.mousePosition, matrix);

            Vector3 pointOnSpline = matrix.MultiplyPoint3x4(path.points.GetPositionAtMu(path.Closed,mu));

            float size = HandleUtility.GetHandleSize(pointOnSpline) * 0.12f;

            Ray ray = HandleUtility.GUIPointToWorldRay(Event.current.mousePosition);
   
            Handles.color = Color.red;
            Handles.DrawDottedLine(pointOnSpline, ray.origin, 4);

            int index = path.GetClosestControlPointIndex(mu);
            Handles.SphereHandleCap(0, matrix.MultiplyPoint3x4(path.points[index].position), Quaternion.identity, size, Event.current.type);

            if (Event.current.type == EventType.MouseDown && Event.current.modifiers == EventModifiers.None && index >= 0 && path.ControlPointCount > 2)
            {
                Undo.RecordObject(target, "Remove");

                path.RemoveControlPoint(index);
                ResizeCPArrays();
                for (int i = 0; i < selectedStatus.Length; ++i)
                    selectedStatus[i] = false;

                path.FlushEvents();
                Event.current.Use();
            }

            // Repaint the scene, so that the add control point helpers are updated every frame.
            SceneView.RepaintAll();

        }

        protected bool DrawControlPoint(int i)
        {
            bool repaint = false;
            var wp = path.points[i];
            float size = HandleUtility.GetHandleSize(wp.position) * 0.04f;

            if (selectedStatus[i] && showTangentHandles)
            {

                Handles.color = handleColor;

                if (!(i == 0 && !path.Closed))
                {
                    Vector3 tangentPosition = wp.inTangentEndpoint;

                    if (Event.current.type == EventType.Repaint)
                        Handles.DrawDottedLine(tangentPosition, wp.position, 2);

                    EditorGUI.BeginChangeCheck();
                    Handles.DotHandleCap(0, tangentPosition, Quaternion.identity, size, Event.current.type);
                    Vector3 newTangent = Handles.PositionHandle(tangentPosition, Quaternion.identity);
                    if (EditorGUI.EndChangeCheck())
                    {
                        Undo.RecordObject(target, "Modify tangent");
                        wp.SetInTangentEndpoint(newTangent);
                        path.points[i] = wp;
                        path.FlushEvents();
                        repaint = true;
                    }
                }

                if (!(i == path.ControlPointCount - 1 && !path.Closed))
                {
                    Vector3 tangentPosition = wp.outTangentEndpoint;

                    if (Event.current.type == EventType.Repaint)
                        Handles.DrawDottedLine(tangentPosition, wp.position, 2);

                    EditorGUI.BeginChangeCheck();
                    Handles.DotHandleCap(0, tangentPosition, Quaternion.identity, size, Event.current.type);
                    Vector3 newTangent = Handles.PositionHandle(tangentPosition, Quaternion.identity);
                    if (EditorGUI.EndChangeCheck())
                    {
                        Undo.RecordObject(target, "Modify tangent");
                        wp.SetOutTangentEndpoint(newTangent);
                        path.points[i] = wp;
                        path.FlushEvents();
                        repaint = true;
                    }
                }
            }

            if (Event.current.type == EventType.Repaint)
            {

                Handles.color = selectedStatus[i] ? handleColor : Color.white;
                Vector3 pos = wp.position;

                if (orientTool)
                {
                    Handles.ArrowHandleCap(0, pos, Quaternion.LookRotation(path.normals[i]), HandleUtility.GetHandleSize(pos), EventType.Repaint);
                }

                Handles.SphereHandleCap(0, pos, Quaternion.identity, size * 3, EventType.Repaint);

            }
            return repaint;
        }

        protected Vector3 GetControlPointAverage(out int lastSelected, out int selectedCount)
        {

            lastSelected = -1;
            selectedCount = 0;
            Vector3 averagePos = Vector3.zero;

            // Find center of all selected control points:
            for (int i = 0; i < path.ControlPointCount; ++i)
            {
                if (selectedStatus[i])
                {

                    averagePos += path.points[i].position;
                    selectedCount++;
                    lastSelected = i;

                }
            }
            if (selectedCount > 0)
                averagePos /= selectedCount;
            return averagePos;

        }

        protected bool SplineCPTools(Matrix4x4 matrix)
        {
            bool repaint = false;

            // Calculate handle rotation, for local or world pivot modes.
            Quaternion handleRotation = Tools.pivotRotation == PivotRotation.Local ? Quaternion.identity : Quaternion.Inverse(matrix.rotation);

            // Reset initial handle rotation/orientation after using a tool:
            if (GUIUtility.hotControl == 0)
            {

                prevRot = handleRotation;
                prevScale = Vector3.one;

                if (selectedCount == 1 && Tools.pivotRotation == PivotRotation.Local && orientTool)
                {
                    //prevRot = Quaternion.LookRotation(GetNormal(lastSelected));
                }
            }

            // Transform handles:
            if (selectedCount > 0)
            {

                if (useOrientation && orientTool)
                {
                    repaint |= OrientTool(selectionAverage,handleRotation);
                }
                else
                {
                    switch (Tools.current)
                    {
                        case Tool.Move:
                            {
                                repaint |= MoveTool(selectionAverage, handleRotation);
                            }
                            break;

                        case Tool.Scale:
                            {
                                repaint |= ScaleTool(selectionAverage, handleRotation);
                            }
                            break;

                        case Tool.Rotate:
                            {
                                repaint |= RotateTool(selectionAverage, handleRotation);
                            }
                            break;
                    }
                }
            }
            return repaint;
        }

        protected bool MoveTool(Vector3 handlePosition, Quaternion handleRotation)
        {

            EditorGUI.BeginChangeCheck();
            Vector3 newPos = Handles.PositionHandle(handlePosition, handleRotation);
            if (EditorGUI.EndChangeCheck())
            {

                Undo.RecordObject(target, "Move control point");

                Vector3 delta = newPos - handlePosition;

                for (int i = 0; i < path.ControlPointCount; ++i)
                {
                    if (selectedStatus[i])
                    {
                        var wp = path.points[i];
                        wp.Transform(delta, Quaternion.identity, Vector3.one);
                        path.points[i] = wp;
                    }
                }

                path.FlushEvents();
                return true;
            }
            return false;
        }

        protected bool ScaleTool(Vector3 handlePosition, Quaternion handleRotation)
        {

            EditorGUI.BeginChangeCheck();
            Vector3 scale = Handles.ScaleHandle(prevScale, handlePosition, handleRotation, HandleUtility.GetHandleSize(handlePosition));

            if (EditorGUI.EndChangeCheck())
            {

                Vector3 deltaScale = new Vector3(scale.x / prevScale.x, scale.y / prevScale.y, scale.z / prevScale.z);
                prevScale = scale;

                Undo.RecordObject(target, "Scale control point");

                if (Tools.pivotMode == PivotMode.Center && selectedCount > 1)
                {
                    for (int i = 0; i < path.ControlPointCount; ++i)
                    {
                        if (selectedStatus[i])
                        {
                            var wp = path.points[i];
                            Vector3 newPos = handlePosition + Vector3.Scale(wp.position - handlePosition, deltaScale);
                            wp.Transform(newPos - wp.position, Quaternion.identity, Vector3.one);
                            path.points[i] = wp;
                        }
                    }
                }
                else
                {
                    // Scale all handles of selected control points relative to their control point:
                    for (int i = 0; i < path.ControlPointCount; ++i)
                    {
                        if (selectedStatus[i])
                        {
                            var wp = path.points[i];
                            wp.Transform(Vector3.zero, Quaternion.identity, deltaScale);
                            path.points[i] = wp;
                        }
                    }
                }

                path.FlushEvents();
                return true;
            }
            return false;
        }

        protected bool RotateTool(Vector3 handlePosition, Quaternion handleRotation)
        {

            EditorGUI.BeginChangeCheck();
            // TODO: investigate weird rotation gizmo:
            Quaternion newRotation = Handles.RotationHandle(prevRot, handlePosition);

            if (EditorGUI.EndChangeCheck())
            {

                Quaternion delta = newRotation * Quaternion.Inverse(prevRot);
                prevRot = newRotation;

                Undo.RecordObject(target, "Rotate control point");

                if (Tools.pivotMode == PivotMode.Center && selectedCount > 1)
                {

                    // Rotate all selected control points around their average:
                    for (int i = 0; i < path.ControlPointCount; ++i)
                    {
                        if (selectedStatus[i])
                        {
                            var wp = path.points[i];
                            Vector3 newPos = handlePosition + delta * (wp.position - handlePosition);
                            wp.Transform(newPos - wp.position, Quaternion.identity, Vector3.one);
                            path.points[i] = wp;
                        }
                    }

                }
                else
                {

                    // Rotate all handles of selected control points around their control point:
                    for (int i = 0; i < path.ControlPointCount; ++i)
                    {
                        if (selectedStatus[i])
                        {
                            var wp = path.points[i];
                            wp.Transform(Vector3.zero, delta, Vector3.one); 
                            path.points[i] = wp;
                        }
                    }
                }

                path.FlushEvents();
                return true;
            }
            return false;
        }

        protected bool OrientTool(Vector3 averagePos, Quaternion pivotRotation)
        {

            EditorGUI.BeginChangeCheck();
            Quaternion newRotation = Handles.RotationHandle(prevRot, averagePos);

            if (EditorGUI.EndChangeCheck())
            {

                Quaternion delta = newRotation * Quaternion.Inverse(prevRot);
                prevRot = newRotation;

                Undo.RecordObject(target, "Orient control point");

                // Rotate all selected control points around their average:
                for (int i = 0; i < path.ControlPointCount; ++i)
                {
                    if (selectedStatus[i])
                    {
                        path.normals[i] = delta * path.normals[i];
                    }
                }

                path.FlushEvents();
                return true;
            }
            return false;
        }


        protected bool DoThicknessHandles(float scale)
        {
            Color oldColor = Handles.color;
            Handles.color = handleColor;

            EditorGUI.BeginChangeCheck();
            for (int i = 0; i < path.ControlPointCount; ++i)
            {
                if (selectedStatus[i])
                {
                    Vector3 position = path.points[i].position;
                    Quaternion orientation = Quaternion.LookRotation(path.points.GetTangent(i));

                    float offset = 0.05f;
                    float thickness = (path.thicknesses[i] * scale) + offset;
                    thickness = DoRadiusHandle(orientation, position, thickness);
                    path.thicknesses[i] = Mathf.Max(0,(thickness - offset) / scale);
                }
            }
            Handles.color = oldColor;

            if (EditorGUI.EndChangeCheck())
            {
                // TODO: add undo.
                path.FlushEvents();
                return true;
            }

            return false;
        }

        public void DrawUIWindow(int windowID)
        {

            DrawToolButtons();
           
            DrawControlPointInspector();

        }

        private void DrawToolButtons()
        {
            GUILayout.BeginHorizontal();

            EditorGUI.BeginChangeCheck();
            insertTool = GUILayout.Toggle(insertTool, new GUIContent(Resources.Load<Texture2D>("AddControlPoint"), "Add CPs"), "Button", GUILayout.MaxHeight(24), GUILayout.Width(42));
            if (EditorGUI.EndChangeCheck())
            {
                if (insertTool) removeTool = false;
            }

            EditorGUI.BeginChangeCheck();
            removeTool = GUILayout.Toggle(removeTool, new GUIContent(Resources.Load<Texture2D>("RemoveControlPoint"), "Remove CPs"), "Button", GUILayout.MaxHeight(24), GUILayout.Width(42));
            if (EditorGUI.EndChangeCheck())
            {
                if (removeTool) insertTool = false;
            }

            EditorGUI.BeginChangeCheck();
            bool closed = GUILayout.Toggle(path.Closed, new GUIContent(Resources.Load<Texture2D>("OpenCloseCurve"), "Open/Close the path"), "Button", GUILayout.MaxHeight(24), GUILayout.Width(42));
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(target, "Open/close path");
                path.Closed = closed;
                path.FlushEvents();
                needsRepaint = true;
            }

            if (useOrientation)
                orientTool = GUILayout.Toggle(orientTool, new GUIContent(Resources.Load<Texture2D>("OrientControlPoint"), "Orientation tool"), "Button", GUILayout.MaxHeight(24), GUILayout.Width(42));

            showTangentHandles = GUILayout.Toggle(showTangentHandles, new GUIContent(Resources.Load<Texture2D>("ShowTangentHandles"), "Orientation tool"), "Button", GUILayout.MaxHeight(24), GUILayout.Width(42));
            showThicknessHandles = GUILayout.Toggle(showThicknessHandles, new GUIContent(Resources.Load<Texture2D>("ShowThicknessHandles"), "Orientation tool"), "Button", GUILayout.MaxHeight(24), GUILayout.Width(42));

            GUILayout.EndHorizontal();
        }

        private void DrawControlPointInspector()
        {
            
            GUI.enabled = selectedCount > 0;
            EditorGUILayout.BeginVertical();

            GUILayout.Box("", ObiEditorUtils.GetSeparatorLineStyle());

            // tangent mode:
            EditorGUI.showMixedValue = false;
            var mode = ObiWingedPoint.TangentMode.Free;
            bool firstSelected = true;
            for (int i = 0; i < path.ControlPointCount; ++i)
            {
                if (selectedStatus[i])
                {
                    if (firstSelected)
                    {
                        mode = path.points[i].tangentMode;
                        firstSelected = false;
                    }
                    else if (mode != path.points[i].tangentMode)
                    {
                        EditorGUI.showMixedValue = true;
                        break;
                    }
                }
            }

            EditorGUI.BeginChangeCheck();
            var newMode = (ObiWingedPoint.TangentMode) EditorGUILayout.EnumPopup("Tangent mode",mode, GUI.skin.FindStyle("DropDown"), GUILayout.MinWidth(94));
            EditorGUI.showMixedValue = false;
            if (EditorGUI.EndChangeCheck())
            {

                Undo.RecordObject(target, "Change control points mode");

                for (int i = 0; i < path.ControlPointCount; ++i)
                {
                    if (selectedStatus[i])
                    {
                        var wp = path.points[i];
                        wp.tangentMode = newMode;
                        path.points[i] = wp;
                    }
                }
                path.FlushEvents();
                needsRepaint = true;
            }

            // thickness:
            EditorGUI.showMixedValue = false;
            float thickness = 0;
            firstSelected = true;
            for (int i = 0; i < path.ControlPointCount; ++i)
            {
                if (selectedStatus[i])
                {
                    if (firstSelected)
                    {
                        thickness = path.thicknesses[i];
                        firstSelected = false;
                    }
                    else if (!Mathf.Approximately(thickness,path.thicknesses[i]))
                    {
                        EditorGUI.showMixedValue = true;
                        break;
                    }
                }
            }

            EditorGUI.BeginChangeCheck();
            thickness = EditorGUILayout.FloatField("Thickness",thickness,GUILayout.MinWidth(94));
            EditorGUI.showMixedValue = false;
            if (EditorGUI.EndChangeCheck())
            {

                Undo.RecordObject(target, "Change control point thickness");

                for (int i = 0; i < path.ControlPointCount; ++i)
                {
                    if (selectedStatus[i])
                        path.thicknesses[i] = Mathf.Max(0,thickness);
                }
                path.FlushEvents();
                needsRepaint = true;
            }

            // mass:
            EditorGUI.showMixedValue = false;
            float mass = 0;
            firstSelected = true;
            for (int i = 0; i < path.ControlPointCount; ++i)
            {
                if (selectedStatus[i])
                {
                    if (firstSelected)
                    {
                        mass = path.masses[i];
                        firstSelected = false;
                    }
                    else if (!Mathf.Approximately(mass, path.masses[i]))
                    {
                        EditorGUI.showMixedValue = true;
                        break;
                    }
                }
            }

            EditorGUI.BeginChangeCheck();
            mass = EditorGUILayout.FloatField("Mass",mass,GUILayout.MinWidth(94));
            EditorGUI.showMixedValue = false;
            if (EditorGUI.EndChangeCheck())
            {

                Undo.RecordObject(target, "Change control point mass");

                for (int i = 0; i < path.ControlPointCount; ++i)
                {
                    if (selectedStatus[i])
                        path.masses[i] = mass;
                }
                path.FlushEvents();
                needsRepaint = true;
            }

            if (useOrientation)
            {
                // rotational mass:
                EditorGUI.showMixedValue = false;
                float rotationalMass = 0;
                firstSelected = true;
                for (int i = 0; i < path.ControlPointCount; ++i)
                {
                    if (selectedStatus[i])
                    {
                        if (firstSelected)
                        {
                            rotationalMass = path.rotationalMasses[i];
                            firstSelected = false;
                        }
                        else if (!Mathf.Approximately(rotationalMass, path.rotationalMasses[i]))
                        {
                            EditorGUI.showMixedValue = true;
                            break;
                        }
                    }
                }

                EditorGUI.BeginChangeCheck();
                rotationalMass = EditorGUILayout.FloatField("Rotational mass", rotationalMass, GUILayout.MinWidth(94));
                EditorGUI.showMixedValue = false;
                if (EditorGUI.EndChangeCheck())
                {

                    Undo.RecordObject(target, "Change control point rotational mass");

                    for (int i = 0; i < path.ControlPointCount; ++i)
                    {
                        if (selectedStatus[i])
                            path.rotationalMasses[i] = rotationalMass;
                    }
                    path.FlushEvents();
                    needsRepaint = true;
                }
            }

            // phase:
            EditorGUI.showMixedValue = false;
            int phase = 0;
            firstSelected = true;
            for (int i = 0; i < path.ControlPointCount; ++i)
            {
                if (selectedStatus[i])
                {
                    if (firstSelected)
                    {
                        phase = path.phases[i];
                        firstSelected = false;
                    }
                    else if (!Mathf.Approximately(phase, path.phases[i]))
                    {
                        EditorGUI.showMixedValue = true;
                        break;
                    }
                }
            }

            EditorGUI.BeginChangeCheck();
            phase = EditorGUILayout.IntField("Phase", phase, GUILayout.MinWidth(94));
            EditorGUI.showMixedValue = false;
            if (EditorGUI.EndChangeCheck())
            {

                Undo.RecordObject(target, "Change control point phase");

                for (int i = 0; i < path.ControlPointCount; ++i)
                {
                    if (selectedStatus[i])
                        path.phases[i] = phase;
                }
                path.FlushEvents();
                needsRepaint = true;
            }

            // color:
            EditorGUI.showMixedValue = false;
            Color color = Color.white;
            firstSelected = true;
            for (int i = 0; i < path.ControlPointCount; ++i)
            {
                if (selectedStatus[i])
                {
                    if (firstSelected)
                    {
                        color = path.colors[i];
                        firstSelected = false;
                    }
                    else if (color != path.colors[i])
                    {
                        EditorGUI.showMixedValue = true;
                        break;
                    }
                }
            }

            EditorGUI.BeginChangeCheck();
            color = EditorGUILayout.ColorField("Color",color,GUILayout.MinWidth(94));
            EditorGUI.showMixedValue = false;
            if (EditorGUI.EndChangeCheck())
            {

                Undo.RecordObject(target, "Change control point color");

                for (int i = 0; i < path.ControlPointCount; ++i)
                {
                    if (selectedStatus[i])
                        path.colors[i] = color;
                }
                path.FlushEvents();
                needsRepaint = true;
            }

            // name:
            EditorGUI.showMixedValue = false;
            string name = "";
            firstSelected = true;
            for (int i = 0; i < path.ControlPointCount; ++i)
            {
                if (selectedStatus[i])
                {
                    if (firstSelected)
                    {
                        name = path.GetName(i);
                        firstSelected = false;
                    }
                    else if (name != path.GetName(i))
                    {
                        EditorGUI.showMixedValue = true;
                        break;
                    }
                }
            }

            EditorGUI.BeginChangeCheck();
            name = EditorGUILayout.DelayedTextField("Name",name,GUILayout.MinWidth(94));
            EditorGUI.showMixedValue = false;
            if (EditorGUI.EndChangeCheck())
            {

                Undo.RecordObject(target, "Change control point name");

                for (int i = 0; i < path.ControlPointCount; ++i)
                {
                    if (selectedStatus[i])
                        path.SetName(i, name);
                }
                path.FlushEvents();
                needsRepaint = true;
            }


            EditorGUILayout.EndVertical();

            GUI.enabled = true;
        }

        internal static float DoRadiusHandle(Quaternion rotation, Vector3 position, float radius)
        {
            Vector3[] vector3Array;

            Vector3 camToPosition;
            if (Camera.current.orthographic)
            {
                camToPosition = Camera.current.transform.forward;
                Handles.DrawWireDisc(position, camToPosition, radius);

                vector3Array = new Vector3[4]
                {
                    Camera.current.transform.right,
                    Camera.current.transform.up,
                    -Camera.current.transform.right,
                    -Camera.current.transform.up,
                };

            }else{
                camToPosition = position - Camera.current.transform.position;
                Handles.DrawWireDisc(position, rotation * Vector3.forward, radius);

                vector3Array = new Vector3[4]
                {
                    rotation * Vector3.right,
                    rotation * Vector3.up,
                    rotation * -Vector3.right,
                    rotation * -Vector3.up,
                };
            }

            for (int index = 0; index < 4; ++index)
            {
                int controlId = GUIUtility.GetControlID("ObiPathThicknessHandle".GetHashCode(), FocusType.Keyboard);
                Vector3 position1 = position + radius * vector3Array[index];
                bool changed = GUI.changed;
                GUI.changed = false;
                Vector3 a = Handles.Slider(controlId, position1, vector3Array[index], HandleUtility.GetHandleSize(position1) * 0.03f, Handles.DotHandleCap, 0.0f);
                if (GUI.changed)
                    radius = Vector3.Distance(a, position);
                GUI.changed |= changed;
            }

            return radius;
        }

        public static float ScreenPointToCurveMu(ObiPath path, Vector2 screenPoint, Matrix4x4 referenceFrame, int samples = 30)
        {

            if (path.ControlPointCount >= 2)
            {

                samples = Mathf.Max(1, samples);
                float step = 1 / (float)samples;

                float closestMu = 0;
                float minDistance = float.MaxValue;

                for (int k = 0; k < path.GetSpanCount(); ++k)
                {
                    int nextCP = (k + 1) % path.ControlPointCount;

                    var wp1 = path.points[k];
                    var wp2 = path.points[nextCP];

                    Vector3 _p = referenceFrame.MultiplyPoint3x4(wp1.position);
                    Vector3 p = referenceFrame.MultiplyPoint3x4(wp1.outTangentEndpoint);
                    Vector3 p_ = referenceFrame.MultiplyPoint3x4(wp2.inTangentEndpoint);
                    Vector3 p__ = referenceFrame.MultiplyPoint3x4(wp2.position);

                    Vector2 lastPoint = HandleUtility.WorldToGUIPoint(path.m_Points.Evaluate(_p, p, p_, p__, 0));
                    for (int i = 1; i <= samples; ++i)
                    {

                        Vector2 currentPoint = HandleUtility.WorldToGUIPoint(path.m_Points.Evaluate(_p, p, p_, p__, i * step));

                        float mu;
                        float distance = Vector2.SqrMagnitude((Vector2)ObiUtils.ProjectPointLine(screenPoint, lastPoint, currentPoint, out mu) - screenPoint);

                        if (distance < minDistance)
                        {
                            minDistance = distance;
                            closestMu = (k + (i - 1) * step + mu / samples) / (float)path.GetSpanCount();
                        }
                        lastPoint = currentPoint;
                    }

                }

                return closestMu;

            }
            else
            {
                Debug.LogWarning("Curve needs at least 2 control points to be defined.");
            }
            return 0;

        }

    }
}