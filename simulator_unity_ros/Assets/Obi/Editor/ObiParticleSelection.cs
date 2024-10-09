using UnityEngine;
using UnityEditor;
using System;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{

    public static class ObiParticleSelection
    {

        static int particleSelectorHash = "ObiParticleSelectorHash".GetHashCode();

        static Vector2 startPos;
        static Vector2 currentPos;
        static bool dragging = false;
        static Rect marquee;

        public static bool DoSelection(Vector3[] positions,
                                            bool[] selectionStatus,
                                            bool[] facingCamera)
        {

            Matrix4x4 cachedMatrix = Handles.matrix;

            int controlID = GUIUtility.GetControlID(particleSelectorHash, FocusType.Passive);
            int selectedParticleIndex = -1;
            bool selectionStatusChanged = false;

            // select vertex on mouse click:
            switch (Event.current.GetTypeForControl(controlID))
            {

                case EventType.MouseDown:

                    if (Event.current.button != 0) break;

                    startPos = Event.current.mousePosition;
                    marquee.Set(0, 0, 0, 0);

                    // If the user is not pressing shift, clear selection.
                    if ((Event.current.modifiers & EventModifiers.Shift) == 0 && (Event.current.modifiers & EventModifiers.Alt) == 0)
                    {
                        for (int i = 0; i < selectionStatus.Length; i++)
                            selectionStatus[i] = false;
                    }

                    // Allow use of marquee selection
                    if (Event.current.modifiers == EventModifiers.None || (Event.current.modifiers & EventModifiers.Shift) != 0)
                        GUIUtility.hotControl = controlID;

                    float minSqrDistance = System.Single.MaxValue;

                    for (int i = 0; i < positions.Length; i++)
                    {
                        // skip not selectable particles:
                        //if (!facingCamera[i] && (selectBackfaces & ObiActorBlueprintEditor.ParticleCulling.Back) != 0) continue;
                        //if (facingCamera[i] && (selectBackfaces & ObiActorBlueprintEditor.ParticleCulling.Front) != 0) continue;

                        // get particle position in gui space:
                        Vector2 pos = HandleUtility.WorldToGUIPoint(positions[i]);

                        // get distance from mouse position to particle position:
                        float sqrDistance = Vector2.SqrMagnitude(startPos - pos);

                        // check if this particle is closer to the cursor that any previously considered particle.
                        if (sqrDistance < 100 && sqrDistance < minSqrDistance)
                        { //magic number 100 = 10*10, where 10 is min distance in pixels to select a particle.
                            minSqrDistance = sqrDistance;
                            selectedParticleIndex = i;
                        }

                    }

                    if (selectedParticleIndex >= 0)
                    { // toggle particle selection status.

                        selectionStatus[selectedParticleIndex] = !selectionStatus[selectedParticleIndex];
                        selectionStatusChanged = true;
                        GUIUtility.hotControl = controlID;
                        Event.current.Use();

                    }
                    else if (Event.current.modifiers == EventModifiers.None)
                    { // deselect all particles:
                        for (int i = 0; i < selectionStatus.Length; i++)
                            selectionStatus[i] = false;

                        selectionStatusChanged = true;
                    }

                    break;

                case EventType.MouseMove:
                    SceneView.RepaintAll();
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

                        for (int i = 0; i < positions.Length; i++)
                        {

                            // skip not selectable particles:
                            //switch (selectBackfaces)
                            {
                                //case ObiActorBlueprintEditor.ParticleCulling.Back: if (!facingCamera[i]) continue; break;
                                //case ObiActorBlueprintEditor.ParticleCulling.Front: if (facingCamera[i]) continue; break;
                            }

                            // get particle position in gui space:
                            Vector2 pos = HandleUtility.WorldToGUIPoint(positions[i]);

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

                    Handles.matrix = Matrix4x4.identity;

                    if (dragging)
                    {
                        GUISkin oldSkin = GUI.skin;
                        GUI.skin = EditorGUIUtility.GetBuiltinSkin(EditorSkin.Scene);
                        Handles.BeginGUI();
                        GUI.Box(new Rect(marquee.xMin, marquee.yMin, marquee.width, marquee.height), "");
                        Handles.EndGUI();
                        GUI.skin = oldSkin;
                    }

                    Handles.matrix = cachedMatrix;

                    break;

            }

            return selectionStatusChanged;
        }

    }
}

