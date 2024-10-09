using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.Collections;
using System;

namespace Obi
{
    [CustomEditor(typeof(ObiRopeBlueprintBase), true)]
    public class ObiRopeBaseBlueprintEditor : ObiActorBlueprintEditor
    {

        public override void OnEnable()
        {
            base.OnEnable();
            Undo.undoRedoPerformed += UndoRedoPerformed;
        }

        public override void OnDisable()
        {
            base.OnDisable();
            Undo.undoRedoPerformed -= UndoRedoPerformed;
        }

        void UndoRedoPerformed()
        {
            // Re-generate the blueprint after undo/redo.
            if (blueprint != null)
                blueprint.GenerateImmediate();
        }

        public override void OnInspectorGUI()
        {

            serializedObject.UpdateIfRequiredOrScript();

            EditorGUILayout.BeginVertical(EditorStyles.inspectorDefaultMargins);
            Editor.DrawPropertiesExcluding(serializedObject, "m_Script");

            EditorGUILayout.EndVertical();

            if (GUI.changed)
            {
                serializedObject.ApplyModifiedProperties();

                // Re-generate the blueprint if any element has been changed.
                if (blueprint != null)
                    blueprint.GenerateImmediate();

                // There might be blueprint editing operations that have no undo entry, so do this to 
                // ensure changes are serialized to disk by Unity.
                EditorUtility.SetDirty(this);
            }
        }
    }


}
