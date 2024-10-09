using UnityEditor;
using UnityEditorInternal;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;


namespace Obi
{

    [CustomEditor(typeof(ObiParticleAttachment))]
    public class ObiParticleAttachmentEditor : Editor
    {

        SerializedProperty targetTransform;
        SerializedProperty particleGroup;
        SerializedProperty attachmentType;
        SerializedProperty constrainOrientation;
        SerializedProperty compliance;
        SerializedProperty breakThreshold;
        Rect groupDropdownRect;

        ObiParticleAttachment attachment;

        public void OnEnable()
        {

            attachment = target as ObiParticleAttachment;
            targetTransform = serializedObject.FindProperty("m_Target");
            particleGroup = serializedObject.FindProperty("m_ParticleGroup");
            attachmentType = serializedObject.FindProperty("m_AttachmentType");
            constrainOrientation = serializedObject.FindProperty("m_ConstrainOrientation");
            compliance = serializedObject.FindProperty("m_Compliance");
            breakThreshold = serializedObject.FindProperty("m_BreakThreshold");
        }

        public override void OnInspectorGUI()
        {

            serializedObject.UpdateIfRequiredOrScript();

            // warn about incorrect setups:
            if (!attachmentType.hasMultipleDifferentValues && !targetTransform.hasMultipleDifferentValues)
            {
                if (attachmentType.enumValueIndex == (int)ObiParticleAttachment.AttachmentType.Dynamic)
                {
                    var targetValue = targetTransform.objectReferenceValue as UnityEngine.Component;
                    if (targetValue != null)
                    {
                        var collider = targetValue.GetComponent<ObiColliderBase>();
                        if (collider == null)
                        {
                            EditorGUILayout.HelpBox("Dynamic attachments require the target object to have a ObiCollider component. Either add one, or change the attachment type to Static.", MessageType.Warning);
                        }
                    }
                }
            }

            EditorGUILayout.PropertyField(targetTransform, new GUIContent("Target"));
            var blueprint = attachment.actor.sourceBlueprint;

            if (blueprint != null)
            {
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.PrefixLabel("Particle group");

                if (GUILayout.Button(attachment.particleGroup != null ? attachment.particleGroup.name : "None", EditorStyles.popup))
                {
                    // create the menu and add items to it
                    GenericMenu menu = new GenericMenu();
                    menu.allowDuplicateNames = true;

                    for (int i = 0; i < blueprint.groups.Count; ++i)
                    {
                        menu.AddItem(new GUIContent(blueprint.groups[i].name), blueprint.groups[i] == attachment.particleGroup, OnParticleGroupSelected, blueprint.groups[i]);
                    }

                    // display the menu
                    menu.DropDown(groupDropdownRect);
                }

                if (Event.current.type == EventType.Repaint)
                    groupDropdownRect = GUILayoutUtility.GetLastRect();

                EditorGUILayout.EndHorizontal();
            }

            EditorGUILayout.PropertyField(attachmentType, new GUIContent("Type"));

            if (attachment.actor.usesOrientedParticles)
                EditorGUILayout.PropertyField(constrainOrientation, new GUIContent("Constraint Orientation"));

            if (attachment.attachmentType == ObiParticleAttachment.AttachmentType.Dynamic)
            {
                EditorGUILayout.PropertyField(compliance, new GUIContent("Compliance"));
                EditorGUILayout.PropertyField(breakThreshold, new GUIContent("Break threshold"));
            }

            if (GUI.changed)
                serializedObject.ApplyModifiedProperties();

        }

        // the GenericMenu.MenuFunction2 event handler for when a menu item is selected
        void OnParticleGroupSelected(object index)
        {
            Undo.RecordObject(attachment, "Set particle group");
            attachment.particleGroup = index as ObiParticleGroup;
        }
    }

}


