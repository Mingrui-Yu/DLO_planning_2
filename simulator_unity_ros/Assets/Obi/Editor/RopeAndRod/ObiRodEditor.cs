using UnityEditor;
using UnityEditorInternal;
using UnityEngine;
using System;
using System.Collections.Generic;
using System.Linq;


namespace Obi
{

    [CustomEditor(typeof(ObiRod))]
    public class ObiRodEditor : Editor
    {
        [MenuItem("GameObject/3D Object/Obi/Obi Rod", false, 301)]
        static void CreateObiRod(MenuCommand menuCommand)
        {
            GameObject go = new GameObject("Obi Rod", typeof(ObiRod), typeof(ObiRopeExtrudedRenderer));
            ObiEditorUtils.PlaceActorRoot(go, menuCommand);
        }

        ObiRod actor;
        ObiPathEditor pathEditor;

        SerializedProperty rodBlueprint;

        SerializedProperty collisionMaterial;
        SerializedProperty selfCollisions;

        SerializedProperty stretchShearConstraintsEnabled;
        SerializedProperty stretchCompliance;
        SerializedProperty shear1Compliance;
        SerializedProperty shear2Compliance;

        SerializedProperty bendTwistConstraintsEnabled;
        SerializedProperty torsionCompliance;
        SerializedProperty bend1Compliance;
        SerializedProperty bend2Compliance;

        SerializedProperty chainConstraintsEnabled;
        SerializedProperty tightness;

        protected bool editMode = false;

        GUIStyle editLabelStyle;

        public void OnEnable()
        {
            actor = (ObiRod)target;

            rodBlueprint = serializedObject.FindProperty("m_RodBlueprint");

            collisionMaterial = serializedObject.FindProperty("m_CollisionMaterial");
            selfCollisions = serializedObject.FindProperty("m_SelfCollisions");

            stretchShearConstraintsEnabled = serializedObject.FindProperty("_stretchShearConstraintsEnabled");
            stretchCompliance = serializedObject.FindProperty("_stretchCompliance");
            shear1Compliance = serializedObject.FindProperty("_shear1Compliance");
            shear2Compliance = serializedObject.FindProperty("_shear2Compliance");

            bendTwistConstraintsEnabled = serializedObject.FindProperty("_bendTwistConstraintsEnabled");
            torsionCompliance = serializedObject.FindProperty("_torsionCompliance");
            bend1Compliance = serializedObject.FindProperty("_bend1Compliance");
            bend2Compliance = serializedObject.FindProperty("_bend2Compliance");

            chainConstraintsEnabled = serializedObject.FindProperty("_chainConstraintsEnabled");
            tightness = serializedObject.FindProperty("_tightness");
        }

        public void OnDisable()
        {
            Tools.hidden = false;
        }

        private void DoEditButton()
        {
            using (new EditorGUI.DisabledScope(actor.rodBlueprint == null))
            {
                if (!GUI.enabled)
                    Tools.hidden = editMode = false;

                EditorGUILayout.BeginHorizontal();
                GUILayout.Space(EditorGUIUtility.labelWidth);
                EditorGUI.BeginChangeCheck();
                Tools.hidden = editMode = GUILayout.Toggle(editMode, new GUIContent(Resources.Load<Texture2D>("EditCurves")), "Button", GUILayout.MaxWidth(36), GUILayout.MaxHeight(24));
                EditorGUILayout.LabelField("Edit path", editLabelStyle, GUILayout.ExpandHeight(true), GUILayout.MaxHeight(24));
                if (EditorGUI.EndChangeCheck())
                {
                    SceneView.RepaintAll();
                }
                EditorGUILayout.EndHorizontal();
            }
        }

        public override void OnInspectorGUI()
        {
            if (actor.rodBlueprint != null && pathEditor == null)
                pathEditor = new ObiPathEditor(actor.rodBlueprint, actor.rodBlueprint.path, true);

            if (editLabelStyle == null)
            {
                editLabelStyle = new GUIStyle(GUI.skin.label);
                editLabelStyle.alignment = TextAnchor.MiddleLeft;
            }

            serializedObject.UpdateIfRequiredOrScript();

            if (pathEditor != null)
                pathEditor.ResizeCPArrays();

            using (new EditorGUI.DisabledScope(editMode))
            {
                EditorGUI.BeginChangeCheck();
                EditorGUILayout.PropertyField(rodBlueprint, new GUIContent("Blueprint"));
                if (EditorGUI.EndChangeCheck())
                {
                    foreach (var t in targets)
                    {
                        (t as ObiRod).RemoveFromSolver();
                        (t as ObiRod).ClearState();
                    }
                    serializedObject.ApplyModifiedProperties();
                    foreach (var t in targets)
                        (t as ObiRod).AddToSolver();
                }
            }

            DoEditButton();


            if (actor.rodBlueprint != null && actor.rodBlueprint.path.ControlPointCount < 2)
            {
                actor.rodBlueprint.GenerateImmediate();
            }

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Collisions", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(collisionMaterial, new GUIContent("Collision material"));
            EditorGUILayout.PropertyField(selfCollisions, new GUIContent("Self collisions"));

            EditorGUILayout.Space();
            ObiEditorUtils.DoToggleablePropertyGroup(stretchShearConstraintsEnabled, new GUIContent("StretchShear Constraints", Resources.Load<Texture2D>("Icons/ObiStretchShearConstraints Icon")),
            () => {
                EditorGUILayout.PropertyField(stretchCompliance, new GUIContent("Stretch compliance"));
                EditorGUILayout.PropertyField(shear1Compliance, new GUIContent("Shear compliance 1"));
                EditorGUILayout.PropertyField(shear2Compliance, new GUIContent("Shear compliance 2"));
            });

            ObiEditorUtils.DoToggleablePropertyGroup(bendTwistConstraintsEnabled, new GUIContent("Bend Twist Constraints", Resources.Load<Texture2D>("Icons/ObiBendTwistConstraints Icon")),
            () => {
                EditorGUILayout.PropertyField(torsionCompliance, new GUIContent("Torsion compliance"));
                EditorGUILayout.PropertyField(bend1Compliance, new GUIContent("Bend compliance 1"));
                EditorGUILayout.PropertyField(bend2Compliance, new GUIContent("Bend compliance 2"));
            });

            ObiEditorUtils.DoToggleablePropertyGroup(chainConstraintsEnabled, new GUIContent("Chain Constraints", Resources.Load<Texture2D>("Icons/ObiChainConstraints Icon")),
            () => {
                EditorGUILayout.PropertyField(tightness, new GUIContent("Tightness"));
            });


            if (GUI.changed)
                serializedObject.ApplyModifiedProperties();

        }

        public void OnSceneGUI()
        {
            if (!editMode || actor.rodBlueprint == null || actor.rodBlueprint.path.ControlPointCount < 2)
                return;

            if (pathEditor.OnSceneGUI(actor.rodBlueprint.thickness, actor.transform.localToWorldMatrix))
            {
                Repaint();
                pathEditor.needsRepaint = false;
            }
        }

        [DrawGizmo(GizmoType.Selected)]
        private static void DrawGizmos(ObiRod actor, GizmoType gizmoType)
        {
            Handles.color = Color.white;
            if (actor.rodBlueprint != null)
                ObiPathHandles.DrawPathHandle(actor.rodBlueprint.path, actor.transform.localToWorldMatrix, actor.rodBlueprint.thickness ,20);
        }

    }

}


