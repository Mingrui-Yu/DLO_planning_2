using System;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEditor;
using UnityEditor.SceneManagement;
using System.IO;

namespace Obi{

    public static class ObiEditorUtils
	{
        static GUIStyle separatorLine;
        static GUIStyle toggleablePropertyGroup;
        static GUIStyle boldToggle;

        public static GUIStyle GetSeparatorLineStyle()
        {
            if (separatorLine == null)
            {
                separatorLine = new GUIStyle(EditorGUIUtility.GetBuiltinSkin(EditorSkin.Scene).box);
                separatorLine.normal.background = Resources.Load<Texture2D>("SeparatorLine");
                separatorLine.border = new RectOffset(3, 3, 0, 0);
                separatorLine.padding = new RectOffset(0, 0, 0, 0);
                separatorLine.margin = new RectOffset(0, 0, 0, 0);
                separatorLine.fixedHeight = 3;
                separatorLine.stretchWidth = true;
            }
            return separatorLine;
        }

        public static GUIStyle GetToggleablePropertyGroupStyle()
        {
            if (toggleablePropertyGroup == null)
            {
                toggleablePropertyGroup = new GUIStyle();
                toggleablePropertyGroup.normal.background = Resources.Load<Texture2D>("ToggleableGroupBg"); 
                toggleablePropertyGroup.border = new RectOffset(3, 3, 3, 3);
                toggleablePropertyGroup.padding = new RectOffset(0, 0, 0, 0);
                toggleablePropertyGroup.margin = new RectOffset(0, 0, 3, 3);
            }
            return toggleablePropertyGroup;
        }

        public static GUIStyle GetBoldToggleStyle()
        {
            if (boldToggle == null)
            {
                boldToggle = new GUIStyle(EditorStyles.toggle);
                boldToggle.fontStyle = FontStyle.Bold;
            }
            return boldToggle;
        }

        public static void SaveMesh (Mesh mesh, string title, string name, bool makeNewInstance = true, bool optimizeMesh = true) {

			string path = EditorUtility.SaveFilePanel(title, "Assets/", name, "asset");
			if (string.IsNullOrEmpty(path)) return;
	        
			path = FileUtil.GetProjectRelativePath(path);

			Mesh meshToSave = (makeNewInstance) ? GameObject.Instantiate(mesh) as Mesh : mesh;

			if (optimizeMesh)
			     MeshUtility.Optimize(meshToSave);
	        
			AssetDatabase.CreateAsset(meshToSave, path);
			AssetDatabase.SaveAssets();
		}

        public static void PlaceActorRoot(GameObject element, MenuCommand menuCommand)
        {
            GameObject parent = menuCommand.context as GameObject;

            if (parent == null)
            {
                parent = GetOrCreateSolverObject();
            }

            if (parent.GetComponentsInParent<ObiSolver>(true).Length == 0)
            {
                // Create solver under context GameObject,
                // and make that be the parent which actor is added under.
                GameObject solver = CreateNewSolver();
                solver.transform.SetParent(parent.transform, false);
                parent = solver;
            }

            // The element needs to be already in its destination scene when the
            // RegisterCreatedObjectUndo is performed; otherwise the scene it was created in is dirtied.
            SceneManager.MoveGameObjectToScene(element, parent.scene);

            Undo.RegisterCreatedObjectUndo(element, "Create " + element.name);

            if (element.transform.parent == null)
                Undo.SetTransformParent(element.transform, parent.transform, "Parent " + element.name);

            GameObjectUtility.EnsureUniqueNameForSibling(element);

            // We have to fix up the undo name since the name of the object was only known after reparenting it.
            Undo.SetCurrentGroupName("Create " + element.name);

            GameObjectUtility.SetParentAndAlign(element, parent);
            Selection.activeGameObject = element;
        }

        // Helper function that returns a Solver GameObject; preferably a parent of the selection, or other existing Canvas.
        private static GameObject GetOrCreateSolverObject()
        {
            GameObject selectedGo = Selection.activeGameObject;

            // Try to find a gameobject that is the selected GO or one if its parents.
            ObiSolver solver = (selectedGo != null) ? selectedGo.GetComponentInParent<ObiSolver>() : null;
            if (IsValidSolver(solver))
                return solver.gameObject;

            // No solver in selection or its parents? Then use any valid solver.
            // We have to find all loaded solvers, not just the ones in main scenes.
            ObiSolver[] solverArray = StageUtility.GetCurrentStageHandle().FindComponentsOfType<ObiSolver>();
            for (int i = 0; i < solverArray.Length; i++)
                if (IsValidSolver(solverArray[i]))
                    return solverArray[i].gameObject;

            // No solver in the scene at all? Then create a new one.
            return CreateNewSolver();
        }

        public static GameObject CreateNewSolver()
        {
            // Root for the actors.
            var root = new GameObject("Obi Solver");
            ObiSolver solver = root.AddComponent<ObiSolver>();

            // Try to find a fixed updater in the scene (though other kinds of updaters can exist, updating in FixedUpdate is the preferred option).
            ObiFixedUpdater updater = StageUtility.GetCurrentStageHandle().FindComponentOfType<ObiFixedUpdater>();
            // If we could not find an fixed updater in the scene, add one to the solver object.
            if (updater == null)
                updater = root.AddComponent<ObiFixedUpdater>();

            // Add the solver to the updater:
            updater.solvers.Add(solver);

            // Works for all stages.
            StageUtility.PlaceGameObjectInCurrentStage(root);
            Undo.RegisterCreatedObjectUndo(root, "Create " + root.name);

            return root;
        }

        static bool IsValidSolver(ObiSolver solver)
        {
            if (solver == null || !solver.gameObject.activeInHierarchy)
                return false;

            if (EditorUtility.IsPersistent(solver) || (solver.hideFlags & HideFlags.HideInHierarchy) != 0)
                return false;

            if (StageUtility.GetStageHandle(solver.gameObject) != StageUtility.GetCurrentStageHandle())
                return false;

            return true;
        }

        public static void DoPropertyGroup(GUIContent content, System.Action action)
        {
            EditorGUILayout.BeginVertical(GetToggleablePropertyGroupStyle());
            {
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField(content, EditorStyles.boldLabel); 
                EditorGUILayout.EndHorizontal();

                if (action != null)
                {
                    EditorGUI.indentLevel++;
                    action();
                    EditorGUI.indentLevel--;
                }
            }
            EditorGUILayout.EndVertical();
        }

        public static void DoToggleablePropertyGroup(SerializedProperty enabledProperty, GUIContent content, System.Action action)
        {
            bool enabled = GUI.enabled;
            GUI.enabled &= enabledProperty.boolValue;
            EditorGUILayout.BeginVertical(GetToggleablePropertyGroupStyle());
            GUI.enabled = enabled;
            {
                EditorGUILayout.BeginHorizontal();
                    enabledProperty.boolValue = EditorGUILayout.ToggleLeft(content,enabledProperty.boolValue,EditorStyles.boldLabel);
                EditorGUILayout.EndHorizontal();

                if (enabledProperty.boolValue && action != null)
                {
                    EditorGUI.indentLevel++;
                    action();
                    EditorGUI.indentLevel--;
                }
            }
            EditorGUILayout.EndVertical();
        }

        public static int DoToolBar(int selected, GUIContent[] items)
        {
            // Keep the selected index within the bounds of the items array
            selected = selected < 0 ? 0 : selected >= items.Length ? items.Length - 1 : selected;

            GUIStyle style = GUI.skin.FindStyle("Button");

            EditorGUILayout.BeginHorizontal();
            GUILayout.FlexibleSpace();
            for (int i = 0; i < items.Length; i++)
            {
                if (i == 0 && items.Length > 1)
                    style = GUI.skin.FindStyle("ButtonLeft");
                else if (items.Length > 1 && i == items.Length-1)
                    style = GUI.skin.FindStyle("ButtonRight");
                else if (i > 0)
                    style = GUI.skin.FindStyle("ButtonMid");
                    

                // Display toggle. Get if toggle changed.
                bool change = GUILayout.Toggle(selected == i, items[i],style,GUILayout.Height(24));
                // If changed, set selected to current index.
                if (change)
                    selected = i;
            }
            GUILayout.FlexibleSpace();
            EditorGUILayout.EndHorizontal();

            // Return the currently selected item's index
            return selected;
        }
	}
}


