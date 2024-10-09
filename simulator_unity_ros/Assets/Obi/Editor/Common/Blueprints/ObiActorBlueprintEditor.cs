using UnityEngine;
using UnityEngine.Rendering;
using UnityEditor;
using UnityEngine.SceneManagement;
using UnityEditor.SceneManagement;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Linq;

namespace Obi
{

    [CustomEditor(typeof(ObiActorBlueprint), true)]
    public class ObiActorBlueprintEditor : Editor, IObiSelectableParticleProvider
    {
        protected IEnumerator routine;

        public List<ObiBlueprintEditorTool> tools = new List<ObiBlueprintEditorTool>();
        public int currentToolIndex = 0;

        protected List<ObiBlueprintPropertyBase> properties = new List<ObiBlueprintPropertyBase>();
        public int currentPropertyIndex = 0;

        protected List<ObiBlueprintRenderMode> renderModes = new List<ObiBlueprintRenderMode>();
        public int renderModeFlags = 0;

        public bool editMode = false;
        public bool isEditing = false;
        protected List<SceneStateCache> m_SceneStates;
        protected SceneSetup[] oldSetup;
        protected UnityEngine.Object oldSelection;

        //Additional status info for all particles:
        public int selectedCount = 0;
        public bool[] selectionStatus = new bool[0];
        public bool[] visible = new bool[0];
        protected float[] sqrDistanceToCamera = new float[0];
        public int[] sortedIndices = new int[0];

        public ObiActorBlueprint blueprint
        {
            get { return target as ObiActorBlueprint; }
        }

        public ObiBlueprintPropertyBase currentProperty
        {
            get { return (properties.Count > currentPropertyIndex && currentPropertyIndex >= 0) ? properties[currentPropertyIndex] : null; }
        }

        public ObiBlueprintEditorTool currentTool
        {
            get { return (tools.Count > currentToolIndex && currentToolIndex >= 0) ? tools[currentToolIndex] : null; }
        }

        public override bool UseDefaultMargins()
        {
            return false;
        }

#if (UNITY_2019_1_OR_NEWER)
        System.Action<ScriptableRenderContext, Camera> renderCallback;
#endif

        public virtual void OnEnable()
        {
            properties.Add(new ObiBlueprintMass(this));
            properties.Add(new ObiBlueprintRadius(this));
            properties.Add(new ObiBlueprintLayer(this));

            renderModes.Add(new ObiBlueprintRenderModeParticles(this));

#if (UNITY_2019_1_OR_NEWER)
            renderCallback = new System.Action<ScriptableRenderContext, Camera>((cntxt, cam) => { DrawWithCamera(cam); });
            RenderPipelineManager.beginCameraRendering -= renderCallback;
            RenderPipelineManager.beginCameraRendering += renderCallback;
#endif
            Camera.onPreCull -= DrawWithCamera;
            Camera.onPreCull += DrawWithCamera;
        }

        public virtual void OnDisable()
        {
            ExitBlueprintEditMode();

#if (UNITY_2019_1_OR_NEWER)
            RenderPipelineManager.beginCameraRendering -= renderCallback;
#endif
            Camera.onPreCull -= DrawWithCamera;
            
            ObiParticleEditorDrawing.DestroyParticlesMesh();

            foreach (var tool in tools)
            {
                tool.OnDisable();
                tool.OnDestroy();
            }

            foreach (var renderMode in renderModes)
            {
                renderMode.OnDestroy();
            }

            properties.Clear();
            renderModes.Clear();
        }

        protected void Generate()
        {
            if (blueprint.empty)
            {
                EditorUtility.SetDirty(target);
                CoroutineJob job = new CoroutineJob();
                routine = job.Start(blueprint.Generate());
                EditorCoroutine.ShowCoroutineProgressBar("Generating blueprint...", ref routine);
                EditorGUIUtility.ExitGUI();
            }
            else
            {
                if (EditorUtility.DisplayDialog("Blueprint generation", "This blueprint already contains data. Are you sure you want to re-generate this blueprint from scratch?", "Ok", "Cancel"))
                {
                    EditorUtility.SetDirty(target);
                    CoroutineJob job = new CoroutineJob();
                    routine = job.Start(blueprint.Generate());
                    EditorCoroutine.ShowCoroutineProgressBar("Generating blueprint...", ref routine);
                    EditorGUIUtility.ExitGUI();
                }
            }
        }

        protected virtual bool ValidateBlueprint() { return true; }

        public override void OnInspectorGUI()
        {

            serializedObject.UpdateIfRequiredOrScript();

            EditorGUILayout.BeginVertical(EditorStyles.inspectorDefaultMargins);
            Editor.DrawPropertiesExcluding(serializedObject, "m_Script");

            GUI.enabled = ValidateBlueprint();
            if (GUILayout.Button("Generate", GUI.skin.FindStyle("LargeButton"), GUILayout.Height(32)))
                Generate();

            GUI.enabled = (blueprint != null && !blueprint.empty && !Application.isPlaying);
            EditorGUI.BeginChangeCheck();
            editMode = GUILayout.Toggle(editMode, editMode ? "Done" : "Edit", "Button");
            if (EditorGUI.EndChangeCheck())
            {
                if (editMode)
                    EditorApplication.delayCall += EnterBlueprintEditMode;
                else
                    EditorApplication.delayCall += ExitBlueprintEditMode;
            }
            EditorGUILayout.EndVertical();
            GUI.enabled = true;

            if (isEditing)
                DrawTools();

            if (GUI.changed)
            {
                serializedObject.ApplyModifiedProperties();

                // There might be blueprint editing operations that have no undo entry, so do this to 
                // ensure changes are serialized to disk by Unity.
                EditorUtility.SetDirty(target);
            }

        }

        private void DrawWithCamera(Camera camera)
        {
            if (editMode)
            {
                for (int i = 0; i < renderModes.Count; ++i)
                {
                    if ((1 << i & renderModeFlags) != 0)
                        renderModes[i].DrawWithCamera(camera);
                }
            }
        }


        [System.Serializable]
        protected class SceneStateCache
        {
            public SceneView view;
            public SceneView.SceneViewState state;
        }

        void EnterBlueprintEditMode()
        {
            if (!isEditing)
            {
#if (UNITY_2019_1_OR_NEWER)
                SceneView.duringSceneGui -= this.OnSceneGUI;
                SceneView.duringSceneGui += this.OnSceneGUI;
#else
                SceneView.onSceneGUIDelegate -= this.OnSceneGUI;
                SceneView.onSceneGUIDelegate += this.OnSceneGUI;
#endif

                oldSelection = Selection.activeObject;
                if (EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
                {
                    ActiveEditorTracker.sharedTracker.isLocked = true;

                    oldSetup = EditorSceneManager.GetSceneManagerSetup();
                    EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects);

                    // Set properties for all scene views:
                    m_SceneStates = new List<SceneStateCache>();
                    foreach (SceneView s in SceneView.sceneViews)
                    {
                        m_SceneStates.Add(new SceneStateCache { state = new SceneView.SceneViewState(s.sceneViewState), view = s });
                        s.sceneViewState.showFlares = false;
                        s.sceneViewState.alwaysRefresh = false;
                        s.sceneViewState.showFog = false;
                        s.sceneViewState.showSkybox = false;
                        s.sceneViewState.showImageEffects = false;
                        s.sceneViewState.showParticleSystems = false;
                        s.Frame(blueprint.bounds);
                    }
                    
                    isEditing = true;
                    Repaint();
                }
            }
        }

        void ExitBlueprintEditMode()
        {
            if (isEditing)
            {

                isEditing = false;

                AssetDatabase.SaveAssets();

                // Reset all scene views:
                foreach (var state in m_SceneStates)
                {
                    if (state.view == null)
                        continue;

                    state.view.sceneViewState.showFog = state.state.showFog;
                    state.view.sceneViewState.showFlares = state.state.showFlares;
                    state.view.sceneViewState.alwaysRefresh = state.state.alwaysRefresh;
                    state.view.sceneViewState.showSkybox = state.state.showSkybox;
                    state.view.sceneViewState.showImageEffects = state.state.showImageEffects;
                    state.view.sceneViewState.showParticleSystems = state.state.showParticleSystems;
                }

                ActiveEditorTracker.sharedTracker.isLocked = false;

                if (SceneManager.GetActiveScene().path.Length <= 0)
                {
                    if (oldSetup != null && oldSetup.Length > 0)
                    {
                        EditorSceneManager.RestoreSceneManagerSetup(oldSetup);
                        oldSetup = null;
                    }
                    else
                    {
                        EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects);
                    }
                }

                Selection.activeObject = oldSelection;

#if (UNITY_2019_1_OR_NEWER)
                SceneView.duringSceneGui -= this.OnSceneGUI;
#else
                SceneView.onSceneGUIDelegate -= this.OnSceneGUI;
#endif

                Repaint();
            }
        }

        public virtual void OnSceneGUI(SceneView sceneView)
        {

            if (!isEditing || sceneView.camera == null)
                return;

            ResizeParticleArrays();

            Event e = Event.current;

            if (e.type == EventType.Repaint)
            {

                // Update camera facing status and world space positions array:
                UpdateParticleVisibility();

                // Generate sorted indices for back-to-front rendering:
                for (int i = 0; i < sortedIndices.Length; i++)
                    sortedIndices[i] = i;
                Array.Sort<int>(sortedIndices, (a, b) => sqrDistanceToCamera[b].CompareTo(sqrDistanceToCamera[a]));

                // render modes OnSceneRepaint:
                for (int i = 0; i < renderModes.Count; ++i)
                {
                    if ((1 << i & renderModeFlags) != 0)
                        renderModes[i].OnSceneRepaint(sceneView);
                }

                // property OnSceneRepaint:
                currentProperty.OnSceneRepaint();

                // Draw particle handles:
                ObiParticleEditorDrawing.DrawParticles(sceneView.camera, blueprint, visible, selectionStatus, sortedIndices);

            }

            if (currentTool != null)
                currentTool.OnSceneGUI(sceneView);

        }

        protected void ResizeParticleArrays()
        {
            if (blueprint.positions != null)
            {
                Array.Resize(ref selectionStatus, blueprint.positions.Length);
                Array.Resize(ref visible, blueprint.positions.Length);
                Array.Resize(ref sqrDistanceToCamera, blueprint.positions.Length);
                Array.Resize(ref sortedIndices, blueprint.positions.Length);
            }

        }

        public bool PropertySelector()
        {
            // get all particle properties:
            string[] propertyNames = new string[properties.Count];
            for (int i = 0; i < properties.Count; ++i)
                propertyNames[i] = properties[i].name;

            // Draw a selection dropdown:
            EditorGUI.BeginChangeCheck();
            int newPropertyIndex = EditorGUILayout.Popup("Property", currentPropertyIndex, propertyNames);
            if (EditorGUI.EndChangeCheck())
            {
                currentPropertyIndex = newPropertyIndex;
                Refresh();
                return true;
            }
            return false;
        }

        public void RenderModeSelector()
        {
            string[] renderModeNames = new string[renderModes.Count];
            for (int i = 0; i < renderModes.Count; ++i)
                renderModeNames[i] = renderModes[i].name;

            // Draw a selection dropdown:
            EditorGUI.BeginChangeCheck();
            int newRenderModeFlags = EditorGUILayout.MaskField("Render mode", renderModeFlags, renderModeNames);
            if (EditorGUI.EndChangeCheck())
            {
                renderModeFlags = newRenderModeFlags;
                Refresh();
            }
        }

        public void Refresh()
        {
            currentProperty.RecalculateMinMax();

            // refresh render modes:
            for (int i = 0; i < renderModes.Count; ++i)
            {
                if ((1 << i & renderModeFlags) != 0)
                    renderModes[i].Refresh();
            }

            SceneView.RepaintAll();
        }

        public virtual void UpdateParticleVisibility()
        {

            for (int i = 0; i < blueprint.positions.Length; i++)
            {
                if (blueprint.IsParticleActive(i))
                {
                    visible[i] = true;

                    if (Camera.current != null)
                    {
                        Vector3 camToParticle = Camera.current.transform.position - blueprint.positions[i];
                        sqrDistanceToCamera[i] = camToParticle.sqrMagnitude;
                    }
                }
            }

        }

        protected void DrawTools()
        {

            GUIContent[] contents = new GUIContent[tools.Count];

            for (int i = 0; i < tools.Count; ++i)
                contents[i] = new GUIContent(tools[i].icon, tools[i].name);

            EditorGUILayout.Space();
            GUILayout.Box(GUIContent.none, ObiEditorUtils.GetSeparatorLineStyle());
            EditorGUILayout.Space();

            EditorGUILayout.BeginVertical(EditorStyles.inspectorDefaultMargins);
            EditorGUI.BeginChangeCheck();
            int newSelectedTool = ObiEditorUtils.DoToolBar(currentToolIndex, contents);
            EditorGUILayout.EndVertical();

            if (EditorGUI.EndChangeCheck())
            {
                if (currentTool != null)
                    currentTool.OnDisable();

                currentToolIndex = newSelectedTool;

                if (currentTool != null)
                    currentTool.OnEnable();

                SceneView.RepaintAll();
            }

            if (currentTool != null)
            {
                EditorGUILayout.BeginVertical(EditorStyles.inspectorDefaultMargins);
                EditorGUILayout.LabelField(currentTool.name, EditorStyles.boldLabel);

                string help = currentTool.GetHelpString();
                if (!help.Equals(string.Empty))
                    EditorGUILayout.LabelField(help, EditorStyles.helpBox);
                EditorGUILayout.EndVertical();

                currentTool.OnInspectorGUI();
            }

        }

        public void SetSelected(int particleIndex, bool selected)
        {
            selectionStatus[particleIndex] = selected;
        }

        public bool IsSelected(int particleIndex)
        {
            return selectionStatus[particleIndex];
        }

        public bool Editable(int particleIndex)
        {
            return currentTool.Editable(particleIndex) && blueprint.IsParticleActive(particleIndex);
        }
    }

}