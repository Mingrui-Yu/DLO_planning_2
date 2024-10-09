using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;

namespace Obi
{
    class ObiSettingsProvider : SettingsProvider
    {
        private SerializedObject m_ObiSettings;

        class Styles
        {
            public static GUIContent particleBrush = new GUIContent("Brush");
            public static GUIContent brushWireframe = new GUIContent("Brush wireframe");
            public static GUIContent particle = new GUIContent("Particle");
            public static GUIContent selectedParticle = new GUIContent("Selected particle");
            public static GUIContent propertyGradient = new GUIContent("Property gradient");
        }

        const string m_ObiEditorSettingsPath = "Assets/ObiEditorSettings.asset";
        public ObiSettingsProvider(string path, SettingsScope scope = SettingsScope.User)
            : base(path, scope) { }

        public static bool IsSettingsAvailable()
        {
            return File.Exists(m_ObiEditorSettingsPath);
        }

#if UNITY_2019_1_OR_NEWER
        public override void OnActivate(string searchContext, UnityEngine.UIElements.VisualElement rootElement)
#else
        public override void OnActivate(string searchContext, UnityEngine.Experimental.UIElements.VisualElement rootElement)
#endif
        {
            // This function is called when the user clicks on the MyCustom element in the Settings window.
            m_ObiSettings = ObiEditorSettings.GetSerializedSettings();
        }

        public override void OnDeactivate()
        {
            base.OnDeactivate();

            if (m_ObiSettings != null)
                m_ObiSettings.ApplyModifiedProperties();
        }

        public override void OnGUI(string searchContext)
        {
            EditorGUILayout.LabelField("Colors", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(m_ObiSettings.FindProperty("m_ParticleBrush"), Styles.particleBrush);
            EditorGUILayout.PropertyField(m_ObiSettings.FindProperty("m_BrushWireframe"),Styles.brushWireframe);
            EditorGUILayout.PropertyField(m_ObiSettings.FindProperty("m_Particle"),Styles.particle);
            EditorGUILayout.PropertyField(m_ObiSettings.FindProperty("m_SelectedParticle"),Styles.selectedParticle);
            EditorGUILayout.PropertyField(m_ObiSettings.FindProperty("m_PropertyGradient"), Styles.propertyGradient);
        }

        // Register the SettingsProvider
        [SettingsProvider]
        public static SettingsProvider CreateMyCustomSettingsProvider()
        {
            if (IsSettingsAvailable())
            {
                var provider = new ObiSettingsProvider("Preferences/Obi", SettingsScope.User);

                // Automatically extract all keywords from the Styles.
                provider.keywords = GetSearchKeywordsFromGUIContentProperties<Styles>();
                return provider;
            }

            // Settings Asset doesn't exist yet; no need to display anything in the Settings window.
            return null;
        }
    }
}