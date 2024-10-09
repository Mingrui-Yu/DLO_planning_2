
using System;
using UnityEditor;
using UnityEngine;

namespace Obi
{
    public class ObiAboutWindow : EditorWindow
    {

        [MenuItem ("Window/Obi/About")]
        public static void Init()
        {
            ObiAboutWindow window = (ObiAboutWindow)EditorWindow.GetWindow(typeof(ObiAboutWindow),true,"Welcome to Obi!");
            window.position = new Rect(Screen.width / 2, Screen.height / 2, 380, 300);
            window.maxSize = window.minSize = new Vector2(380,300);
            window.ShowPopup();
        }
    
        void OnGUI()
        {
            // Draw logo and copyright notice:
            EditorGUILayout.BeginHorizontal();

				GUILayout.Label(Resources.Load<Texture2D>("obi_editor_logo"));

                EditorGUILayout.BeginVertical(GUILayout.MaxHeight(119.0f/EditorGUIUtility.pixelsPerPoint));  
   
                    GUILayout.FlexibleSpace();
    
                    Color oldColor = GUI.contentColor;
                    GUI.contentColor = Color.black;
                    GUILayout.Label("Obi - Unified particle physics",EditorStyles.centeredGreyMiniLabel);
                    GUI.contentColor = oldColor;
    
                    GUILayout.Label("© Copyright Virtual Method, 2015-2016.\nAll rights reserved.",EditorStyles.centeredGreyMiniLabel);

                    GUILayout.FlexibleSpace();

                EditorGUILayout.EndVertical();

            EditorGUILayout.EndHorizontal();

            DrawAboutGUI();
            
        }

        void DrawAboutGUI(){
            
            GUILayout.FlexibleSpace();

            EditorGUILayout.LabelField("Programming:",EditorStyles.centeredGreyMiniLabel);
            EditorGUILayout.LabelField("José María Méndez González",EditorStyles.centeredGreyMiniLabel);

            EditorGUILayout.LabelField("Additional resources:",EditorStyles.centeredGreyMiniLabel);
            EditorGUILayout.LabelField("Lidia Martínez Prado",EditorStyles.centeredGreyMiniLabel);

            GUILayout.FlexibleSpace();
        
            if (GUILayout.Button("Manual",EditorStyles.toolbarButton))
                Application.OpenURL("http://obi.virtualmethodstudio.com/tutorials/");
            if (GUILayout.Button("API docs",EditorStyles.toolbarButton))
                Application.OpenURL("http://obi.virtualmethodstudio.com/docs/");
            if (GUILayout.Button("visit www.virtualmethodstudio.com",EditorStyles.toolbarButton))
                Application.OpenURL("http://www.virtualmethodstudio.com");
            if (GUILayout.Button("Create preferences file", EditorStyles.toolbarButton))
                ObiEditorSettings.GetOrCreateSettings();

        }
    }
}


