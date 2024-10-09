using UnityEngine;
using UnityEditor;
using UnityEditorInternal;
using System.Collections;
using System;

namespace Obi
{
    public class ObiPropertyTextureEditorTool : ObiBlueprintEditorTool
    {
        public enum TextureChannel
        {
            Red = 0,
            Green = 1,
            Blue = 2,
            Alpha = 3,
        }

        protected bool selectionMask = false;
        protected bool import = true;
        protected bool export = true;

        protected float minPropertyValue = 0;
        protected float maxPropertyValue = 10;

        protected int exportWidth = 512;
        protected int exportHeight = 512;
        protected int padding = 64;

        protected Texture2D propertyTexture;
        protected TextureChannel textureChannel;

        protected ObiBlueprintFloatProperty floatProperty;
        protected ObiBlueprintColorProperty colorProperty;
        protected Action<int, Color> textureReadCallback;

        public ObiMeshBasedActorBlueprintEditor meshBasedEditor
        {
            get { return editor as ObiMeshBasedActorBlueprintEditor; }
        }

        public ObiPropertyTextureEditorTool(ObiMeshBasedActorBlueprintEditor editor) : base(editor)
        {
            m_Icon = Resources.Load<Texture2D>("TextureIcon");
            m_Name = "Texture import/export";
        }

        public override string GetHelpString()
        {
            return "Import/export particle properties to textures. Assumes that your mesh has non-overlapping UVs.";
        }

        private void FloatFromTexture(int i, Color color)
        {
            if (!selectionMask || editor.selectionStatus[i])
            {
                float value = minPropertyValue + color[(int)textureChannel] * (maxPropertyValue - minPropertyValue);
                floatProperty.Set(i, value);
            }
        }

        private void ColorFromTexture(int i, Color color)
        {
            if (!selectionMask || editor.selectionStatus[i])
                colorProperty.Set(i, color);
        }

        public override void OnInspectorGUI()
        {
            EditorGUILayout.BeginVertical(EditorStyles.inspectorDefaultMargins);
            EditorGUILayout.Space();
            editor.RenderModeSelector();
            editor.PropertySelector();
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space();
            GUILayout.Box(GUIContent.none, ObiEditorUtils.GetSeparatorLineStyle());

            EditorGUILayout.BeginVertical(EditorStyles.inspectorDefaultMargins);
            import = EditorGUILayout.Foldout(import, "Import texture");

            if (import)
            {
                propertyTexture = (Texture2D)EditorGUILayout.ObjectField("Source", propertyTexture, typeof(Texture2D), false);

                floatProperty = editor.currentProperty as ObiBlueprintFloatProperty;
                colorProperty = editor.currentProperty as ObiBlueprintColorProperty;

                if (floatProperty != null)
                {
                    textureReadCallback = FloatFromTexture;
                    textureChannel = (TextureChannel)EditorGUILayout.EnumPopup("Source channel", textureChannel);
                    minPropertyValue = EditorGUILayout.FloatField("Min value", minPropertyValue);
                    maxPropertyValue = EditorGUILayout.FloatField("Max value", maxPropertyValue);
                }
                else if (colorProperty != null)
                {
                    textureReadCallback = ColorFromTexture;
                }

                if (GUILayout.Button("Import"))
                {
                    Undo.RecordObject(editor.blueprint, "Import particle property");
                    if (!meshBasedEditor.ReadParticlePropertyFromTexture(propertyTexture, textureReadCallback))
                    {
                        EditorUtility.DisplayDialog("Invalid texture", "The texture is either null or not readable.", "Ok");
                    }

                    // force automatic range calculation for floating point properties.
                    if (floatProperty != null)
                        floatProperty.autoRange = true;
                    editor.Refresh();
                }
            }
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space();
            GUILayout.Box(GUIContent.none, ObiEditorUtils.GetSeparatorLineStyle());

            EditorGUILayout.BeginVertical(EditorStyles.inspectorDefaultMargins);
            export = EditorGUILayout.Foldout(export, "Export texture");

            if (export)
            {
                exportWidth = EditorGUILayout.IntField("Texture width", exportWidth);
                exportHeight = EditorGUILayout.IntField("Texture height", exportHeight);
                padding = EditorGUILayout.IntField("Padding", padding);
                if (GUILayout.Button("Export"))
                {
                    var path = EditorUtility.SaveFilePanel("Save texture as PNG",
                                                            "",
                                                            "property.png",
                                                            "png");
                    if (path.Length > 0)
                    {
                        // force automatic range calculation for floating point properties.
                        if (floatProperty != null)
                            floatProperty.autoRange = true;
                        editor.Refresh();

                        if (!meshBasedEditor.WriteParticlePropertyToTexture(path, exportWidth, exportHeight, padding))
                        {
                            EditorUtility.DisplayDialog("Invalid path", "Could not write a texture to that location.", "Ok");
                        }
                    }
                }
            }

            EditorGUILayout.EndVertical();

        }

    }
}