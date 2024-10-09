using UnityEngine;
using UnityEditor;
using System.Collections;

namespace Obi
{
    public abstract class ObiBlueprintFloatProperty : ObiBlueprintProperty<float>
    {
        public float minVisualizationValue = 0;
        public float maxVisualizationValue = 10;
        protected float minUserVisualizationValue = 0;
        protected float maxUserVisualizationValue = 10;

        protected float? minValue = null;
        protected float? maxValue = null;

        public bool autoRange = true;
        public ObiActorBlueprintEditor editor;

        public ObiBlueprintFloatProperty(ObiActorBlueprintEditor editor, float? minValue = null, float? maxValue = null)
        {
            this.editor = editor;
            this.minValue = minValue;
            this.maxValue = maxValue;
        }

        public override bool Equals(int firstIndex, int secondIndex)
        {
            float v1 = Get(firstIndex);
            float v2 = Get(secondIndex);
            if (v1 == v2) return true; 
            return Mathf.Approximately(v1,v2);
        }

        public override void PropertyField()
        {
            EditorGUI.BeginChangeCheck();
            value = EditorGUILayout.FloatField(name, value);
            if (EditorGUI.EndChangeCheck())
            {
                if (minValue.HasValue)
                    value = Mathf.Max(minValue.Value, value);
                if (maxValue.HasValue)
                    value = Mathf.Min(maxValue.Value, value);
            }
        }

		public override void RecalculateMinMax()
		{
            if (autoRange)
            {
                maxVisualizationValue = float.MinValue;
                minVisualizationValue = float.MaxValue;

                for (int i = 0; i < editor.blueprint.activeParticleCount; i++)
                {
                    float v = Get(i);
                    maxVisualizationValue = Mathf.Max(maxVisualizationValue, v);
                    minVisualizationValue = Mathf.Min(minVisualizationValue, v);
                }
            }
            else
            {
                maxVisualizationValue = maxUserVisualizationValue;
                minVisualizationValue = minUserVisualizationValue;
            }
		}

        public override void VisualizationOptions()
        {
            EditorGUI.BeginChangeCheck();
            autoRange = EditorGUILayout.Toggle("Automatic range", autoRange);
            GUI.enabled = !autoRange;
            EditorGUI.indentLevel++;
            minUserVisualizationValue = EditorGUILayout.FloatField("Min", minUserVisualizationValue);
            maxUserVisualizationValue = EditorGUILayout.FloatField("Max", maxUserVisualizationValue);
            EditorGUI.indentLevel--;
            GUI.enabled = true;

            if (EditorGUI.EndChangeCheck())
            {
                RecalculateMinMax();
                editor.Refresh();
            }
        }

		public override Color ToColor(int index)
        {
            Gradient gradient = ObiEditorSettings.GetOrCreateSettings().propertyGradient;
            if (!Mathf.Approximately(minVisualizationValue, maxVisualizationValue))
                return gradient.Evaluate(Mathf.InverseLerp(minVisualizationValue, maxVisualizationValue, Get(index)));
            else return gradient.Evaluate(0);
        }
    }
}
