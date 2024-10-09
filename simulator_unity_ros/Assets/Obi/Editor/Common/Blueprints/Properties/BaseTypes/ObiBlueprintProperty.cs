using UnityEngine;
using UnityEditor;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    public abstract class ObiBlueprintPropertyBase
    {
        protected List<ObiBrushMode> brushModes = new List<ObiBrushMode>();
        private int selectedBrushMode;

        public abstract string name
        {
            get;
        }

        public abstract void PropertyField();
        public virtual void VisualizationOptions(){}
        public virtual void OnSceneRepaint(){}

        public abstract bool Equals(int firstIndex, int secondIndex);

        public abstract void GetDefaultFromIndex(int index);
        public abstract void SetDefaultToIndex(int index);
        public virtual bool Masked(int index)
        {
            return false;
        }

        public virtual void RecalculateMinMax() { }
        public virtual Color ToColor(int index) { return Color.white; }

        public void OnSelect(ObiBrushBase paintBrush) 
        {
            // Initialize the brush:
            if (brushModes.Count > 0)
                paintBrush.brushMode = brushModes[0];
        }

        public void BrushModes(ObiBrushBase paintBrush)
        {
            // Initialize the brush if there's no mode set:
            if (paintBrush.brushMode == null && brushModes.Count > 0)
                paintBrush.brushMode = brushModes[0];

            GUIContent[] contents = new GUIContent[brushModes.Count];

            for (int i = 0; i < brushModes.Count; ++i)
                contents[i] = new GUIContent(brushModes[i].name);

            EditorGUI.BeginChangeCheck();
            selectedBrushMode = ObiEditorUtils.DoToolBar(selectedBrushMode, contents);
            if (EditorGUI.EndChangeCheck())
            {
                paintBrush.brushMode = brushModes[selectedBrushMode];
            }
        }
    }

    public abstract class ObiBlueprintProperty<T> : ObiBlueprintPropertyBase
    {
        protected T value;

        public T GetDefault() { return value; }
        public override void GetDefaultFromIndex(int index) { value = Get(index); }
        public override void SetDefaultToIndex(int index) { Set(index, value); }

        public abstract T Get(int index);
        public abstract void Set(int index, T value);
    }
}
