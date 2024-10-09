using UnityEngine;
using UnityEditor;
using System.Collections;

namespace Obi
{
    public class ObiBlueprintColor : ObiBlueprintColorProperty
    {
        public ObiBlueprintColor(ObiActorBlueprintEditor editor) : base(editor)
        {
            brushModes.Add(new ObiColorPaintBrushMode(this)); 
            brushModes.Add(new ObiColorSmoothBrushMode(this)); 
        }

        public override string name
        {
            get { return "Color"; }
        }

        public override Color Get(int index)
        {
            return editor.blueprint.colors[index];
        }
        public override void Set(int index, Color value)
        {
            editor.blueprint.colors[index] = value;
        }
        public override bool Masked(int index)
        {
            return !editor.Editable(index);
        }

    }
}
