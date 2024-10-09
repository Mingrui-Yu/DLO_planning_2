using UnityEngine;
using UnityEditor;
using System.Collections;

namespace Obi
{
    public class ObiBlueprintLayer : ObiBlueprintIntProperty
    {
        public ObiActorBlueprintEditor editor;

        public ObiBlueprintLayer(ObiActorBlueprintEditor editor) : base(0,(1 << 24) - 1) 
        {
            this.editor = editor;
            brushModes.Add(new ObiIntPaintBrushMode(this));
        }

        public override string name
        {
            get { return "Phase"; }
        }

        public override int Get(int index)
        {
            return ObiUtils.GetGroupFromPhase(editor.blueprint.phases[index]);
        }
        public override void Set(int index, int value)
        {
            editor.blueprint.phases[index] = ObiUtils.MakePhase((int)value, 0);
        }
        public override bool Masked(int index)
        {
            return !editor.Editable(index);
        }
    }
}
