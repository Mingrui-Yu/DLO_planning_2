using UnityEngine;
using UnityEditor;
using System.Collections;

namespace Obi
{
    public class ObiBlueprintMass : ObiBlueprintFloatProperty
    {

        public ObiBlueprintMass(ObiActorBlueprintEditor editor) : base(editor,0)
        {
            brushModes.Add(new ObiFloatPaintBrushMode(this)); 
            brushModes.Add(new ObiFloatAddBrushMode(this)); 
            brushModes.Add(new ObiFloatSmoothBrushMode(this)); 
        }

        public override string name
        {
            get { return "Mass"; }
        }

        public override float Get(int index)
        {
            return ObiUtils.InvMassToMass(editor.blueprint.invMasses[index]);
        }
        public override void Set(int index, float value)
        {
            editor.blueprint.invMasses[index] = ObiUtils.MassToInvMass(value);
        }
        public override bool Masked(int index)
        {
            return !editor.Editable(index);
        }
    }
}
