using UnityEngine;
using UnityEditor;
using System.Collections;

namespace Obi
{
    public abstract class ObiBlueprintColorProperty : ObiBlueprintProperty<Color>
    {
        public ObiActorBlueprintEditor editor;

        public ObiBlueprintColorProperty(ObiActorBlueprintEditor editor)
        {
            this.editor = editor;
        }

        public override bool Equals(int firstIndex, int secondIndex)
        {
            return Get(firstIndex) == Get(secondIndex);
        }

        public override void PropertyField()
        {
            value = EditorGUILayout.ColorField(name, value);
        }

        public override Color ToColor(int index)
        {
            return editor.blueprint.colors[index];
        }
    }
}
