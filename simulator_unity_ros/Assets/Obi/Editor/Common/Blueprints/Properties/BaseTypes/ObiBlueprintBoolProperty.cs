using UnityEngine;
using UnityEditor;
using System.Collections;

namespace Obi
{
    public abstract class ObiBlueprintBoolProperty : ObiBlueprintProperty<bool>
    {
        public override bool Equals(int firstIndex, int secondIndex)
        {
            return Get(firstIndex) == Get(secondIndex);
        }

        public override void PropertyField()
        {
            value = EditorGUILayout.Toggle(name, value);
        }

        public override Color ToColor(int index)
        {
            return value ? Color.white : Color.gray;
        }
    }
}
