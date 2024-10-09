using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace Obi{


	[System.AttributeUsage(System.AttributeTargets.Field)]
	public class Indent : MultiPropertyAttribute
	{
		#if UNITY_EDITOR
		internal override void OnPreGUI(Rect position, SerializedProperty property)
		{
			EditorGUI.indentLevel++;
		}
		internal override void OnPostGUI(Rect position, SerializedProperty property)
		{
			EditorGUI.indentLevel--;
		}
		#endif
	}

}

