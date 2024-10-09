using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace Obi{


	[System.AttributeUsage(System.AttributeTargets.Field)]
	public class ChildrenOnly : MultiPropertyAttribute
	{
		#if UNITY_EDITOR
		public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
	    {

			float height = 0;
			SerializedProperty it = property;
			int depth = it.depth;
			it.NextVisible(true);
			do 
			{
				EditorGUI.PropertyField(new Rect(position.x,position.y+height,position.width,EditorGUIUtility.singleLineHeight),it,true);
				height += EditorGUIUtility.singleLineHeight + EditorGUIUtility.standardVerticalSpacing;
			}while (it.NextVisible(false) && it.depth != depth);

	    }
	 
	    internal override float? GetPropertyHeight(SerializedProperty property, GUIContent label)
	    {

			float height = -EditorGUIUtility.standardVerticalSpacing;
			SerializedProperty it = property;
			int depth = it.depth;
			it.NextVisible(true);
			do 
			{
			    height += EditorGUI.GetPropertyHeight(it, label) + EditorGUIUtility.standardVerticalSpacing;
			}while (it.NextVisible(false) && it.depth != depth);
			return height;

	    }
		#endif
	}

}
