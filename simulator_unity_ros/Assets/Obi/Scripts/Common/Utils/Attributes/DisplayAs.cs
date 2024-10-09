using System;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace Obi{

	#if UNITY_EDITOR
	[System.AttributeUsage(System.AttributeTargets.Field)]
	public class DisplayAs : MultiPropertyAttribute
	{
	    string name;
	    public DisplayAs(string name)
	    {
	        this.name = name;
	    }

	    public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
	    {
			EditorGUI.PropertyField(position,property,new GUIContent(name),true);
	    }
	}
	#endif
}

