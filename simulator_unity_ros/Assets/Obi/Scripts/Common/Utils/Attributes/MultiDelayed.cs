using System;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace Obi{

	[System.AttributeUsage(System.AttributeTargets.Field)]
	public class MultiDelayed : MultiPropertyAttribute
	{
		#if UNITY_EDITOR
	    public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
	    {
	        // Now draw the property as a Slider or an IntSlider based on whether it's a float or integer.
	        if (property.propertyType == SerializedPropertyType.Float)
	            EditorGUI.DelayedFloatField(position, property, label);
	        else if (property.propertyType == SerializedPropertyType.Integer)
	            EditorGUI.DelayedIntField(position, property, label);
			else if (property.propertyType == SerializedPropertyType.String)
	            EditorGUI.DelayedTextField(position, property, label);
	        else
	            EditorGUI.LabelField(position, label.text, "Use MultiRange with float or int.");
	    }
		#endif
	}

}

