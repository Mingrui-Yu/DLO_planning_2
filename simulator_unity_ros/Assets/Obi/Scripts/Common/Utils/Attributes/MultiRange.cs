using System;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace Obi{

	[System.AttributeUsage(System.AttributeTargets.Field)]
	public class MultiRange : MultiPropertyAttribute
	{
	    float min;
	    float max;
	    public MultiRange(float min, float max)
	    {
	        this.min = min;
	        this.max = max;
	    }
		#if UNITY_EDITOR
	    public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
	    {
	        // Now draw the property as a Slider or an IntSlider based on whether it's a float or integer.
	        if (property.propertyType == SerializedPropertyType.Float)
	            EditorGUI.Slider(position, property, min, max, label);
	        else if (property.propertyType == SerializedPropertyType.Integer)
	            EditorGUI.IntSlider(position, property, (int)min, (int)max, label);
	        else
	            EditorGUI.LabelField(position, label.text, "Use MultiRange with float or int.");
	    }
		#endif
	}

}

