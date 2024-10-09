using System;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace Obi{

	[System.AttributeUsage(System.AttributeTargets.Field)]
	public class MinMaxAttribute : MultiPropertyAttribute
	{
	    float min;
	    float max;
	    public MinMaxAttribute(float min, float max)
	    {
	        this.min = min;
	        this.max = max;
	    }
		#if UNITY_EDITOR
	    public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
	    {
	        if (property.propertyType == SerializedPropertyType.Vector2){
				float minValue = property.vector2Value.x; 
            	float maxValue = property.vector2Value.y;

	            EditorGUI.MinMaxSlider(position, label, ref minValue, ref maxValue, min, max);

				var vec = Vector2.zero;
	            vec.x = minValue;
	            vec.y = maxValue;
	
	            property.vector2Value = vec;
			}else{
            	EditorGUI.LabelField(position, label.text, "Use MinMaxAttribute with Vector2.");
			}
	    }
		#endif
	}

}

