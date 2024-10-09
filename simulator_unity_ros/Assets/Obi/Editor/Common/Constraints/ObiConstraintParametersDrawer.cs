using UnityEngine;
using UnityEditor;
using System;

namespace Obi
{

	[CustomPropertyDrawer(typeof(Oni.ConstraintParameters))]
	public class ObiConstraintParametersDrawer : PropertyDrawer
	{		
		public static float padding = 4;

		public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
		{
			float propHeight = EditorGUIUtility.singleLineHeight + EditorGUIUtility.standardVerticalSpacing;

			// Remove the "Constraint Parameters" trailing in the label:
			string shortenedLabel = label.text.Remove(label.text.Length-22);

			EditorGUI.BeginProperty(position, label, property);

			SerializedProperty enabled = property.FindPropertyRelative("enabled");
			Rect contRect = new Rect(position.x+padding, position.y+padding, position.width-padding*2, propHeight);
	
			// Draw a box around the parameters:
			GUI.enabled = enabled.boolValue;
            GUI.Box(position,"",ObiEditorUtils.GetToggleablePropertyGroupStyle());
			GUI.enabled = true;

			// Draw main constraint toggle:
            enabled.boolValue = EditorGUI.ToggleLeft(contRect, shortenedLabel, enabled.boolValue, EditorStyles.boldLabel);

			if (enabled.boolValue){
	
				Rect evalRect = new Rect(position.x+padding, position.y+propHeight+padding, position.width-padding*2, propHeight);
	       		Rect iterRect = new Rect(position.x+padding, position.y+propHeight*2+padding, position.width-padding*2, propHeight);
	        	Rect sorRect =  new Rect(position.x+padding, position.y+propHeight*3+padding, position.width-padding*2, EditorGUIUtility.singleLineHeight);
	
				EditorGUI.indentLevel++;
					Rect evalCtrl = EditorGUI.PrefixLabel(evalRect,new GUIContent("Evaluation"));
					EditorGUI.PropertyField(evalCtrl, property.FindPropertyRelative("evaluationOrder"),GUIContent.none);
	
					Rect iterCtrl = EditorGUI.PrefixLabel(iterRect,new GUIContent("Iterations"));
					EditorGUI.PropertyField(iterCtrl, property.FindPropertyRelative("iterations"),GUIContent.none);

					Rect sorCtrl = EditorGUI.PrefixLabel(sorRect,new GUIContent("Relaxation"));
					EditorGUI.PropertyField(sorCtrl, property.FindPropertyRelative("SORFactor"),GUIContent.none);
				EditorGUI.indentLevel--;
			
			}

			EditorGUI.EndProperty();
		}

 		public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
		{
			SerializedProperty enabled = property.FindPropertyRelative("enabled");
			if (enabled.boolValue)
				return EditorGUIUtility.singleLineHeight*4 + EditorGUIUtility.standardVerticalSpacing*3 + padding*2;
			else
				return EditorGUIUtility.singleLineHeight + padding*2;
		}
	}

}

