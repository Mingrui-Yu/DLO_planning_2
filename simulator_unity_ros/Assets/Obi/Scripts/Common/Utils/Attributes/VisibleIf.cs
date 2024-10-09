using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif
using System.Reflection;
 
namespace Obi{

	[System.AttributeUsage(System.AttributeTargets.Field)]
	public class VisibleIf : MultiPropertyAttribute
	{
	    public string MethodName { get; private set; }
		public bool Negate {get; private set;}

		private MethodInfo eventMethodInfo = null;
		private FieldInfo fieldInfo = null;

		public VisibleIf(string methodName, bool negate = false)
	    {
	        this.MethodName = methodName;
			this.Negate = negate;
	    } 

		#if UNITY_EDITOR
	    internal override bool IsVisible(SerializedProperty property)
	    {
	        return Visibility(property) == !Negate;
	    }

		private bool Visibility(SerializedProperty property)
	    {
			System.Type eventOwnerType = property.serializedObject.targetObject.GetType();
			string eventName = MethodName;
			
			// Try finding a method with the name provided:
			if (eventMethodInfo == null)
				eventMethodInfo = eventOwnerType.GetMethod(eventName, BindingFlags.Instance | BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);

			// If we could not find a method with that name, look for a field:
			if (eventMethodInfo == null && fieldInfo == null)
				fieldInfo = eventOwnerType.GetField(eventName, BindingFlags.Instance | BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);
			
			if (eventMethodInfo != null)
				return (bool)eventMethodInfo.Invoke(property.serializedObject.targetObject, null); 
			else if (fieldInfo != null)
				return (bool)fieldInfo.GetValue(property.serializedObject.targetObject);
			else
				Debug.LogWarning(string.Format("VisibleIf: Unable to find method or field {0} in {1}", eventName, eventOwnerType));

			return true;
	    }
		#endif
	}
	

}