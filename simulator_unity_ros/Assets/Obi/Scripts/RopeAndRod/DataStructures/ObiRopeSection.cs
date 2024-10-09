using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Obi
{
    [CreateAssetMenu(fileName = "rope section", menuName = "Obi/Rope Section", order = 142)]
	public class ObiRopeSection : ScriptableObject
	{
		[HideInInspector] public List<Vector2> vertices;
		public int snapX = 0;
		public int snapY = 0;

		public int Segments{
			get{return vertices.Count-1;}
		}

		public void OnEnable(){

			if (vertices == null){
				vertices = new List<Vector2>();
				CirclePreset(8);
			}

    	}

		public void CirclePreset(int segments){

			vertices.Clear();

			for (int j = 0; j <= segments; ++j){
				float angle = 2 * Mathf.PI / segments * j;
				vertices.Add(Mathf.Cos(angle)*Vector2.right + Mathf.Sin(angle)*Vector2.up);
			}
		}

		/**
		 * Snaps a float value to the nearest multiple of snapInterval.
		 */	
		public static int SnapTo(float val, int snapInterval, int threshold){
			int intVal = (int) val;	
			if (snapInterval <= 0)
				return intVal;
			int under = Mathf.FloorToInt(val / snapInterval) * snapInterval;
			int over = under + snapInterval;
			if (intVal - under < threshold) return under;
			if (over - intVal < threshold) return over;
			return intVal;
		}

	}
}

