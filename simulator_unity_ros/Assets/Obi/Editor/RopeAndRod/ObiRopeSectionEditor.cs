using UnityEditor;
using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

namespace Obi{
	
	/**
	 * Custom inspector for ObiParticleRenderer component. 
	 */

	[CustomEditor(typeof(ObiRopeSection))] 
	public class ObiRopeSectionEditor : Editor
	{
	
		ObiRopeSection section;
		bool[] selected = new bool[0];

		Color previewBck = new Color(0.2f,0.2f,0.2f,1);
		Color previewLines = new Color(0.15f,0.15f,0.15f,1);

		public void OnEnable(){
			section = (ObiRopeSection)target;
		}
		
		public override bool HasPreviewGUI(){
			return true;
		}

		private void ResetSelection(){
			selected = new bool[section.Segments];
		}

		public override void OnInspectorGUI() {
			
			serializedObject.UpdateIfRequiredOrScript();
			
			Editor.DrawPropertiesExcluding(serializedObject,"m_Script");

			GUI.enabled = !EditorApplication.isPlaying;
			GUILayout.Label("Presets");

			GUILayout.BeginHorizontal();
			if (GUILayout.Button("4-segment circle")){
				Undo.RecordObject(section, "Set rope section preset");
				section.CirclePreset(4);
				ResetSelection();
			}

			if (GUILayout.Button("8-segment circle")){
				Undo.RecordObject(section, "Set rope section preset");
				section.CirclePreset(8);
				ResetSelection();
			}
			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal();
			if (GUILayout.Button("12-segment circle")){
				Undo.RecordObject(section, "Set rope section preset");
				section.CirclePreset(12);
				ResetSelection();
			}

			if (GUILayout.Button("16-segment circle")){
				Undo.RecordObject(section, "Set rope section preset");
				section.CirclePreset(16);
				ResetSelection();				
			}
			GUILayout.EndHorizontal();

			GUILayout.Label("Tools");
			if (GUILayout.Button("Add vertex")){
				Undo.RecordObject(section, "Add rope vertex");
				section.vertices.Add(Vector2.zero);
			}

			if (GUILayout.Button("Remove selected vertices")){
				Undo.RecordObject(section, "Remove rope vertices");
				for (int i = selected.Length-1; i > 0; --i){
					if (selected[i] && section.vertices.Count > 3)
						section.vertices.RemoveAt(i);
				}
				// special cases: first vertex:
				if (selected[0] && section.vertices.Count > 3){
					section.vertices.RemoveAt(0);
					section.vertices[section.vertices.Count-1] = section.vertices[0];
				}

				ResetSelection();
			}
			GUI.enabled = true;
						
			// Apply changes to the serializedProperty
			if (GUI.changed){
				serializedObject.ApplyModifiedProperties();
                EditorUtility.SetDirty(target);
			}
			
		}

		private void DrawSectionOutline(Rect region, Color color){
			// Draw segment lines:
			Handles.BeginGUI( );
			Color oldColor = Handles.color;
			Handles.color = color;
			Vector3[] points = new Vector3[section.vertices.Count];
			for (int i = 0; i < section.vertices.Count; i++){
				points[i] = new Vector3(region.center.x + section.vertices[i].x * region.width * 0.5f,
										region.center.y + section.vertices[i].y * region.height * 0.5f,0);		
			}
			Handles.DrawAAPolyLine(points);
			Handles.EndGUI();
			Handles.color = oldColor;
		}

		private void DrawDrawingArea(Rect region){
			// Draw drawing area grid:
			Handles.BeginGUI();
			Handles.DrawSolidRectangleWithOutline(region,previewBck,previewLines);

			Color oldColor = Handles.color;
			Handles.color = previewLines;

			if (section.snapX > 5){
				float x = region.center.x;
				while (x < region.xMax){
					Handles.DrawLine(new Vector3(x,region.yMin,0),new Vector3(x,region.yMax,0));
					x += section.snapX;
				}
				x = region.center.x - section.snapX;
				while (x > region.xMin){
					Handles.DrawLine(new Vector3(x,region.yMin,0),new Vector3(x,region.yMax,0));
					x -= section.snapX;
				}
			}

			if (section.snapY > 5){
				float y = region.center.y;
				while (y < region.yMax){
					Handles.DrawLine(new Vector3(region.xMin,y,0),new Vector3(region.xMax,y,0));
					y += section.snapY;
				}
				y = region.center.y - section.snapY;
				while (y > region.yMin){
					Handles.DrawLine(new Vector3(region.xMin,y,0),new Vector3(region.xMax,y,0));
					y -= section.snapY;
				}
			}

			Handles.color = oldColor;
			Handles.EndGUI();
		}

		public override void OnPreviewGUI(Rect region, GUIStyle background)
		{
			DrawSectionOutline(region, Color.red);
		}

		public override void OnInteractivePreviewGUI(Rect region, GUIStyle background)
		{
			Array.Resize(ref selected,section.Segments);
		
			// Calculate drawing area rect:
			Vector2 oldCenter = region.center;
			if (region.width > region.height)
				region.width = region.height;
			if (region.height > region.width)
				region.height = region.width;

			region.width -= 10;
			region.height -= 15;
			
			region.center = oldCenter;
		
			// Draw background and lines:
			DrawDrawingArea(region);

			// Draw the section outline:
			DrawSectionOutline(region, Color.white);

			// Draw all draggable vertices:
			for (int i = 0; i < section.Segments; i++){

				float x = region.center.x + section.vertices[i].x * region.width * 0.5f;
				float y = region.center.y + section.vertices[i].y * region.height * 0.5f;
				Vector2 pos = new Vector2(x,y);
				
				bool oldSelection = selected[i];
				Vector2 olsPos = pos;
				selected[i] = ObiDraggableIcon.Draw(selected[i],i,ref pos,Color.red);

				if (selected[i] != oldSelection)
					this.Repaint();

				if (pos != olsPos){

					pos.x = Mathf.Clamp(ObiRopeSection.SnapTo(pos.x - region.center.x,section.snapX,5) / (region.width * 0.5f),-1,1);
					pos.y = Mathf.Clamp(ObiRopeSection.SnapTo(pos.y - region.center.y,section.snapY,5) / (region.height * 0.5f),-1,1);
					section.vertices[i] = pos;
					if (i == 0)
						section.vertices[section.Segments] = pos;
					
                    EditorUtility.SetDirty(target);
				}
			}
			
		}
		
	}

}

