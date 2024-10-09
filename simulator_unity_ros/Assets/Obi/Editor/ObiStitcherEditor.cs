using UnityEditor;
using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace Obi{
	
	/**
	 * Custom inspector for ObiStitcher component. 
	 */
	
	[CustomEditor(typeof(ObiStitcher))] 
	public class ObiStitcherEditor : Editor
	{
		
		ObiStitcher stitcher;
		static public bool editing = false;

		static public Vector3 sewingToolHandle1 = Vector3.zero;
		static public Vector3 sewingToolHandle2 = Vector3.one;

		static public bool[] selectionStatus = new bool[0];
		
		public void OnEnable(){
			stitcher = (ObiStitcher)target;

			// initialize sewing tool to sensible values:
			if (stitcher.Actor1 != null && stitcher.Actor2 != null){
				sewingToolHandle1 = stitcher.Actor1.transform.position;
				sewingToolHandle2 = stitcher.Actor2.transform.position;
			}
		}
		
		public override void OnInspectorGUI() {
			
			serializedObject.UpdateIfRequiredOrScript();

			EditorGUI.BeginChangeCheck();
			ObiActor actor1 = EditorGUILayout.ObjectField("First actor",stitcher.Actor1, typeof(ObiActor),true) as ObiActor;
			if (EditorGUI.EndChangeCheck()){
				Undo.RecordObject(stitcher, "Set first actor");
				stitcher.Actor1 = actor1;
			}

			EditorGUI.BeginChangeCheck();
			ObiActor actor2 = EditorGUILayout.ObjectField("Second actor",stitcher.Actor2, typeof(ObiActor),true) as ObiActor;
			if (EditorGUI.EndChangeCheck()){
				Undo.RecordObject(stitcher, "Set second actor");
				stitcher.Actor2 = actor2;
			}

			if (stitcher.Actor1 != null && stitcher.Actor2 != null && stitcher.Actor1.solver != stitcher.Actor2.solver){
				EditorGUILayout.HelpBox("Both actors must be managed by the same solver.",MessageType.Error);
			}

			EditorGUILayout.HelpBox("Stitch count: " + stitcher.StitchCount,MessageType.None);

			// edit mode:
			GUI.enabled = stitcher.Actor1 != null && stitcher.Actor2 != null;
			editing = GUILayout.Toggle(editing,"Edit","LargeButton");

			if (editing){

				EditorGUILayout.HelpBox("Remember that when working with the sewing tool, you can use Unity's snap to vertex feature by pressing 'V' in your keyboard.",MessageType.Info);

				// Clear all stitches
				if (GUILayout.Button("Clear all stitches")){
					if (EditorUtility.DisplayDialog("Clearing stitches","Are you sure you want to remove all stitches?","Ok","Cancel")){
						Undo.RecordObject(stitcher, "Clear all stitches");
						stitcher.Clear();
					}
				}
	
				// Remove selected stitches
				if (GUILayout.Button("Remove selected stitches")){

					List<int> removedStitches = new List<int>();

					for(int i = 0; i < selectionStatus.Length; ++i){
						if (selectionStatus[i]){
							removedStitches.Add(i);
							selectionStatus[i] = false;
						}
					}

					if (removedStitches.Count > 0){
	
						Undo.RecordObject(stitcher, "Remove stitches");

						// Remove from last to first, to avoid throwing off subsequent indices:
						foreach(int i in removedStitches.OrderByDescending(i => i)){
							stitcher.RemoveStitch(i);
						}
					}
				}

				// Add stitch:
				if (GUILayout.Button("Add Stitch")){
					Undo.RecordObject(stitcher, "Add stitch");

					int particle1 = 0;
					int particle2 = 0;
					UseSewingTool(ref particle1, ref particle2);

					stitcher.AddStitch(particle1,particle2);
				}
			}
			GUI.enabled = true;

			// Apply changes to the serializedProperty
			if (GUI.changed){
				
				serializedObject.ApplyModifiedProperties();
				
				//stitcher.PushDataToSolver(ParticleData.NONE);
				
			}
			
		}

		public void UseSewingTool(ref int particle1, ref int particle2){

			float minDistance = float.MaxValue;

			if (stitcher.Actor1 == stitcher.Actor2){
				float minDistance2 = float.MaxValue;
                for (int i = 0; i < stitcher.Actor1.particleCount;++i){
					Vector3 pos = stitcher.Actor1.GetParticlePosition(stitcher.Actor1.solverIndices[i]);
					float distance1 = (pos - sewingToolHandle1).sqrMagnitude;
					float distance2 = (pos - sewingToolHandle2).sqrMagnitude;
					if (distance1 < minDistance){
						minDistance = distance1;
						particle1 = i;
					}
					if (distance2 < minDistance2){
						minDistance2 = distance2;
						particle2 = i;
					}
				}
			}else{

				// find closest particle to each end of the sewing tool:
                for (int i = 0; i < stitcher.Actor1.particleCount;++i){
					Vector3 pos = stitcher.Actor1.GetParticlePosition(stitcher.Actor1.solverIndices[i]);
					float distance1 = (pos - sewingToolHandle1).sqrMagnitude;
					float distance2 = (pos - sewingToolHandle2).sqrMagnitude;
					float min = Mathf.Min(distance1,distance2);
					if (min < minDistance){
						minDistance = min;
						particle1 = i;
					}
				}
		
				minDistance = float.MaxValue;
                for (int i = 0; i < stitcher.Actor2.particleCount;++i){
					Vector3 pos = stitcher.Actor2.GetParticlePosition(stitcher.Actor2.solverIndices[i]);
					float distance1 = (pos - sewingToolHandle1).sqrMagnitude;
					float distance2 = (pos - sewingToolHandle2).sqrMagnitude;
					float min = Mathf.Min(distance1,distance2);
					if (min < minDistance){
						minDistance = min;
						particle2 = i;
					}
				}
			}
		}

		public void DrawSewingTool(){
	        sewingToolHandle1 = Handles.FreeMoveHandle(sewingToolHandle1, Quaternion.identity,HandleUtility.GetHandleSize(sewingToolHandle1)*0.05f,new Vector3(.5f,.5f,.5f),Handles.RectangleHandleCap);
			sewingToolHandle2 = Handles.FreeMoveHandle(sewingToolHandle2, Quaternion.identity,HandleUtility.GetHandleSize(sewingToolHandle2)*0.05f,new Vector3(.5f,.5f,.5f),Handles.RectangleHandleCap);
			Handles.DrawDottedLine(sewingToolHandle1,sewingToolHandle2,2);
		}

		/**
		 * Draws selected stitches in the scene view and allows their selection.
		 */
		public void OnSceneGUI(){

			Array.Resize(ref selectionStatus,stitcher.StitchCount);

			if (!editing)
				return;
			
			DrawSewingTool();

			if (stitcher.Actor1 != null && stitcher.Actor2 != null){

				int controlID = GUIUtility.GetControlID("stitcher".GetHashCode(),FocusType.Passive);
				float distanceToClosest = float.MaxValue;
				int selectedIndex = -1;
				int i = 0;

				foreach(ObiStitcher.Stitch stitch in stitcher.Stitches){
					
					Vector3 pos1 = stitcher.Actor1.GetParticlePosition(stitcher.Actor1.solverIndices[stitch.particleIndex1]);
					Vector3 pos2 = stitcher.Actor2.GetParticlePosition(stitcher.Actor2.solverIndices[stitch.particleIndex2]);
	
					switch (Event.current.GetTypeForControl(controlID)){
						case EventType.MouseDown: 
			
							if (Event.current.button != 0) break;

							// If the user is pressing shift, accumulate selection.
							if ((Event.current.modifiers & EventModifiers.Shift) == 0 && (Event.current.modifiers & EventModifiers.Alt) == 0){
								for(int j = 0; j < selectionStatus.Length; j++)
									selectionStatus[j] = false;
							}

							float distance = HandleUtility.DistanceToLine(pos1,pos2);
							if (distance < 10 && distance < distanceToClosest){

								distanceToClosest = distance;
								selectedIndex = i;
				
								// Prevent deselection if we have selected a stitch:
								GUIUtility.hotControl = controlID;
								Event.current.Use();

							}
						break;
						case EventType.Repaint:
							Handles.color = selectionStatus[i]?Color.red:Color.green;
							Handles.DrawDottedLine(pos1,pos2,2);
						break;
					}
					++i;
				}

				if (selectedIndex >= 0){
					selectionStatus[selectedIndex] = !selectionStatus[selectedIndex];
				}

			}
			
		}
		
	}
}

