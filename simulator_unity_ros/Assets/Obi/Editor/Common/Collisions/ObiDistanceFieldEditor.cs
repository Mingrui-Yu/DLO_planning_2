using UnityEngine;
using UnityEditor;
using System.IO;
using System.Collections;

namespace Obi{
	[CustomEditor(typeof(ObiDistanceField))]
	public class ObiDistanceFieldEditor : Editor
	{

		ObiDistanceField distanceField;

		PreviewHelpers previewHelper;
		Vector2 previewDir;
		Material previewMaterial;

		Mesh previewMesh;
		Texture3D volumeTexture;

		protected IEnumerator routine;

		private void UpdatePreview(){

			CleanupPreview();

			if (distanceField.InputMesh != null){

				previewMesh = CreateMeshForBounds(distanceField.FieldBounds);
				previewMesh.hideFlags = HideFlags.HideAndDontSave;

				volumeTexture = distanceField.GetVolumeTexture(64);
				volumeTexture.hideFlags = HideFlags.HideAndDontSave;

				previewMaterial = Resources.Load<Material>("DistanceFieldPreview");
				previewMaterial.SetTexture("_Volume",volumeTexture);
				previewMaterial.SetVector("_AABBMin",-distanceField.FieldBounds.extents);
                previewMaterial.SetVector("_AABBMax",distanceField.FieldBounds.extents);
			}

		}

		private void CleanupPreview(){
			GameObject.DestroyImmediate(previewMesh);
			GameObject.DestroyImmediate(volumeTexture);
		}

		public void OnEnable(){
			distanceField = (ObiDistanceField) target;
			previewHelper = new PreviewHelpers();
			UpdatePreview();
		}

		public void OnDisable(){
			EditorUtility.ClearProgressBar();
			previewHelper.Cleanup();
			CleanupPreview();
		}

		public override void OnInspectorGUI() {

			serializedObject.UpdateIfRequiredOrScript();	

			Editor.DrawPropertiesExcluding(serializedObject,"m_Script");

			GUI.enabled = (distanceField.InputMesh != null);
			if (GUILayout.Button("Generate")){
				// Start a coroutine job in the editor.
				EditorUtility.SetDirty(target);
				CoroutineJob job = new CoroutineJob();
				routine = job.Start( distanceField.Generate());
				EditorCoroutine.ShowCoroutineProgressBar("Generating distance field",ref routine);
				UpdatePreview();
				EditorGUIUtility.ExitGUI();
			}
			GUI.enabled = true;		

			int nodeCount = (distanceField.nodes != null ? distanceField.nodes.Count : 0);
			float resolution = distanceField.FieldBounds.size.x / distanceField.EffectiveSampleSize;
			EditorGUILayout.HelpBox("Nodes: "+ nodeCount+"\n"+
									"Size in memory: "+ (nodeCount * 0.062f).ToString("0.#") +" kB\n"+
									"Compressed to: " + (nodeCount / Mathf.Pow(resolution,3) * 100).ToString("0.##") + "%",MessageType.Info);

			if (GUI.changed)
				serializedObject.ApplyModifiedProperties();

		}

		public override bool HasPreviewGUI(){
			return true;
		}

		public override void OnInteractivePreviewGUI(Rect region, GUIStyle background)
		{
			previewDir = PreviewHelpers.Drag2D(previewDir, region);

			if (Event.current.type != EventType.Repaint || previewMesh == null)
			{
                return;
            }

			Quaternion quaternion = Quaternion.Euler(this.previewDir.y, 0f, 0f) * Quaternion.Euler(0f, this.previewDir.x, 0f) * Quaternion.Euler(0, 120, -20f);

			previewHelper.BeginPreview(region, background);

			Bounds bounds = previewMesh.bounds;
			float magnitude = Mathf.Sqrt(bounds.extents.sqrMagnitude);
			float num = 4f * magnitude;
			previewHelper.m_Camera.transform.position = -Vector3.forward * num;
			previewHelper.m_Camera.transform.rotation = Quaternion.identity;
			previewHelper.m_Camera.nearClipPlane = num - magnitude * 1.1f;
			previewHelper.m_Camera.farClipPlane = num + magnitude * 1.1f;

			// Compute matrix to rotate the mesh around the center of its bounds:
			Matrix4x4 matrix = Matrix4x4.TRS(Vector3.zero,quaternion,Vector3.one) * Matrix4x4.TRS(-bounds.center,Quaternion.identity,Vector3.one);

			Graphics.DrawMesh(previewMesh, matrix, previewMaterial,1, previewHelper.m_Camera, 0);

			Texture texture = previewHelper.EndPreview();
			GUI.DrawTexture(region, texture, ScaleMode.StretchToFill, true);

        }

		/**
		 * Creates a solid mesh from some Bounds. This is used to display the distance field volumetric preview.
		 */
		private Mesh CreateMeshForBounds(Bounds b){
			Mesh m = new Mesh();

			/** Indices of bounds corners:

			  		   Y
			  		   2	   6
			    	   +------+
			 	  3  .'|  7 .'|
				   +---+--+'  |
				   |   |  |   |
				   |   +--+---+   X
				   | .' 0 | .' 4
				   +------+'
				Z 1        5

			*/
			Vector3[] vertices = new Vector3[8]{
				b.center + new Vector3(-b.extents.x,-b.extents.y,-b.extents.z), //0
				b.center + new Vector3(-b.extents.x,-b.extents.y,b.extents.z),  //1
				b.center + new Vector3(-b.extents.x,b.extents.y,-b.extents.z),  //2
				b.center + new Vector3(-b.extents.x,b.extents.y,b.extents.z),   //3
				b.center + new Vector3(b.extents.x,-b.extents.y,-b.extents.z),  //4
				b.center + new Vector3(b.extents.x,-b.extents.y,b.extents.z),   //5
				b.center + new Vector3(b.extents.x,b.extents.y,-b.extents.z),   //6
				b.center + new Vector3(b.extents.x,b.extents.y,b.extents.z)     //7
			};
			int[] triangles = new int[36]{
				2,3,7,
				6,2,7,

				7,5,4,
				6,7,4,

				3,1,5,
				7,3,5,

				2,0,3,
				3,0,1,

				6,4,2,
				2,4,0,

				4,5,0,
				5,1,0
			};

			m.vertices = vertices;
			m.triangles = triangles;
			return m;
		}
	}
}
