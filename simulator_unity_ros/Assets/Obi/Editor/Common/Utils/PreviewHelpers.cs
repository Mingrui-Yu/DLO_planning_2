using System;
using UnityEditor;
using UnityEngine;

namespace Obi{

	class PreviewHelpers
	{

		// Preview interaction related stuff:
		static int sliderHash = "Slider".GetHashCode();
		public static Vector2 Drag2D(Vector2 scrollPosition, Rect position)
		{
			int controlID = GUIUtility.GetControlID(PreviewHelpers.sliderHash, FocusType.Passive);
			Event current = Event.current;
			switch (current.GetTypeForControl(controlID))
			{
			case EventType.MouseDown:
				if (position.Contains(current.mousePosition) && position.width > 50f)
				{
					GUIUtility.hotControl = controlID;
					current.Use();
					EditorGUIUtility.SetWantsMouseJumping(1);
				}
				break;
			case EventType.MouseUp:
				if (GUIUtility.hotControl == controlID)
				{
					GUIUtility.hotControl = 0;
				}
				EditorGUIUtility.SetWantsMouseJumping(0);
				break;
			case EventType.MouseDrag:
				if (GUIUtility.hotControl == controlID)
				{
					scrollPosition -= current.delta * (float)((!current.shift) ? 1 : 3) / Mathf.Min(position.width, position.height) * 140f;
					scrollPosition.y = Mathf.Clamp(scrollPosition.y, -90f, 90f);
					current.Use();
					GUI.changed = true;
				}
				break;
			}
			return scrollPosition;
		}

		public Camera m_Camera;
		public float m_CameraFieldOfView = 30f;
		public Light[] m_Light = new Light[2];
		internal RenderTexture m_RenderTexture;
		public PreviewHelpers() : this(false)
		{
		}
		public PreviewHelpers(bool renderFullScene)
		{
			GameObject gameObject = EditorUtility.CreateGameObjectWithHideFlags("PreRenderCamera", HideFlags.HideAndDontSave, new Type[]
			                                                                    {
				typeof(Camera)
			});
			this.m_Camera = gameObject.GetComponent<Camera>();
			this.m_Camera.fieldOfView = this.m_CameraFieldOfView;
			this.m_Camera.cullingMask = 1 << 1;
			this.m_Camera.enabled = false;
			this.m_Camera.clearFlags = CameraClearFlags.SolidColor;
			this.m_Camera.farClipPlane = 10f;
			this.m_Camera.nearClipPlane = 1f;
			this.m_Camera.backgroundColor = new Color(0.192156866f, 0.192156866f, 0.192156866f, 0);
			this.m_Camera.renderingPath = RenderingPath.Forward;
			this.m_Camera.useOcclusionCulling = false;

			for (int i = 0; i < 2; i++)
			{
				GameObject gameObject2 = EditorUtility.CreateGameObjectWithHideFlags("PreRenderLight", HideFlags.HideAndDontSave, new Type[]
				                                                                     {
					typeof(Light)
				});
				this.m_Light[i] = gameObject2.GetComponent<Light>();
				this.m_Light[i].type = LightType.Directional;
				this.m_Light[i].intensity = 1f;
				this.m_Light[i].enabled = false;
			}

			this.m_Light[0].color = new Color(0.4f, 0.4f, 0.45f, 0f);
			this.m_Light[1].transform.rotation = Quaternion.Euler(340f, 218f, 177f);
			this.m_Light[1].color = new Color(0.4f, 0.4f, 0.45f, 0f) * 0.7f;
		}
		public void Cleanup()
		{
			if (this.m_Camera)
			{
				UnityEngine.Object.DestroyImmediate(this.m_Camera.gameObject, true);
			}
			if (this.m_RenderTexture)
			{
				UnityEngine.Object.DestroyImmediate(this.m_RenderTexture);
				this.m_RenderTexture = null;
			}
			Light[] light = this.m_Light;
			for (int i = 0; i < light.Length; i++)
			{
				Light light2 = light[i];
				if (light2)
				{
					UnityEngine.Object.DestroyImmediate(light2.gameObject, true);
				}
			}
		}

		private void InitPreview(Rect r)
		{
			int num = (int)r.width;
			int num2 = (int)r.height;
			if (!this.m_RenderTexture || this.m_RenderTexture.width != num || this.m_RenderTexture.height != num2)
			{
				if (this.m_RenderTexture)
				{
					UnityEngine.Object.DestroyImmediate(this.m_RenderTexture);
					this.m_RenderTexture = null;
				}
				float scaleFactor = this.GetScaleFactor((float)num, (float)num2);
				this.m_RenderTexture = new RenderTexture((int)((float)num * scaleFactor), (int)((float)num2 * scaleFactor), 16);
				this.m_RenderTexture.hideFlags = HideFlags.HideAndDontSave;
				this.m_Camera.targetTexture = this.m_RenderTexture;
			}
			float num3 = (this.m_RenderTexture.width > 0) ? Mathf.Max(1f, (float)this.m_RenderTexture.height / (float)this.m_RenderTexture.width) : 1f;
			this.m_Camera.fieldOfView = Mathf.Atan(num3 * Mathf.Tan(this.m_CameraFieldOfView * 0.5f * 0.0174532924f)) * 57.29578f * 2f;

		}
		public float GetScaleFactor(float width, float height)
		{
			float a = Mathf.Max(Mathf.Min(width * 2f, 1024f), width) / width;
			float b = Mathf.Max(Mathf.Min(height * 2f, 1024f), height) / height;
			return Mathf.Min(a, b);
		}

		public void BeginPreview(Rect r, GUIStyle previewBackground)
		{
			this.InitPreview(r);
			if (previewBackground == null || previewBackground == GUIStyle.none)
			{
				return;
			}
		}
		public Texture EndPreview()
		{
			m_Camera.Render();
			return this.m_RenderTexture;
		}

	}
}
