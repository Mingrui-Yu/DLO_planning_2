using System;
using UnityEngine;

namespace Obi
{
	[DisallowMultipleComponent]
	public class ObiProfiler : MonoBehaviour
	{
		[Header("Appearance")]
		public GUISkin skin;	
		public Color threadColor = Color.white;
        public Color taskColor = new Color(0.1f, 1, 0.2f);
		public Color parallelTaskColor = new Color(1,0.8f,0.2f);
		//public Color idleColor = new Color(0.7f,0.7f,0.7f);
        public Color renderTaskColor = new Color(0.2f, 0.7f, 1.0f);
        public Color defaultTaskColor = new Color(1, 0.5f, 0.2f);

		[Header("Visualization")]
		public bool showPercentages = false;
		public int profileThrottle = 30;

		private Oni.ProfileInfo[] info;
        private double frameStart;
		private double frameEnd;
		private int frameCounter = 0;
		private int yPos = 25;
        private bool profiling = false;

		private float zoom = 1;
		private Vector2 scrollPosition = Vector2.zero;

		private static ObiProfiler _instance;

	    private void Awake()
	    {
	        if (_instance != null && _instance != this)
	            DestroyImmediate(this);
	        else{
	            _instance = this;
			}
	    }

		public void OnDestroy(){
#if (OBI_ONI_SUPPORTED)
            _instance = null;
            Oni.EnableProfiler(false);
#endif
        }

        private void OnEnable()
        {
#if (OBI_ONI_SUPPORTED)
            if (_instance != null && _instance.profiling)
                Oni.EnableProfiler(true);
#endif
        }

        private void OnDisable()
        {
#if (OBI_ONI_SUPPORTED)
            if (_instance != null)
                Oni.EnableProfiler(false);
#endif
        }

        public static void EnableProfiler()
        {
#if (OBI_ONI_SUPPORTED)
            if (_instance != null)
            {
                _instance.profiling = true;
                if (_instance.isActiveAndEnabled)
                    Oni.EnableProfiler(true);
            }
#endif
        }

        public static void DisableProfiler()
        {
#if (OBI_ONI_SUPPORTED)
            if (_instance != null)
            {
                _instance.profiling = false;
                Oni.EnableProfiler(false);
            }
#endif
        }

        public static void BeginSample(string name, byte type)
        {
#if (OBI_ONI_SUPPORTED)
            if (_instance != null)
                Oni.BeginSample(name, type);
#endif
        }

        public static void EndSample()
        {
#if (OBI_ONI_SUPPORTED)
            if (_instance != null)
                Oni.EndSample();
#endif
        }

		private void UpdateProfilerInfo(){
#if (OBI_ONI_SUPPORTED)
            frameCounter--;
			if (frameCounter <= 0)
			{
				int count = Oni.GetProfilingInfoCount();
				info = new Oni.ProfileInfo[count];
				Oni.GetProfilingInfo(info,count);

				frameCounter = profileThrottle;

				// Calculate frame duration:
                frameStart = double.MaxValue;		
                frameEnd = double.MinValue;
				foreach (Oni.ProfileInfo i in info){
                    frameStart = Math.Min(frameStart,i.start);
                    frameEnd = Math.Max(frameEnd,i.end);
				}
			}

            Oni.ClearProfiler();
#endif
        }

		public void OnGUI()
		{
#if (OBI_ONI_SUPPORTED)
            if (Event.current.type == EventType.Layout)
				UpdateProfilerInfo();

			if (info == null)
				return;

			GUI.skin = skin;
			int toolbarHeight = 20;
			int threadHeight = 20;
			int scrollViewWidth = (int)(Screen.width / zoom);

            double frameDuration = frameEnd - frameStart;
	
			// Toolbar:
			GUI.BeginGroup(new Rect(0,0,Screen.width,toolbarHeight),"","Box");

				GUI.Label(new Rect(5,0,50,toolbarHeight),"Zoom:");
				zoom = GUI.HorizontalSlider(new Rect(50,5,100,toolbarHeight),zoom,0.005f,1);
				GUI.Label(new Rect(Screen.width - 100,0,100,toolbarHeight),(frameDuration/1000.0f).ToString("0.###") + " ms/frame");

			GUI.EndGroup();

			// Timeline view:
			scrollPosition = GUI.BeginScrollView(new Rect(0, toolbarHeight, Screen.width, Screen.height-20), scrollPosition,
												 new Rect(0, 0, scrollViewWidth, yPos+30)); // height depends on amount of threads.

			GUI.color = threadColor;
			GUI.Label(new Rect(5,0,200,20),"Thread 1");
			GUI.Box(new Rect(0, 0, scrollViewWidth, 40),"","Thread");

			yPos = 25;
			uint currentThreadId = 0;
			uint currentLevel = 0;
			foreach (Oni.ProfileInfo i in info)
			{	
				uint threadId = (i.info & (uint)Oni.ProfileMask.ThreadIdMask) >> 16;
				uint level 	  = (i.info & (uint)Oni.ProfileMask.StackLevelMask) >> 8;
				uint type 	  = i.info & (uint)Oni.ProfileMask.TypeMask;

				if (currentThreadId != threadId){
					yPos += threadHeight+1;
					GUI.color = threadColor;
					GUI.Label(new Rect(5,yPos+5,200,20),"Thread "+(threadId+1));
					GUI.Box(new Rect(0, yPos+5, scrollViewWidth, 40),"","Thread");
					yPos += 30;
				}else if (currentLevel != level){
					yPos += threadHeight+1;
				}

				currentLevel = level;
				currentThreadId = threadId;

				switch(type){
					case 0: GUI.color = taskColor; break;
					//case 1: GUI.color = idleColor; break;
					case 2: GUI.color = parallelTaskColor; break;
                    case 3: GUI.color = renderTaskColor; break;
                    default: GUI.color = defaultTaskColor; break;
				}

				// task duration:
                int taskStart = (int) ((i.start - frameStart) / frameDuration * (Screen.width-10) / zoom);
                int taskEnd = (int) ((i.end - frameStart) / frameDuration * (Screen.width-10) / zoom);
				int taskDuration = taskEnd-taskStart;
			
				string name;
				if (showPercentages)
				{
					double pctg = (i.end-i.start)/frameDuration*100;
					name = i.name + " ("+pctg.ToString("0.#")+"%)"; 
				}
				else{
					double ms = (i.end-i.start)/1000.0f;
					name = i.name + " ("+ms.ToString("0.###")+"ms)"; 
				}

				GUI.Box(new Rect(taskStart, yPos, taskDuration-1, threadHeight),name,"Task");
			}
			GUI.EndScrollView();
#endif
        }
    }
}

