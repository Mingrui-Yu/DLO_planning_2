using UnityEngine;
#if (UNITY_EDITOR)
	using UnityEditor;
#endif
using System.Collections;

namespace Obi
{

public class EditorCoroutine
{

	public static void ShowCoroutineProgressBar(string title, ref IEnumerator coroutine){
		
		#if (UNITY_EDITOR)
		if (coroutine != null){

			CoroutineJob.ProgressInfo progressInfo;

			do{
				if (!coroutine.MoveNext())
					progressInfo = null;
				else 
					progressInfo = coroutine.Current as CoroutineJob.ProgressInfo;
			
				if (progressInfo != null && EditorUtility.DisplayCancelableProgressBar(title, progressInfo.userReadableInfo, progressInfo.progress)){
					progressInfo = null;
				}
			}while (progressInfo != null);

			// once finished, clear progress bar and set coroutine to null.
			coroutine = null;

			// Unity bug here: https://issuetracker.unity3d.com/issues/unity-throws-nullreferenceexception-or-endlayoutgroup-errors-when-editorutility-dot-clearprogressbar-is-called
			EditorUtility.ClearProgressBar();

		}
		#endif

	}
		
}
}