using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Obi;

[ExecuteInEditMode]
[RequireComponent(typeof(ObiActor))]
public class DebugParticleFrames : MonoBehaviour {

	ObiActor actor;
	public float size = 1;
	
	public void Awake()
    {
		actor = GetComponent<ObiActor>();
	}
	
	// Update is called once per frame
	void OnDrawGizmos () 
    {
        Vector4 b1 = new Vector4(1, 0, 0, 0);
        Vector4 b2 = new Vector4(0, 1, 0, 0);
        Vector4 b3 = new Vector4(0, 0, 1, 0);
        for (int i = 0; i < actor.activeParticleCount; ++i)
        {

            Vector3 position = actor.GetParticlePosition(actor.solverIndices[i]);
            Quaternion quat = actor.GetParticleOrientation(actor.solverIndices[i]);
 
            Gizmos.color = Color.red;
            Gizmos.DrawRay(position, quat * b1 * size);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(position, quat * b2 * size);
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(position, quat * b3 * size);
		}
	
	}
}
