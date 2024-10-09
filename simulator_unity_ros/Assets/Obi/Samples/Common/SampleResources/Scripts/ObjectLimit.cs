using UnityEngine;
 using System.Collections;
 
 public class ObjectLimit : MonoBehaviour 
 {
 
	 public float minX = 0;
	 public float maxX = 1;
	 public float minY = 0;
	 public float maxY = 1;
	 public float minZ = 0;
	 public float maxZ = 1;
	 
	 void Update()
	 {
		transform.localPosition = new Vector3(Mathf.Clamp(gameObject.transform.localPosition.x,minX,maxX),
										 	  Mathf.Clamp(gameObject.transform.localPosition.y,minY,maxY),
										 	  Mathf.Clamp(gameObject.transform.localPosition.z,minZ,maxZ));
	 
	 }
 
 }