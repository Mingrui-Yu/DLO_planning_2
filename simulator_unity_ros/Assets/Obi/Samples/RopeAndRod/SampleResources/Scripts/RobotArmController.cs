using UnityEngine;
using System.Collections;

public class RobotArmController : MonoBehaviour {

	public Transform section1 = null;
	public Transform section2 = null;
	public Transform actuator = null;
	public float speed = 40;

	// Update is called once per frame
	void Update () {
	
		if (Input.GetKey(KeyCode.A)){
			section1.Rotate(0,speed*Time.deltaTime,0,Space.World);
		}
		if (Input.GetKey(KeyCode.D)){
			section1.Rotate(0,-speed*Time.deltaTime,0,Space.World);
		}

		if (Input.GetKey(KeyCode.W)){
			section1.Rotate(0,speed*Time.deltaTime,0,Space.Self);
		}
		if (Input.GetKey(KeyCode.S)){
			section1.Rotate(0,-speed*Time.deltaTime,0,Space.Self);
		}
		if (Input.GetKey(KeyCode.T)){
			section2.Rotate(0,speed*Time.deltaTime,0,Space.Self);
		}
		if (Input.GetKey(KeyCode.G)){
			section2.Rotate(0,-speed*Time.deltaTime,0,Space.Self);
		}
		if (Input.GetKey(KeyCode.Y)){
			actuator.Rotate(0,speed*Time.deltaTime,0,Space.Self);
		}
		if (Input.GetKey(KeyCode.H)){
			actuator.Rotate(0,-speed*Time.deltaTime,0,Space.Self);
		}

	}
}
