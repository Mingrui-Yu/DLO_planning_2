using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CharacterControl2D : MonoBehaviour {

	public float acceleration = 10;
	public float maxSpeed = 8;
	public float jumpPower = 2;

	private Rigidbody unityRigidbody;
	
	public void Awake(){
		unityRigidbody = GetComponent<Rigidbody>();
	}

	
	void FixedUpdate () {

		unityRigidbody.AddForce(new Vector3(Input.GetAxis("Horizontal")*acceleration,0,0));
	
		unityRigidbody.velocity = Vector3.ClampMagnitude(unityRigidbody.velocity,maxSpeed);
		
		if (Input.GetButtonDown("Jump")){
			unityRigidbody.AddForce(Vector3.up * jumpPower,ForceMode.VelocityChange);
		}
	}
}
