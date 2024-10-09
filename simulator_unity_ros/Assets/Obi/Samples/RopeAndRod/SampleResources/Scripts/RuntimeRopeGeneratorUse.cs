using System;
using System.Collections;
using UnityEngine;
using Obi;

public class RuntimeRopeGeneratorUse : MonoBehaviour
{
	public ObiCollider pendulum;
	RuntimeRopeGenerator rg;

	public IEnumerator Start()
	{
		rg = new RuntimeRopeGenerator();

		// Create a rope:
		yield return rg.MakeRope(transform,Vector3.zero,1);

		// Add a pendulum (you should adjust the attachment point depending on your particular pendulum object)
		rg.AddPendulum(pendulum,Vector3.up*0.5f); 
	}

	public void Update(){
	
		if (Input.GetKey(KeyCode.W)){
			rg.ChangeRopeLength(- Time.deltaTime);
		}

		if (Input.GetKey(KeyCode.S)){
			rg.ChangeRopeLength(  Time.deltaTime);
		}
		
	}
}


