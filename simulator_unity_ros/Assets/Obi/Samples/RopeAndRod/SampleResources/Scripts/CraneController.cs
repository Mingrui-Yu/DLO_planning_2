using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Obi;

public class CraneController : MonoBehaviour {

	ObiRopeCursor cursor;
	ObiRope rope;

	// Use this for initialization
	void Start () {
		cursor = GetComponentInChildren<ObiRopeCursor>();
		rope = cursor.GetComponent<ObiRope>();
	}
	
	// Update is called once per frame
	void Update () {
		if (Input.GetKey(KeyCode.W)){
			if (rope.restLength > 6.5f)
				cursor.ChangeLength(rope.restLength - 1f * Time.deltaTime);
		}

		if (Input.GetKey(KeyCode.S)){
			cursor.ChangeLength(rope.restLength + 1f * Time.deltaTime);
		}

		if (Input.GetKey(KeyCode.A)){
			transform.Rotate(0,Time.deltaTime*15f,0);
		}

		if (Input.GetKey(KeyCode.D)){
			transform.Rotate(0,-Time.deltaTime*15f,0);
		}
	}
}
