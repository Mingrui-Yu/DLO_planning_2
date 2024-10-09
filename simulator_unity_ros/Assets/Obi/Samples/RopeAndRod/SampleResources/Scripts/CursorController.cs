using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Obi;

[RequireComponent(typeof(ObiRope))]
public class CursorController : MonoBehaviour
{

	ObiRopeCursor cursor;
	ObiRope rope;
	public float minLength = 0.1f;
    public float speed = 1;

	// Use this for initialization
	void Start ()
    {
        rope = GetComponent<ObiRope>();
        cursor = GetComponent<ObiRopeCursor>();
	}
	
	// Update is called once per frame
	void Update ()
    {
		if (Input.GetKey(KeyCode.W) && cursor != null)
        {
            if (rope.restLength > minLength)
                cursor.ChangeLength(rope.restLength - speed * Time.deltaTime);
		}

		if (Input.GetKey(KeyCode.S) && cursor != null)
        {
            cursor.ChangeLength(rope.restLength + speed * Time.deltaTime);
		}

		if (Input.GetKey(KeyCode.A)){
			rope.transform.Translate(Vector3.left * Time.deltaTime,Space.World);
		}

		if (Input.GetKey(KeyCode.D)){
			rope.transform.Translate(Vector3.right * Time.deltaTime,Space.World);
		}

	}
}
