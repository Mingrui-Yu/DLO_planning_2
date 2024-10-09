using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RopeTenser : MonoBehaviour
{
	public float force = 10;

    // Update is called once per frame
    void Update()
    {
		GetComponent<Rigidbody>().AddForce(Vector3.down * force);
    }
}
