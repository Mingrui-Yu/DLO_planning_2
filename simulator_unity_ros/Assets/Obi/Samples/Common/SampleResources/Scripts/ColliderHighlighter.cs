using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Obi;

[RequireComponent(typeof(ObiSolver))]
public class ColliderHighlighter : MonoBehaviour {

 	ObiSolver solver;

	void Awake(){
		solver = GetComponent<Obi.ObiSolver>();
	}

	void OnEnable () {
		solver.OnCollision += Solver_OnCollision;
	}

	void OnDisable(){
		solver.OnCollision -= Solver_OnCollision;
	}
	
	void Solver_OnCollision (object sender, Obi.ObiSolver.ObiCollisionEventArgs e)
	{
        var colliderWorld = ObiColliderWorld.GetInstance();

        Oni.Contact[] contacts = e.contacts.Data;
		for(int i = 0; i < e.contacts.Count; ++i)
		{
			Oni.Contact c = contacts[i];
			// make sure this is an actual contact:
			if (c.distance < 0.01f)
			{
				// get the collider:
				var col = colliderWorld.colliderHandles[c.other].owner;

				if (col != null)
                {
					// make it blink:
					Blinker blinker = col.GetComponent<Blinker>();
	
					if (blinker)
						blinker.Blink();
				}
			}
		}
	}
}
