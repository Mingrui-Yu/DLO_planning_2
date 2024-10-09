using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using Obi;

[RequireComponent(typeof(ObiSolver))]
public class WrapRopeGameController : MonoBehaviour
{

	ObiSolver solver;

	public Wrappable[] wrappables;
	public UnityEvent onFinish = new UnityEvent();

	private void Awake()
	{
		solver = GetComponent<ObiSolver>();
	}

	// Start is called before the first frame update
	void OnEnable()
	{
		solver.OnCollision += Solver_OnCollision;
	}

	private void OnDisable()
	{
		solver.OnCollision -= Solver_OnCollision;
	}

	private void Update()
	{
		bool allWrapped = true;

        // Test our win condition: all pegs must be wrapped.
		foreach (var wrappable in wrappables)
		{
			if (!wrappable.IsWrapped())
			{
				allWrapped = false;
				break;
			}
		}

		if (allWrapped)
			onFinish.Invoke();
	}

	private void Solver_OnCollision(ObiSolver s, ObiSolver.ObiCollisionEventArgs e)
	{
		// reset to unwrapped state:
		foreach (var wrappable in wrappables)
			wrappable.Reset();

		var world = ObiColliderWorld.GetInstance();
		foreach (Oni.Contact contact in e.contacts)
		{
			// look for actual contacts only:
			if (contact.distance < 0.025f)
			{
				var col = world.colliderHandles[contact.other].owner;
				if (col != null)
				{
					var wrappable = col.GetComponent<Wrappable>();
                    if (wrappable != null)
						wrappable.SetWrapped();
				}
			}
		}
	}
}
