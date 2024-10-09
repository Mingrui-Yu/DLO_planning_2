using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
	/**
	 * Sample script that colors fluid particles based on their vorticity (2D only)
	 */
	[RequireComponent(typeof(ObiActor))]
	public class ColorFromVelocity : MonoBehaviour
	{
		ObiActor actor;
		public float sensibility = 0.2f;

		void Awake(){
			actor = GetComponent<ObiActor>();
		}

		public void OnEnable(){}

		void LateUpdate()
		{
            if (!isActiveAndEnabled || actor.solver == null)
				return;

            for (int i = 0; i < actor.solverIndices.Length; ++i){

				int k = actor.solverIndices[i];

				Vector4 vel = actor.solver.velocities[k];

                actor.solver.colors[k] = new Color(Mathf.Clamp(vel.x / sensibility,-1,1) * 0.5f + 0.5f,
											       Mathf.Clamp(vel.y / sensibility,-1,1) * 0.5f + 0.5f,
											       Mathf.Clamp(vel.z / sensibility,-1,1) * 0.5f + 0.5f,1);

			}
		}
	
	}
}

