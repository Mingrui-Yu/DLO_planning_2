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
	public class ColorFromPhase : MonoBehaviour
	{
		ObiActor actor;

		void Awake(){
			actor = GetComponent<ObiActor>();
		}

		void LateUpdate()
		{
            if (!isActiveAndEnabled || actor.solver == null)
				return;

            for (int i = 0; i < actor.solverIndices.Length; ++i)
            {

				int k = actor.solverIndices[i];
				int phase = ObiUtils.GetGroupFromPhase(actor.solver.phases[k]);

                actor.solver.colors[k] = ObiUtils.colorAlphabet[phase % ObiUtils.colorAlphabet.Length];

            }
		}
	
	}
}

