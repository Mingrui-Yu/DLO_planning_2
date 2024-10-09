using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Obi
{

    [RequireComponent(typeof(ObiSolver))]
    public class ObiParticleGridDebugger : MonoBehaviour
    {

	    ObiSolver solver;
        ObiNativeAabbList cells;

        void OnEnable()
        {
            solver = GetComponent<ObiSolver>();
            cells = new ObiNativeAabbList();
        }

        private void OnDisable()
        {
            cells.Dispose();
        }

        void LateUpdate ()
	    {
            cells.count = solver.implementation.GetParticleGridSize();
            solver.implementation.GetParticleGrid(cells);
	    }

	    void OnDrawGizmos()
        {

		    if (cells != null)
            {
                Gizmos.color = Color.yellow;
                for(int i = 0; i < cells.count; ++i)
			        Gizmos.DrawWireCube(cells[i].center, cells[i].size);
            }

        }

    }
}
