#if (OBI_ONI_SUPPORTED)
using UnityEngine;
using System;
using System.Collections;

namespace Obi
{
    public class OniBackend : IObiBackend
    {
        private OniColliderWorld colliderGrid;

        #region Solver
        public ISolverImpl CreateSolver(ObiSolver solver, int capacity)
        {
            GetOrCreateColliderWorld();
            colliderGrid.IncreaseReferenceCount();
            return new OniSolverImpl(Oni.CreateSolver(capacity));
        }
        public void DestroySolver(ISolverImpl solver)
        {
            if (solver != null)
            {
                if (colliderGrid != null)
                    colliderGrid.DecreaseReferenceCount();
                solver.Destroy();
            }
        }

        // Single type of collision world. Each solver implementation should manage the data as it can.  
        private void GetOrCreateColliderWorld()
        {
            colliderGrid = GameObject.FindObjectOfType<OniColliderWorld>();
            if (colliderGrid == null)
            {
                var world = new GameObject("OniCollisionWorld", typeof(OniColliderWorld));
                colliderGrid = world.GetComponent<OniColliderWorld>();
            }
        }
        #endregion

    }
}
#endif