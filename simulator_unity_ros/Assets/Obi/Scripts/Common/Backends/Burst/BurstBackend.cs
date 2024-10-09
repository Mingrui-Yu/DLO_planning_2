#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using UnityEngine;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Collections;
using System;
using System.Collections;

namespace Obi
{
    public class BurstBackend : IObiBackend
    {
#region Solver
        public ISolverImpl CreateSolver(ObiSolver solver, int capacity)
        {
            return new BurstSolverImpl(solver);
        }
        public void DestroySolver(ISolverImpl  solver)
        {
            if (solver != null)
                solver.Destroy();
        }
#endregion

    }
}
#endif