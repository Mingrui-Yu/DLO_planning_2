using UnityEngine;
using System;
using System.Collections;

namespace Obi
{
    /**
     * Base class for backend implementations. 
     */
    public interface IObiBackend
    {
        #region Solver
        ISolverImpl CreateSolver(ObiSolver solver, int capacity);
        void DestroySolver(ISolverImpl solver);
        #endregion
    }

}