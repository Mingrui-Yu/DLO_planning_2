using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

namespace Obi
{
    /// <summary>
    /// Updater class that will perform simulation during FixedUpdate(). This is the most physically correct updater,
    /// and the one to be used in most cases. Also allows to perform substepping, greatly improving convergence.
                  
    [AddComponentMenu("Physics/Obi/Obi Fixed Updater", 801)]
    [ExecuteInEditMode]
    public class ObiFixedUpdater : ObiUpdater
    {
        /// <summary>
        /// Enabling this will set Physics.autoSimulation to false, and allow the updater to update both Obi's and Unity's engines in sync. 
        /// Can be left off if the amount of substeps is 1, should be enabled for any other substep count if your scene contains dynamic attachments or particle-rigidbody collisions.
        /// </summary>
        [Tooltip("Should be left enabled when using multiple substeps, if you scene contains two-way rigidbody coupling: dynamic constraints and/or rigidbody collisions. If only using 1 substep or no" +
                 "rigidbody coupling, can be disabled.")]
        public bool substepUnityPhysics = false;

        /// <summary>
        /// Each FixedUpdate() call will be divided into several substeps. Performing more substeps will greatly improve the accuracy/convergence speed of the simulation. 
        /// Increasing the amount of substeps is more effective than increasing the amount of constraint iterations.
        /// </summary>
        [Tooltip("Amount of substeps performed per FixedUpdate. Increasing the amount of substeps greatly improves accuracy and convergence speed.")]
        public int substeps = 1;

        private float accumulatedTime;

        private void OnValidate()
        {
            substeps = Mathf.Max(1, substeps);
        }

        private void Awake()
        {
            accumulatedTime = 0;
        }

        private void OnDisable()
        {
            Physics.autoSimulation = true;
        }

        private void FixedUpdate()
        {
            ObiProfiler.EnableProfiler();

            Physics.autoSimulation = !substepUnityPhysics;

            BeginStep(Time.fixedDeltaTime);

            float substepDelta = Time.fixedDeltaTime / (float)substeps;

            // Divide the step into multiple smaller substeps:
            for (int i = 0; i < substeps; ++i)
            {
                // Simulate Obi:
                Substep(substepDelta);

                // Simulate Unity physics:
                if (substepUnityPhysics)
                    Physics.Simulate(substepDelta);
            }

            EndStep(substepDelta);

            ObiProfiler.DisableProfiler();

            accumulatedTime -= Time.fixedDeltaTime;
        }

        private void Update()
        {
            ObiProfiler.EnableProfiler();
            Interpolate(Time.fixedDeltaTime, accumulatedTime);
            ObiProfiler.DisableProfiler();

            accumulatedTime += Time.deltaTime;
        }
    }
}