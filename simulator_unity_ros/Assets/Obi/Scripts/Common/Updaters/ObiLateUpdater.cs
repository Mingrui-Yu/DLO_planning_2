using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

namespace Obi
{
    /// <summary>
    /// Updater class that will perform simulation during LateUpdate(). This is highly unphysical and should be avoided whenever possible.
    /// This updater does not make any accuracy guarantees when it comes to two-way coupling with rigidbodies.
    /// It is only provided for the odd case when there's no way to perform simulation with a fixed timestep.
    /// If in doubt, use the ObiFixedUpdater component instead.
    /// </summary>
    [AddComponentMenu("Physics/Obi/Obi Late Updater", 802)]
    public class ObiLateUpdater : ObiUpdater
    {
        [Tooltip("Smoothing factor fo the timestep (smoothDelta). Values closer to 1 will yield stabler simulation, but it will be off-sync with rendering.")]
        [Range(0,1)]
        public float deltaSmoothing = 0.95f;

        [Tooltip("Target timestep used to advance the simulation. The updater will interpolate this value with Time.deltaTime to find the actual timestep used for each frame.")]
        private float smoothDelta = 0.02f;

		private void OnValidate()
		{
            smoothDelta = Mathf.Max(0.0001f, smoothDelta);
		}

		void LateUpdate()
        {
            if (Time.deltaTime > 0)
            {
                if (Application.isPlaying)
                {
                    BeginStep(Time.fixedDeltaTime);

                    // smooth out timestep:
                    smoothDelta = Mathf.Lerp(Time.deltaTime, smoothDelta, deltaSmoothing);

                    Substep(smoothDelta);

                    EndStep(smoothDelta);
                }

                Interpolate(smoothDelta, smoothDelta);
            }
        }
    }
}