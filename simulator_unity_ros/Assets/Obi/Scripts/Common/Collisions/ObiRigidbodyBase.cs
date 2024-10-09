using UnityEngine;
using Unity.Profiling;
using System;
using System.Collections;

namespace Obi{

	/**
	 * Small helper class that lets you specify Obi-only properties for rigidbodies.
	 */

	[ExecuteInEditMode]
	public abstract class ObiRigidbodyBase : MonoBehaviour
	{

        public bool kinematicForParticles = false;

        public ObiRigidbodyHandle handle;


		public virtual void Awake()
        {
            handle = ObiColliderWorld.GetInstance().CreateRigidbody();
            handle.owner = this;
            UpdateIfNeeded(1);
		}

		public void OnDestroy()
        {
            ObiColliderWorld.GetInstance().DestroyRigidbody(handle);
        }

		public abstract void UpdateIfNeeded(float stepTime);

		/**
		 * Reads velocities back from the solver.
		 */
		public abstract void UpdateVelocities(Vector3 linearDelta, Vector3 angularDelta);

	}
}

