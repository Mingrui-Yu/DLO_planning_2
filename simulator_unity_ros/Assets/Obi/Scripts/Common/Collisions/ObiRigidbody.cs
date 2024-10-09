using UnityEngine;
using System;
using System.Collections;

namespace Obi{

	/**
	 * Small helper class that lets you specify Obi-only properties for rigidbodies.
	 */

	[ExecuteInEditMode]
	[RequireComponent(typeof(Rigidbody))]
	public class ObiRigidbody : ObiRigidbodyBase
	{
		private Rigidbody unityRigidbody;
        private Quaternion prevRotation;
        private Vector3 prevPosition;

        public override void Awake(){
			unityRigidbody = GetComponent<Rigidbody>();
            prevPosition = transform.position;
            prevRotation = transform.rotation;
            base.Awake();
		}

        private void UpdateKinematicVelocities(float stepTime)
        {
            // differentiate positions/orientations to get our own velocites for kinematic objects.
            // when calling Physics.Simulate, MovePosition/Rotation do not work correctly. Also useful for animations.
            if (unityRigidbody.isKinematic)
            {
                // differentiate positions to obtain linear velocity:
                unityRigidbody.velocity = (transform.position - prevPosition) / stepTime;

                // differentiate rotations to obtain angular velocity:
                Quaternion delta = transform.rotation * Quaternion.Inverse(prevRotation);
                unityRigidbody.angularVelocity = new Vector3(delta.x, delta.y, delta.z) * 2.0f / stepTime;
            }

            prevPosition = transform.position;
            prevRotation = transform.rotation;
        }

		public override void UpdateIfNeeded(float stepTime)
        {
            UpdateKinematicVelocities(stepTime);

            var rb = ObiColliderWorld.GetInstance().rigidbodies[handle.index];
            rb.FromRigidbody(unityRigidbody, kinematicForParticles);
            ObiColliderWorld.GetInstance().rigidbodies[handle.index] = rb;

        }

		/**
		 * Reads velocities back from the solver.
		 */
		public override void UpdateVelocities(Vector3 linearDelta, Vector3 angularDelta)
        {
			// kinematic rigidbodies are passed to Obi with zero velocity, so we must ignore the new velocities calculated by the solver:
			if (Application.isPlaying && !(unityRigidbody.isKinematic || kinematicForParticles))
            {
                unityRigidbody.velocity += linearDelta;
                unityRigidbody.angularVelocity += angularDelta;
            }
        }
	}
}

