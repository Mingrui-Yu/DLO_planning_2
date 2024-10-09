using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{

    /**
	 * Add this component to any Collider that you want to be considered by Obi.
	 */
    [ExecuteInEditMode]
    [RequireComponent(typeof(Collider2D))]
    public class ObiCollider2D : ObiColliderBase
    {
        [SerializeProperty("SourceCollider")]
        [SerializeField] private Collider2D sourceCollider;

        public Collider2D SourceCollider
        {
            set
            {
                if (value.gameObject != this.gameObject)
                {
                    Debug.LogError("The Collider component must reside in the same GameObject as ObiCollider.");
                }
                else
                {
                    sourceCollider = value;

                    RemoveCollider();
                    AddCollider();
                }
            }
            get { return sourceCollider; }
        }

        /**
		 * Creates an OniColliderTracker of the appropiate type.
   		 */
        protected override void CreateTracker()
        {

            if (tracker != null)
            {
                tracker.Destroy();
                tracker = null;
            }

            if (sourceCollider is CircleCollider2D)
                tracker = new ObiCircleShapeTracker2D(this,(CircleCollider2D)sourceCollider);
            else if (sourceCollider is BoxCollider2D)
                tracker = new ObiBoxShapeTracker2D(this,(BoxCollider2D)sourceCollider);
            else if (sourceCollider is CapsuleCollider2D)
                tracker = new ObiCapsuleShapeTracker2D(this,(CapsuleCollider2D)sourceCollider);
            else if (sourceCollider is EdgeCollider2D)
                tracker = new ObiEdgeShapeTracker2D(this,(EdgeCollider2D)sourceCollider);
            else
                Debug.LogWarning("Collider2D type not supported by Obi.");

        }

        protected override Component GetUnityCollider(ref bool enabled)
        {

            if (sourceCollider != null)
                enabled = sourceCollider.enabled;

            return sourceCollider;
        }

        protected override void FindSourceCollider()
        {
            if (SourceCollider == null)
                SourceCollider = GetComponent<Collider2D>();
            else
                AddCollider();
        }

    }
}

