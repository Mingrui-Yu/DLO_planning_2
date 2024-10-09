using UnityEngine;
using UnityEngine.Serialization;

namespace Obi
{

    /**
	 * Add this component to any Collider that you want to be considered by Obi.
	 */
    [ExecuteInEditMode]
    [RequireComponent(typeof(Collider))]
    public class ObiCollider : ObiColliderBase
    {

        [SerializeProperty("sourceCollider")]
        [FormerlySerializedAs("SourceCollider")]
        [SerializeField] private Collider m_SourceCollider;

        /// <summary>
        /// The Unity collider that this ObiCollider should mimic.
        /// </summary>
        /// This is automatically set when you first create the ObiCollider component, but you can override it afterwards.
        public Collider sourceCollider
        {
            set
            {
                if (value != null && value.gameObject != this.gameObject)
                {
                    Debug.LogError("The Collider component must reside in the same GameObject as ObiCollider.");
                    return;
                }

                m_SourceCollider = value;

                RemoveCollider();
                AddCollider();

            }
            get { return m_SourceCollider; }
        }

        [SerializeProperty("distanceField")]
        [FormerlySerializedAs("distanceField")]
        [SerializeField] private ObiDistanceField m_DistanceField;

        /// <summary>
        /// The distance field used by this collider.
        /// </summary>
        /// Setting a distance field will cause the collider to ignore its <see cref="m_SourceCollider"/> and use the distance field instead.
        public ObiDistanceField distanceField
        {
            set
            {
                if (m_DistanceField != value)
                {
                    m_DistanceField = value;
                    CreateTracker();
                }
            }
            get { return m_DistanceField; }
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

            if (distanceField != null)
                tracker = new ObiDistanceFieldShapeTracker(this, m_SourceCollider, distanceField);
            else
            {

                if (m_SourceCollider is SphereCollider)
                    tracker = new ObiSphereShapeTracker(this, (SphereCollider)m_SourceCollider);
                else if (m_SourceCollider is BoxCollider)
                    tracker = new ObiBoxShapeTracker(this, (BoxCollider)m_SourceCollider);
                else if (m_SourceCollider is CapsuleCollider)
                    tracker = new ObiCapsuleShapeTracker(this, (CapsuleCollider)m_SourceCollider);
                else if (m_SourceCollider is CharacterController)
                    tracker = new ObiCharacterControllerShapeTracker(this, (CharacterController)m_SourceCollider);
                else if (m_SourceCollider is TerrainCollider)
                    tracker = new ObiTerrainShapeTracker(this, (TerrainCollider)m_SourceCollider);
                else if (m_SourceCollider is MeshCollider)
                    tracker = new ObiMeshShapeTracker(this,(MeshCollider)m_SourceCollider);
                else
                    Debug.LogWarning("Collider type not supported by Obi.");

            }

        }

        protected override Component GetUnityCollider(ref bool enabled)
        {

            if (m_SourceCollider != null)
                enabled = m_SourceCollider.enabled;

            return m_SourceCollider;
        }

        protected override void FindSourceCollider()
        {
            if (sourceCollider == null)
                sourceCollider = GetComponent<Collider>();
            else
                AddCollider();
        }

    }
}

