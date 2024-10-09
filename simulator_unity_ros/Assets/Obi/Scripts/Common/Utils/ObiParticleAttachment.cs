using System;
using UnityEngine;

namespace Obi
{
    [AddComponentMenu("Physics/Obi/Obi Particle Attachment", 820)]
    [RequireComponent(typeof(ObiActor))]
    public class ObiParticleAttachment : MonoBehaviour
    {
        public enum AttachmentType
        {
            Static,
            Dynamic
        }

        [SerializeField] [HideInInspector] private ObiActor m_Actor;

        [SerializeField] [HideInInspector] private Transform m_Target;
        [SerializeField] [HideInInspector] private ObiParticleGroup m_ParticleGroup;
        [SerializeField] [HideInInspector] private AttachmentType m_AttachmentType = AttachmentType.Static;
        [SerializeField] [HideInInspector] private bool m_ConstrainOrientation = false;
        [SerializeField] [HideInInspector] private float m_Compliance = 0;
        [SerializeField] [HideInInspector] [Delayed] private float m_BreakThreshold = float.PositiveInfinity;

        private ObiPinConstraintsBatch pinBatch;
        private int pinBatchIndex = -1;

        // private variables are serialized during script reloading, to keep their value. Must mark them explicitly as non-serialized.
        [NonSerialized] private int[] m_SolverIndices;
        [NonSerialized] private Vector3[] m_PositionOffsets = null;
        [NonSerialized] private Quaternion[] m_OrientationOffsets = null;

        /// <summary>  
        /// The actor this attachment is added to.
        /// </summary> 
        public ObiActor actor
        {
            get { return m_Actor; }
        }

        /// <summary>  
        /// The target transform that the <see cref="particleGroup"/> should be attached to.
        /// </summary> 
        public Transform target
        {
            get { return m_Target; }
            set
            {
                if (value != m_Target)
                {
                    m_Target = value;
                    Bind();
                }
            }
        }

        /// <summary>  
        /// The particle group that should be attached to the <see cref="target"/>.
        /// </summary> 
        public ObiParticleGroup particleGroup
        {
            get
            {
                return m_ParticleGroup;
            }

            set
            {
                if (value != m_ParticleGroup)
                {
                    m_ParticleGroup = value;
                    Bind();
                }
            }
        }

        /// <summary>  
        /// Whether this attachment is currently bound or not.
        /// </summary> 
        public bool isBound
        {
            get { return m_Target != null && m_SolverIndices != null && m_PositionOffsets != null; }
        }

        /// <summary>  
        /// Type of attachment, can be either static or dynamic.
        /// </summary> 
        public AttachmentType attachmentType
        {
            get { return m_AttachmentType; }
            set
            {
                if (value != m_AttachmentType)
                {
                    Disable(m_AttachmentType);
                    m_AttachmentType = value;
                    Enable(m_AttachmentType);
                }
            }
        }

        /// <summary>  
        /// Should this attachment constraint particle orientations too?
        /// </summary>
        public bool constrainOrientation
        {
            get { return m_ConstrainOrientation; }
            set
            {
                if (value != m_ConstrainOrientation)
                {
                    Disable(m_AttachmentType);
                    m_ConstrainOrientation = value;
                    Enable(m_AttachmentType);
                }
            }
        }


        /// <summary>  
        /// Constraint compliance, in case this attachment is dynamic.
        /// </summary>
        /// High compliance values will increase the attachment's elasticity.
        public float compliance
        {
            get { return m_Compliance; }
            set
            {
                if (!Mathf.Approximately(value, m_Compliance))
                {
                    m_Compliance = value;
                    if (m_AttachmentType == AttachmentType.Dynamic && pinBatch != null)
                    {
                        for (int i = 0; i < m_SolverIndices.Length; ++i)
                            pinBatch.stiffnesses[i * 2] = m_Compliance;
                    }
                }
            }
        }

        /// <summary>  
        /// Force thershold above which the attachment should break.
        /// </summary>
        /// Only affects dynamic attachments, as static attachments do not work with forces.
        public float breakThreshold
        {
            get { return m_BreakThreshold; }
            set
            {
                if (!Mathf.Approximately(value, m_BreakThreshold))
                {
                    m_BreakThreshold = value;
                    if (m_AttachmentType == AttachmentType.Dynamic && pinBatch != null)
                    {
                        for (int i = 0; i < m_SolverIndices.Length; ++i)
                            pinBatch.breakThresholds[i] = m_BreakThreshold;
                    }
                }
            }
        }

        private void Awake()
        {
            m_Actor = GetComponent<ObiActor>();
            m_Actor.OnBlueprintLoaded += Actor_OnBlueprintLoaded;
            m_Actor.OnBeginStep += Actor_OnBeginStep;
            m_Actor.OnEndStep += Actor_OnEndStep;

            if (m_Actor.solver != null)
                Actor_OnBlueprintLoaded(m_Actor, m_Actor.sourceBlueprint);
        }

        private void OnDestroy()
        {
            m_Actor.OnBlueprintLoaded -= Actor_OnBlueprintLoaded;
            m_Actor.OnBeginStep -= Actor_OnBeginStep;
            m_Actor.OnEndStep -= Actor_OnEndStep;
        }

        private void OnEnable()
        {
            Enable(m_AttachmentType);
        }

        private void OnDisable()
        {
            Disable(m_AttachmentType);
        }

        private void OnValidate()
        {
            m_Actor = GetComponent<ObiActor>();

            // do not re-bind: simply disable and re-enable the attachment.
            Disable(AttachmentType.Static);
            Disable(AttachmentType.Dynamic);
            Enable(m_AttachmentType);
        }

        void Actor_OnBlueprintLoaded(ObiActor act, ObiActorBlueprint blueprint)
        {
            Bind();
        }

        void Actor_OnBeginStep(ObiActor act, float stepTime)
        {
            // static attachments must be updated at the start of the step, before performing any simulation.
            UpdateStaticAttachment(stepTime);
        }

        private void Actor_OnEndStep(ObiActor actor, float stepTime)
        {
            // dynamic attachments must be tested at the end of the step, once constraint forces have been calculated.
            // if there's any broken constraint, flag pin constraints as dirty for remerging at the start of the next step.
            UpdateDynamicAttachment(stepTime);
        }

        private void Bind()
        {
            // Disable attachment.
            Disable(m_AttachmentType);

            if (m_ParticleGroup != null && m_Actor.solver != null)
            {
                Matrix4x4 bindMatrix = m_Target != null ? m_Target.worldToLocalMatrix * m_Actor.solver.transform.localToWorldMatrix : Matrix4x4.identity;

                m_SolverIndices = new int[m_ParticleGroup.Count];
                m_PositionOffsets = new Vector3[m_ParticleGroup.Count];
                m_OrientationOffsets = new Quaternion[m_ParticleGroup.Count];

                for (int i = 0; i < m_ParticleGroup.Count; ++i)
                {
                    int particleIndex = m_ParticleGroup.particleIndices[i];
                    if (particleIndex >= 0 && particleIndex < m_Actor.solverIndices.Length)
                    {
                        m_SolverIndices[i] = m_Actor.solverIndices[particleIndex];
                        m_PositionOffsets[i] = bindMatrix.MultiplyPoint3x4(m_Actor.solver.positions[m_SolverIndices[i]]);
                    }
                    else
                    {
                        Debug.LogError("The particle group \'" + m_ParticleGroup.name + "\' references a particle that does not exist in the actor \'" + m_Actor.name + "\'.");
                        m_SolverIndices = null;
                        m_PositionOffsets = null;
                        m_OrientationOffsets = null;
                        return;
                    }
                }

                if (m_Actor.usesOrientedParticles)
                {
                    Quaternion bindOrientation = bindMatrix.rotation;

                    for (int i = 0; i < m_ParticleGroup.Count; ++i)
                    {
                        int particleIndex = m_ParticleGroup.particleIndices[i];
                        if (particleIndex >= 0 && particleIndex < m_Actor.solverIndices.Length)
                            m_OrientationOffsets[i] = bindOrientation * m_Actor.solver.orientations[m_SolverIndices[i]];
                    }
                }
            }
            else
            {
                m_PositionOffsets = null;
                m_OrientationOffsets = null;
            }

            Enable(m_AttachmentType);
        }


        private void Enable(AttachmentType type)
        { 

            if (enabled && m_Actor.isLoaded && isBound)
            {

                var solver = m_Actor.solver;
                var blueprint = m_Actor.sourceBlueprint;

                switch (type)
                {
                    case AttachmentType.Dynamic:

                        var pins = m_Actor.GetConstraintsByType(Oni.ConstraintType.Pin) as ObiPinConstraintsData;
                        ObiColliderBase attachedCollider = m_Target.GetComponent<ObiColliderBase>();

                        if (pins != null && attachedCollider != null && pinBatch == null)
                        {
                            // create a new data batch with all our pin constraints:
                            pinBatch = new ObiPinConstraintsBatch(pins);
                            for (int i = 0; i < m_SolverIndices.Length; ++i)
                            {
                                pinBatch.AddConstraint(m_SolverIndices[i],
                                                       attachedCollider,
                                                       m_PositionOffsets[i],
                                                       m_OrientationOffsets[i],
                                                       m_Compliance,
                                                       constrainOrientation ? 0 : 10000,
                                                       m_BreakThreshold);

                                pinBatch.activeConstraintCount++;
                            }

                            // add the batch to the solver, and store its index for later use.
                            pinBatchIndex = pins.GetBatchCount();
                            pins.AddBatch(pinBatch);

                            m_Actor.SetConstraintsDirty(Oni.ConstraintType.Pin);
                        }

                        break;

                    case AttachmentType.Static:

                        for (int i = 0; i < m_SolverIndices.Length; ++i)
                            if (m_SolverIndices[i] >= 0 && m_SolverIndices[i] < solver.invMasses.count)
                                solver.invMasses[m_SolverIndices[i]] = 0;

                        if (m_Actor.usesOrientedParticles && m_ConstrainOrientation)
                        {
                            for (int i = 0; i < m_SolverIndices.Length; ++i)
                                if (m_SolverIndices[i] >= 0 && m_SolverIndices[i] < solver.invRotationalMasses.count)
                                    solver.invRotationalMasses[m_SolverIndices[i]] = 0;
                        }

                        m_Actor.UpdateParticleProperties();

                        break;

                }
            }

        }

        private void Disable(AttachmentType type)
        {
            if (actor.isLoaded && isBound)
            {
                var solver = m_Actor.solver;
                var blueprint = m_Actor.sourceBlueprint;

                switch (type)
                {
                    case AttachmentType.Dynamic:

                        var pins = m_Actor.GetConstraintsByType(Oni.ConstraintType.Pin) as ObiConstraints<ObiPinConstraintsBatch>;
                        if (pins != null && pinBatch != null)
                        {
                            pins.RemoveBatch(pinBatch);
                            pinBatch = null;
                            pinBatchIndex = -1;
                            m_Actor.SetConstraintsDirty(Oni.ConstraintType.Pin);
                        }

                        break;

                    case AttachmentType.Static:

                        for (int i = 0; i < m_SolverIndices.Length; ++i)
                            if (m_SolverIndices[i] >= 0 && m_SolverIndices[i] < solver.invMasses.count)
                                solver.invMasses[m_SolverIndices[i]] = blueprint.invMasses[i];

                        if (m_Actor.usesOrientedParticles)
                        {
                            for (int i = 0; i < m_SolverIndices.Length; ++i)
                                if (m_SolverIndices[i] >= 0 && m_SolverIndices[i] < solver.invRotationalMasses.count)
                                    solver.invRotationalMasses[m_SolverIndices[i]] = blueprint.invRotationalMasses[i];
                        }

                        m_Actor.UpdateParticleProperties();

                        break;

                }
            }
        }

        private void UpdateStaticAttachment(float stepTime)
        {

            if (enabled && m_AttachmentType == AttachmentType.Static && m_Actor.isLoaded && isBound)
            {
                var solver = m_Actor.solver;
                var blueprint = m_Actor.sourceBlueprint;

                // Build the attachment matrix:
                Matrix4x4 attachmentMatrix = solver.transform.worldToLocalMatrix * m_Target.localToWorldMatrix;

                // Fix all particles in the group and update their position:
                for (int i = 0; i < m_SolverIndices.Length; ++i)
                {
                    int solverIndex = m_SolverIndices[i];

                    if (solverIndex >= 0 && solverIndex < solver.invMasses.count)
                    {
                        solver.invMasses[solverIndex] = 0;
                        solver.velocities[solverIndex] = Vector3.zero;

                        // Note: skip assignment to startPositions if you want attached particles to be interpolated too.
                        solver.startPositions[solverIndex] = solver.positions[solverIndex] = attachmentMatrix.MultiplyPoint3x4(m_PositionOffsets[i]);
                    }
                }

                if (m_Actor.usesOrientedParticles && m_ConstrainOrientation)
                {
                    Quaternion attachmentRotation = attachmentMatrix.rotation;

                    for (int i = 0; i < m_SolverIndices.Length; ++i)
                    {
                        int solverIndex = m_SolverIndices[i];

                        if (solverIndex >= 0 && solverIndex < solver.invRotationalMasses.count)
                        {
                            solver.invRotationalMasses[solverIndex] = 0;
                            solver.angularVelocities[solverIndex] = Vector3.zero;

                            // Note: skip assignment to startPositions if you want attached particles to be interpolated too.
                            solver.startOrientations[solverIndex] = solver.orientations[solverIndex] = attachmentRotation * m_OrientationOffsets[i];
                        }
                    }
                }
            }
        }

        private void UpdateDynamicAttachment(float stepTime)
        {

            if (enabled && m_AttachmentType == AttachmentType.Dynamic && m_Actor.isLoaded && isBound)
            {

                var solver = m_Actor.solver;
                var blueprint = m_Actor.sourceBlueprint;

                var actorConstraints = m_Actor.GetConstraintsByType(Oni.ConstraintType.Pin) as ObiConstraints<ObiPinConstraintsBatch>;
                var solverConstraints = solver.GetConstraintsByType(Oni.ConstraintType.Pin) as ObiConstraints<ObiPinConstraintsBatch>;

                bool torn = false;
                if (actorConstraints != null && pinBatch != null)
                {
                    // deactivate constraints over the break threshold.
                    int offset = actor.solverBatchOffsets[(int)Oni.ConstraintType.Pin][pinBatchIndex];
                    var solverBatch = solverConstraints.batches[pinBatchIndex];

                    float sqrTime = stepTime * stepTime;
                    for (int i = 0; i < pinBatch.activeConstraintCount; i++)
                    {
                        if (-solverBatch.lambdas[(offset + i) * 4 + 3] / sqrTime > pinBatch.breakThresholds[i])
                        {
                            pinBatch.DeactivateConstraint(i);
                            torn = true;
                        }
                    }
                }

                // constraints are recreated at the start of a step.
                if (torn)
                    m_Actor.SetConstraintsDirty(Oni.ConstraintType.Pin);
            }
        }
    }
}
