using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    [AddComponentMenu("Physics/Obi/Obi Rod", 881)]
    [ExecuteInEditMode]
    [DisallowMultipleComponent]
    public class ObiRod : ObiRopeBase, IStretchShearConstraintsUser, IBendTwistConstraintsUser, IChainConstraintsUser
    {
        [SerializeField] protected ObiRodBlueprint m_RodBlueprint;

        // distance constraints:
        [SerializeField] protected bool _stretchShearConstraintsEnabled = true;
        [SerializeField] protected float _stretchCompliance = 0;
        [SerializeField] protected float _shear1Compliance = 0;
        [SerializeField] protected float _shear2Compliance = 0;

        // bend constraints:
        [SerializeField] protected bool _bendTwistConstraintsEnabled = true;
        [SerializeField] protected float _torsionCompliance = 0;
        [SerializeField] protected float _bend1Compliance = 0;
        [SerializeField] protected float _bend2Compliance = 0;

        // chain constraints:
        [SerializeField] protected bool _chainConstraintsEnabled = true;
        [SerializeField] [Range(0, 1)] protected float _tightness = 1;

        /// <summary>  
        /// Whether particles in this actor colide with particles using the same phase value.
        /// </summary>
        public bool selfCollisions
        {
            get { return m_SelfCollisions; }
            set { if (value != m_SelfCollisions) { m_SelfCollisions = value; SetSelfCollisions(m_SelfCollisions); } }
        }

        /// <summary>  
        /// Whether this actor's stretch/shear constraints are enabled.
        /// </summary>
        public bool stretchShearConstraintsEnabled
        {
            get { return _stretchShearConstraintsEnabled; }
            set { if (value != _stretchShearConstraintsEnabled) { _stretchShearConstraintsEnabled = value; SetConstraintsDirty(Oni.ConstraintType.StretchShear); } }
        }

        /// <summary>  
        /// Compliance of this actor's stretch/shear constraints, along their length.
        /// </summary>
        public float stretchCompliance
        {
            get { return _stretchCompliance; }
            set { _stretchCompliance = value; SetConstraintsDirty(Oni.ConstraintType.StretchShear); }
        }

        /// <summary>  
        /// Shearing compliance of this actor's stretch/shear constraints, along the first axis orthogonal to their length.
        /// </summary>
        public float shear1Compliance
        {
            get { return _shear1Compliance; }
            set { _shear1Compliance = value; SetConstraintsDirty(Oni.ConstraintType.StretchShear); }
        }

        /// <summary>  
        /// Shearing compliance of this actor's stretch/shear constraints, along the second axis orthogonal to their length.
        /// </summary>
        public float shear2Compliance
        {
            get { return _shear2Compliance; }
            set { _shear2Compliance = value; SetConstraintsDirty(Oni.ConstraintType.StretchShear); }
        }

        /// <summary>  
        /// Whether this actor's bend/twist constraints are enabled.
        /// </summary>
        public bool bendTwistConstraintsEnabled
        {
            get { return _bendTwistConstraintsEnabled; }
            set { if (value != _bendTwistConstraintsEnabled) { _bendTwistConstraintsEnabled = value; SetConstraintsDirty(Oni.ConstraintType.BendTwist); } }
        }

        /// <summary>  
        /// Torsional compliance of this actor's bend/twist constraints along their length.
        /// </summary>
        public float torsionCompliance
        {
            get { return _torsionCompliance; }
            set { _torsionCompliance = value; SetConstraintsDirty(Oni.ConstraintType.BendTwist); }
        }

        /// <summary>  
        /// Bending compliance of this actor's bend/twist constraints along the first axis orthogonal to their length.
        /// </summary>
        public float bend1Compliance
        {
            get { return _bend1Compliance; }
            set { _bend1Compliance = value; SetConstraintsDirty(Oni.ConstraintType.BendTwist); }
        }

        /// <summary>  
        /// Bending compliance of this actor's bend/twist constraints along the second axis orthogonal to their length.
        /// </summary>
        public float bend2Compliance
        {
            get { return _bend2Compliance; }
            set { _bend2Compliance = value; SetConstraintsDirty(Oni.ConstraintType.BendTwist); }
        }

        /// <summary>  
        /// Whether this actor's chain constraints are enabled.
        /// </summary>
        public bool chainConstraintsEnabled
        {
            get { return _chainConstraintsEnabled; }
            set { if (value != _chainConstraintsEnabled) { _chainConstraintsEnabled = value; SetConstraintsDirty(Oni.ConstraintType.BendTwist); } }
        }

        /// <summary>  
        /// Tightness of this actor's chain constraints.
        /// </summary>
        /// Controls how much chain constraints are allowed to compress.
        public float tightness
        {
            get { return _tightness; }
            set { _tightness = value; SetConstraintsDirty(Oni.ConstraintType.Chain); }
        }

        /// <summary>  
        /// Average distance between consecutive particle centers in this rod.
        /// </summary>
        public float interParticleDistance
        {
            get { return m_RodBlueprint.interParticleDistance; }
        }

        public override ObiActorBlueprint sourceBlueprint
        {
            get { return m_RodBlueprint; }
        }

        public ObiRodBlueprint rodBlueprint
        {
            get { return m_RodBlueprint; }
            set
            {
                if (m_RodBlueprint != value)
                {
                    RemoveFromSolver();
                    ClearState();
                    m_RodBlueprint = value;
                    AddToSolver();
                }
            }
        }

        protected override void OnValidate()
        {
            base.OnValidate();
            SetupRuntimeConstraints();
        }

        public override void LoadBlueprint(ObiSolver solver)
        {
            base.LoadBlueprint(solver);
            RebuildElementsFromConstraints();
            SetupRuntimeConstraints();
        }

        private void SetupRuntimeConstraints()
        {
            SetConstraintsDirty(Oni.ConstraintType.StretchShear);
            SetConstraintsDirty(Oni.ConstraintType.BendTwist);
            SetConstraintsDirty(Oni.ConstraintType.Chain);
            SetSelfCollisions(selfCollisions);
            RecalculateRestLength();
        }

        protected override void RebuildElementsFromConstraintsInternal()
        {
            var dc = GetConstraintsByType(Oni.ConstraintType.StretchShear) as ObiConstraints<ObiStretchShearConstraintsBatch>;
            if (dc == null)
                return;

            int constraintCount = dc.batches[0].activeConstraintCount + dc.batches[1].activeConstraintCount;

            elements = new List<ObiStructuralElement>(constraintCount);

            for (int i = 0; i < constraintCount; ++i)
            {
                var batch = dc.batches[i % 2] as ObiStretchShearConstraintsBatch;
                int constraintIndex = i / 2;

                var e = new ObiStructuralElement();
                e.particle1 = batch.particleIndices[constraintIndex * 2];
                e.particle2 = batch.particleIndices[constraintIndex * 2 + 1];
                e.restLength = batch.restLengths[constraintIndex];
                elements.Add(e);
            }

            if (dc.batches.Count > 2)
            {
                var batch = dc.batches[2];
                var e = new ObiStructuralElement();
                e.particle1 = batch.particleIndices[0];
                e.particle2 = batch.particleIndices[1];
                e.restLength = batch.restLengths[0];
                elements.Add(e);
            }
        }

    }
}
