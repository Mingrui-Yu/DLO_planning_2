using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    [AddComponentMenu("Physics/Obi/Obi Rope", 880)]
    [ExecuteInEditMode]
    [DisallowMultipleComponent]
    public class ObiRope : ObiRopeBase, IDistanceConstraintsUser, IBendConstraintsUser
    {
        [SerializeField] protected ObiRopeBlueprint m_RopeBlueprint;
        private ObiRopeBlueprint m_RopeBlueprintInstance;

        // rope has a list of structural elements.
        // each structural element is equivalent to 1 distance constraint and 2 bend constraints (with previous, and following element).
        // a structural element has force and rest length.
        // a function re-generates constraints from structural elements when needed, placing them in the appropiate batches.

        public bool tearingEnabled = false;
        public float tearResistanceMultiplier = 1000;                   /**< Factor that controls how much a structural cloth spring can stretch before breaking.*/
        public int tearRate = 1;

        // distance constraints:
        [SerializeField] protected bool _distanceConstraintsEnabled = true;
        [SerializeField] protected float _stretchingScale = 1;
        [SerializeField] protected float _stretchCompliance = 0;
        [SerializeField] [Range(0, 1)] protected float _maxCompression = 0;

        // bend constraints:
        [SerializeField] protected bool _bendConstraintsEnabled = true;
        [SerializeField] protected float _bendCompliance = 0;
        [SerializeField] [Range(0, 0.5f)] protected float _maxBending = 0;

        /// <summary>  
        /// Whether particles in this actor colide with particles using the same phase value.
        /// </summary>
        public bool selfCollisions
        {
            get { return m_SelfCollisions; }
            set { if (value != m_SelfCollisions) { m_SelfCollisions = value; SetSelfCollisions(selfCollisions); } }
        }

        /// <summary>  
        /// Whether this actor's distance constraints are enabled.
        /// </summary>
        public bool distanceConstraintsEnabled
        {
            get { return _distanceConstraintsEnabled; }
            set { if (value != _distanceConstraintsEnabled) { _distanceConstraintsEnabled = value; SetConstraintsDirty(Oni.ConstraintType.Distance); } }
        }

        /// <summary>  
        /// Scale value for this actor's distance constraints rest length.
        /// </summary>
        /// The default is 1. For instamce, a value of 2 will make the distance constraints twice as long, 0.5 will reduce their length in half.
        public float stretchingScale
        {
            get { return _stretchingScale; }
            set { _stretchingScale = value; SetConstraintsDirty(Oni.ConstraintType.Distance); }
        }

        /// <summary>  
        /// Compliance of this actor's stretch constraints.
        /// </summary>
        public float stretchCompliance
        {
            get { return _stretchCompliance; }
            set { _stretchCompliance = value; SetConstraintsDirty(Oni.ConstraintType.Distance); }
        }

        /// <summary>  
        /// Maximum compression this actor's distance constraints can undergo.
        /// </summary>
        /// This is expressed as a percentage of the scaled rest length. 
        public float maxCompression
        {
            get { return _maxCompression; }
            set { _maxCompression = value; SetConstraintsDirty(Oni.ConstraintType.Distance); }
        }

        /// <summary>  
        ///  Whether this actor's bend constraints are enabled.
        /// </summary>
        public bool bendConstraintsEnabled
        {
            get { return _bendConstraintsEnabled; }
            set { if (value != _bendConstraintsEnabled) { _bendConstraintsEnabled = value; SetConstraintsDirty(Oni.ConstraintType.Bending); } }
        }

        /// <summary>  
        ///  Compliance of this actor's bend constraints.
        /// </summary>
        public float bendCompliance
        {
            get { return _bendCompliance; }
            set { _bendCompliance = value; SetConstraintsDirty(Oni.ConstraintType.Bending); }
        }

        /// <summary>  
        ///  Max bending value that constraints can undergo before resisting bending.
        /// </summary>
        public float maxBending
        {
            get { return _maxBending; }
            set { _maxBending = value; SetConstraintsDirty(Oni.ConstraintType.Bending); }
        }

        /// <summary>  
        /// Average distance between consecutive particle centers in this rope.
        /// </summary>
        public float interParticleDistance
        {
            get { return m_RopeBlueprint.interParticleDistance; }
        }

        public override ObiActorBlueprint sourceBlueprint
        {
            get { return m_RopeBlueprint; }
        }

        public ObiRopeBlueprint ropeBlueprint
        {
            get { return m_RopeBlueprint; }
            set
            {
                if (m_RopeBlueprint != value)
                {
                    RemoveFromSolver();
                    ClearState();
                    m_RopeBlueprint = value;
                    AddToSolver();
                }
            }
        }

        public delegate void RopeTornCallback(ObiRope rope, ObiRopeTornEventArgs tearInfo);
        public event RopeTornCallback OnRopeTorn;  /**< Called when a constraint is torn.*/

        public class ObiRopeTornEventArgs
        {
            public ObiStructuralElement element;    /**< info about the element being torn.*/
            public int particleIndex;               /**< index of the particle being torn*/

            public ObiRopeTornEventArgs(ObiStructuralElement element, int particle)
            {
                this.element = element;
                this.particleIndex = particle;
            }
        }

        protected override void OnValidate()
        {
            base.OnValidate();
            SetupRuntimeConstraints();
        }

        public override void LoadBlueprint(ObiSolver solver)
        {
            // create a copy of the blueprint for this cloth:
            if (Application.isPlaying)
                m_RopeBlueprintInstance = this.blueprint as ObiRopeBlueprint;

            base.LoadBlueprint(solver);
            RebuildElementsFromConstraints();
            SetupRuntimeConstraints();
        }

        public override void UnloadBlueprint(ObiSolver solver)
        {
            base.UnloadBlueprint(solver);

            // delete the blueprint instance:
            if (m_RopeBlueprintInstance != null)
                DestroyImmediate(m_RopeBlueprintInstance);
        }

        private void SetupRuntimeConstraints()
        {
            SetConstraintsDirty(Oni.ConstraintType.Distance);
            SetConstraintsDirty(Oni.ConstraintType.Bending);
            SetSelfCollisions(selfCollisions);
            RecalculateRestLength();
        }

        public override void Substep(float substepTime)
        {
            base.Substep(substepTime);

            if (isActiveAndEnabled)
                ApplyTearing(substepTime);
        }

        protected void ApplyTearing(float substepTime)
        {

            if (!tearingEnabled)
                return;

            float sqrTime = substepTime * substepTime;

            List<ObiStructuralElement> tornElements = new List<ObiStructuralElement>();

            var dc = GetConstraintsByType(Oni.ConstraintType.Distance) as ObiConstraints<ObiDistanceConstraintsBatch>;
            var sc = this.solver.GetConstraintsByType(Oni.ConstraintType.Distance) as ObiConstraints<ObiDistanceConstraintsBatch>;

            if (dc != null && sc != null)
            for (int j = 0; j < dc.GetBatchCount(); ++j)
            {
                var batch = dc.GetBatch(j) as ObiDistanceConstraintsBatch;
                var solverBatch = sc.batches[j] as ObiDistanceConstraintsBatch;

                for (int i = 0; i < batch.activeConstraintCount; i++)
                {
                    int elementIndex = j + 2 * i;

                    // divide lambda by squared delta time to get force in newtons:
                    int offset = solverBatchOffsets[(int)Oni.ConstraintType.Distance][j];
                    float force = solverBatch.lambdas[offset + i] / sqrTime;

                    elements[elementIndex].constraintForce = force;

                    if (-force > tearResistanceMultiplier)
                    {
                        tornElements.Add(elements[elementIndex]);
                    }
                }
            }

            if (tornElements.Count > 0)
            {

                // sort edges by force:
                tornElements.Sort(delegate (ObiStructuralElement x, ObiStructuralElement y)
                {
                    return x.constraintForce.CompareTo(y.constraintForce);
                });

                int tornCount = 0;
                for (int i = 0; i < tornElements.Count; i++)
                {
                    if (Tear(tornElements[i]))
                        tornCount++;
                    if (tornCount >= tearRate)
                        break;
                }

                if (tornCount > 0)
                    RebuildConstraintsFromElements();
            }

        }

        private int SplitParticle(int splitIndex)
        {
            // halve the original particle's mass:
            m_Solver.invMasses[splitIndex] *= 2;

            CopyParticle(solver.particleToActor[splitIndex].indexInActor, activeParticleCount);
            ActivateParticle(activeParticleCount);

            return solverIndices[activeParticleCount - 1];
        }

        public bool Tear(ObiStructuralElement element)
        {
            // don't allow splitting if there are no free particles left in the pool.
            if (activeParticleCount >= m_RopeBlueprint.particleCount)
                return false;

            // Cannot split fixed particles:
            if (m_Solver.invMasses[element.particle1] == 0)
                return false;

            // Or particles that have been already split.
            int index = elements.IndexOf(element);
            if (index > 0 && elements[index - 1].particle2 != element.particle1)
                return false;

            element.particle1 = SplitParticle(element.particle1);

            if (OnRopeTorn != null)
                OnRopeTorn(this, new ObiRopeTornEventArgs(element, element.particle1));

            return true;
        }

        protected override void RebuildElementsFromConstraintsInternal()
        {
            var dc = GetConstraintsByType(Oni.ConstraintType.Distance) as ObiConstraints<ObiDistanceConstraintsBatch>;
            if (dc == null || dc.GetBatchCount() < 2)
                return;

            int constraintCount = dc.batches[0].activeConstraintCount + dc.batches[1].activeConstraintCount;

            elements = new List<ObiStructuralElement>(constraintCount);

            for (int i = 0; i < constraintCount; ++i)
            {
                var batch = dc.batches[i % 2] as ObiDistanceConstraintsBatch;
                int constraintIndex = i / 2;

                var e = new ObiStructuralElement();
                e.particle1 = solverIndices[batch.particleIndices[constraintIndex * 2]];
                e.particle2 = solverIndices[batch.particleIndices[constraintIndex * 2 + 1]];
                e.restLength = batch.restLengths[constraintIndex];
                e.tearResistance = 1;
                elements.Add(e);
            }

            // loop-closing element:
            if (dc.batches.Count > 2)
            {
                var batch = dc.batches[2];
                var e = new ObiStructuralElement();
                e.particle1 = solverIndices[batch.particleIndices[0]];
                e.particle2 = solverIndices[batch.particleIndices[1]];
                e.restLength = batch.restLengths[0];
                e.tearResistance = 1;
                elements.Add(e);
            }
        }

        public override void RebuildConstraintsFromElements()
        {
            // regenerate constraints from elements:
            var dc = GetConstraintsByType(Oni.ConstraintType.Distance) as ObiConstraints<ObiDistanceConstraintsBatch>;
            var bc = GetConstraintsByType(Oni.ConstraintType.Bending) as ObiConstraints<ObiBendConstraintsBatch>;

            dc.DeactivateAllConstraints();
            bc.DeactivateAllConstraints();

            int elementsCount = elements.Count - (ropeBlueprint.path.Closed ? 1 : 0);
            for (int i = 0; i < elementsCount; ++i)
            {
                var db = dc.batches[i % 2] as ObiDistanceConstraintsBatch;
                int constraint = db.activeConstraintCount;

                db.particleIndices[constraint * 2] = solver.particleToActor[elements[i].particle1].indexInActor;
                db.particleIndices[constraint * 2 + 1] = solver.particleToActor[elements[i].particle2].indexInActor;
                db.restLengths[constraint] = elements[i].restLength;
                db.stiffnesses[constraint] = new Vector2(_stretchCompliance, _maxCompression * db.restLengths[constraint]);
                db.ActivateConstraint(constraint);

                if (i < elementsCount - 1)
                {
                    var bb = bc.batches[i % 3] as ObiBendConstraintsBatch;

                    // create bend constraint only if there's continuity between elements:
                    if (elements[i].particle2 == elements[i + 1].particle1)
                    {
                        constraint = bb.activeConstraintCount;

                        int indexA = elements[i].particle1;
                        int indexB = elements[i + 1].particle2;
                        int indexC = elements[i].particle2;
                        float restBend = ObiUtils.RestBendingConstraint(solver.restPositions[indexA], solver.restPositions[indexB], solver.restPositions[indexC]);

                        bb.particleIndices[constraint * 3] = solver.particleToActor[indexA].indexInActor;
                        bb.particleIndices[constraint * 3 + 1] = solver.particleToActor[indexB].indexInActor;
                        bb.particleIndices[constraint * 3 + 2] = solver.particleToActor[indexC].indexInActor;
                        bb.restBends[constraint] = restBend;
                        bb.bendingStiffnesses[constraint] = new Vector2(_maxBending, _bendCompliance);
                        bb.ActivateConstraint(constraint);
                    }
                }
            }

            // loop-closing constraints:
            if (dc.batches.Count > 2)
            {
                var loopClosingBatch = dc.batches[2];
                var lastElement = elements[elements.Count - 1];
                loopClosingBatch.particleIndices[0] = solver.particleToActor[lastElement.particle1].indexInActor;
                loopClosingBatch.particleIndices[1] = solver.particleToActor[lastElement.particle2].indexInActor;
                loopClosingBatch.restLengths[0] = lastElement.restLength;
                loopClosingBatch.stiffnesses[0] = new Vector2(_stretchCompliance, _maxCompression * loopClosingBatch.restLengths[0]);
                loopClosingBatch.ActivateConstraint(0);
            }

            if (bc.batches.Count > 4 && elements.Count > 2)
            {
                var loopClosingBatch = bc.batches[3];
                var lastElement = elements[elements.Count - 2];

                // for loop constraints, 0 is our best approximation of rest bend:
                loopClosingBatch.particleIndices[0] = solver.particleToActor[lastElement.particle1].indexInActor;
                loopClosingBatch.particleIndices[1] = solver.particleToActor[elements[0].particle1].indexInActor;
                loopClosingBatch.particleIndices[2] = solver.particleToActor[lastElement.particle2].indexInActor;
                loopClosingBatch.restBends[0] = 0;
                loopClosingBatch.bendingStiffnesses[0] = new Vector2(_maxBending, _bendCompliance);
                loopClosingBatch.ActivateConstraint(0);

                loopClosingBatch = bc.batches[4];
                loopClosingBatch.particleIndices[0] = solver.particleToActor[lastElement.particle2].indexInActor;
                loopClosingBatch.particleIndices[1] = solver.particleToActor[elements[0].particle2].indexInActor;
                loopClosingBatch.particleIndices[2] = solver.particleToActor[elements[0].particle1].indexInActor;
                loopClosingBatch.restBends[0] = 0;
                loopClosingBatch.bendingStiffnesses[0] = new Vector2(_maxBending, _bendCompliance);
                loopClosingBatch.ActivateConstraint(0);
            }

            SetConstraintsDirty(Oni.ConstraintType.Distance);
            SetConstraintsDirty(Oni.ConstraintType.Bending);
        }
    }
}
