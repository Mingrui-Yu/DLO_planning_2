using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;


namespace Obi
{

    /**
     * Represents a group of related particles. ObiActor does not make
     * any assumptions about the relationship between these particles, except that they get allocated 
     * and released together.
     */
    [ExecuteInEditMode]
    [DisallowMultipleComponent]
    public abstract class ObiActor : MonoBehaviour, IObiParticleCollection
    {
        public class ObiActorSolverArgs : System.EventArgs
        {
            public ObiSolver solver { get; }

            public ObiActorSolverArgs(ObiSolver solver)
            {
                this.solver = solver;
            }
        }

        public delegate void ActorCallback(ObiActor actor);                     
        public delegate void ActorStepCallback(ObiActor actor,float stepTime);
        public delegate void ActorBlueprintCallback(ObiActor actor,ObiActorBlueprint blueprint);

        /// <summary>
        /// Called when the actor blueprint has been loaded into the solver.
        /// </summary>
        public event ActorBlueprintCallback OnBlueprintLoaded;

        /// <summary>
        /// Called when the actor blueprint has been unloaded from the solver.
        /// </summary>
        public event ActorBlueprintCallback OnBlueprintUnloaded;

        /// <summary>
        /// Called at the beginning of a time step.
        /// </summary>
        public event ActorStepCallback OnBeginStep;

        /// <summary>
        /// Called at the beginning of each substep.
        /// </summary>
        public event ActorStepCallback OnSubstep;

        /// <summary>
        /// Called at the end of a timestep, after external forces have been reset and collision callbacks called.
        /// </summary>
        public event ActorStepCallback OnEndStep;

        /// <summary>
        /// Called at the end of each frame.
        /// </summary>
        public event ActorCallback OnInterpolate;                       

        [HideInInspector][NonSerialized] protected int m_ActiveParticleCount = 0;

        /// <summary>
        /// Index of each one of the actor's particles in the solver.
        /// </summary>
        [HideInInspector][NonSerialized] public int[] solverIndices;

        /// <summary>
        /// For each of the actor's constraint types, offset of every batch in the solver.
        /// </summary>
        [HideInInspector][NonSerialized] public List<int>[] solverBatchOffsets;

        protected ObiSolver m_Solver;
        protected bool m_Loaded = false;

        private ObiActorBlueprint state;
        private ObiActorBlueprint m_BlueprintInstance;
        private ObiPinConstraintsData m_PinConstraints;
        [SerializeField] [HideInInspector] protected ObiCollisionMaterial m_CollisionMaterial;

        /// <summary>
        /// The solver in charge of simulating this actor.
        /// </summary>
        /// This is the first ObiSlver component found up the actor's hierarchy.
        public ObiSolver solver
        {
            get { return m_Solver; }
        }

        /// <summary>
        /// True if the actor blueprint has been loaded into a solver. Being true, guarantees actor.solver won't be null.
        /// </summary>
        public bool isLoaded
        {
            get { return m_Loaded; }
        }

        /// <summary>
        /// The collision material being used by this actor.
        /// </summary>
        public ObiCollisionMaterial collisionMaterial
        {
            get
            {
                return m_CollisionMaterial;
            }
            set
            {
                if (m_CollisionMaterial != value)
                {
                    m_CollisionMaterial = value;
                    UpdateCollisionMaterials();
                }
            }
        }

        /// <summary>
        /// Amount of particles allocated by this actor.
        /// </summary>
        public int particleCount
        {
            get
            {
                return sourceBlueprint != null ? sourceBlueprint.particleCount : 0;
            }
        }

        /// <summary>
        /// Amount of particles in use by this actor. 
        /// </summary>
        /// This will always be equal to or smaller than <see cref="particleCount"/>.
        public int activeParticleCount
        {
            get
            {
               return m_ActiveParticleCount;
            }
        }

        /// <summary>
        /// Whether this actors makes use of particle orientations or not.
        /// </summary>
        public bool usesOrientedParticles
        {
            get
            {
                return sourceBlueprint != null &&
                       sourceBlueprint.invRotationalMasses != null && sourceBlueprint.invRotationalMasses.Length > 0 &&
                       sourceBlueprint.orientations != null && sourceBlueprint.orientations.Length > 0 &&
                       sourceBlueprint.restOrientations != null && sourceBlueprint.restOrientations.Length > 0;
            }
        }

        /// <summary>
        /// If true, it means particles may not be completely spherical, but ellipsoidal.
        /// </summary>
        public virtual bool usesAnisotropicParticles
        {
            get
            {
                return false;
            }
        }

        /// <summary>
        /// If true, it means external forces aren't applied to the particles directly. 
        /// </summary>
        /// For instance, cloth uses aerodynamic constraints to do so, and fluid uses drag.
        public virtual bool usesCustomExternalForces
        {
            get { return false; }
        }

        /// <summary>
        /// Matrix that transforms from the actor's local space to the solver's local space.
        /// </summary>
        /// If there's no solver present, this is the same as the actor's local to world matrix.
        public Matrix4x4 actorLocalToSolverMatrix
        {
            get
            {
                if (m_Solver != null)
                    return m_Solver.transform.worldToLocalMatrix * transform.localToWorldMatrix;
                else
                    return transform.localToWorldMatrix;
            }
        }

        /// <summary>
        /// Matrix that transforms from the solver's local space to the actor's local space.
        /// </summary>
        /// If there's no solver present, this is the same as the actor's world to local matrix.
        /// This is always the same as the inverse of <see cref="actorLocalToSolverMatrix"/>.
        public Matrix4x4 actorSolverToLocalMatrix
        {
            get
            {
                if (m_Solver != null)
                    return transform.worldToLocalMatrix * m_Solver.transform.localToWorldMatrix;
                else
                    return transform.worldToLocalMatrix;
            }
        }

        /// <summary>
        /// Reference to the bleuprint asset sed by this actor.
        /// </summary>
        public abstract ObiActorBlueprint sourceBlueprint
        {
            get;
        }

        /// <summary>
        /// Reference to the blueprint in use by this actor. 
        /// </summary>
        /// If you haven't called <see cref="blueprint"/> before, this will be the same as <see cref="sourceBlueprint"/>.
        /// If you have called <see cref="blueprint"/> before, it will be the same as <see cref="blueprint"/>.
        public ObiActorBlueprint sharedBlueprint
        {
            get
            {
                if (m_BlueprintInstance != null)
                    return m_BlueprintInstance;
                return sourceBlueprint;
            }
        }

        /// <summary>
        /// Returns a unique instance of this actor's <see cref="sourceBlueprint"/>. 
        /// </summary>
        /// This is mostly used when the actor needs to change some blueprint data at runtime,
        /// and you don't want to change the blueprint asset as this would affect all other actors using it. Tearable cloth and ropes
        /// make use of this.
        public ObiActorBlueprint blueprint 
        {
            get
            {
                if (m_BlueprintInstance == null && sourceBlueprint != null)
                    m_BlueprintInstance = Instantiate(sourceBlueprint);

                return m_BlueprintInstance;
            }
        }

        protected void Awake()
        {
            solverBatchOffsets = new List<int>[Oni.ConstraintTypeCount];
            for (int i = 0; i < solverBatchOffsets.Length; ++i)
                solverBatchOffsets[i] = new List<int>();

            m_PinConstraints = new ObiPinConstraintsData();

#if UNITY_EDITOR

            // Check if this script's GameObject is in a PrefabStage
            var prefabStage = UnityEditor.Experimental.SceneManagement.PrefabStageUtility.GetPrefabStage(gameObject);

            if (prefabStage != null)
            {
                // Only create a solver if there's not one up our hierarchy.
                if (GetComponentInParent<ObiSolver>() == null)
                {
                    // Add our own environment root and move it to the PrefabStage scene
                    var newParent = new GameObject("ObiSolver (Environment)", typeof(ObiSolver), typeof(ObiLateFixedUpdater));
                    newParent.GetComponent<ObiLateFixedUpdater>().solvers.Add(newParent.GetComponent<ObiSolver>());
                    UnityEngine.SceneManagement.SceneManager.MoveGameObjectToScene(newParent, gameObject.scene);
                    transform.root.parent = newParent.transform;
                }
            }
#endif
        }

        protected void OnDestroy()
        {
            if (m_BlueprintInstance != null)
                DestroyImmediate(m_BlueprintInstance);
        }

        protected virtual void OnEnable()
        {
            // when an actor is enabled, grabs the first solver up its hierarchy,
            // initializes it (if not initialized) and gets added to it.
            m_Solver = GetComponentInParent<ObiSolver>();

            AddToSolver();
        }

        protected virtual void OnDisable()
        {
            RemoveFromSolver();
        }

        protected virtual void OnValidate()
        {
        }

        private void OnTransformParentChanged()
        {
            if (isActiveAndEnabled)
                SetSolver(GetComponentInParent<ObiSolver>());
        }

        /// <summary>
        /// Adds this actor to its solver, if any. Automatically called by <see cref="ObiSolver"/>.
        /// </summary>
        public void AddToSolver()
        {
            if (m_Solver != null)
            {
                if (!m_Solver.AddActor(this))
                    m_Solver = null;
                else if (sourceBlueprint != null)
                    sourceBlueprint.OnBlueprintGenerate += OnBlueprintRegenerate;
            }
        }

        /// <summary>
        /// Remove this actor from its solver, if any. Automatically called by <see cref="ObiSolver"/>.
        /// </summary>
        public void RemoveFromSolver()
        {
            if (m_Solver != null)
            {
                m_Solver.RemoveActor(this);
                if (sourceBlueprint != null)
                    sourceBlueprint.OnBlueprintGenerate -= OnBlueprintRegenerate;
            }
        }

        /// <summary>
        /// Forcibly changed the solver in charge of this actor
        /// </summary>
        /// <param name="newSolver"> The solver we want to put in charge of this actor.</param>  
        /// First it removes the actor from its current solver, then changes the actor's current solver and then readds it to this new solver.
        protected void SetSolver(ObiSolver newSolver)
        {
            // In case the first solver up our hierarchy is not the one we're currently in, change solver.
            if (newSolver != m_Solver)
            {
                RemoveFromSolver();

                m_Solver = newSolver;

                AddToSolver();
            }
        }

        protected virtual void OnBlueprintRegenerate(ObiActorBlueprint blueprint)
        {
            // Reload:
            RemoveFromSolver();
            AddToSolver();
        }

        protected void UpdateCollisionMaterials()
        {
            if (m_Solver != null && solverIndices != null)
            {
                int index = m_CollisionMaterial != null ? m_CollisionMaterial.handle.index : -1;
                for (int i = 0; i < solverIndices.Length; i++)
                    solver.collisionMaterials[solverIndices[i]] = index;
            }
        }

        /// <summary>
        /// Copies all data (position, velocity, phase, etc) from one particle to another one. 
        /// </summary>
        /// <param name="actorSourceIndex"> Index in the actor arrays of the particle we will copy data from.</param>
        /// <param name="actorDestIndex"> Index in the actor arrays of the particle we will copy data to.</param>
        /// <returns>
        /// Whether the indices passed are within actor bounds.
        /// </returns> 
        /// Extend this method to implement copying your own custom particle data in custom actors.
        public virtual bool CopyParticle(int actorSourceIndex, int actorDestIndex)
        {
            if (!isLoaded ||
                actorSourceIndex < 0 || actorSourceIndex >= solverIndices.Length ||
                actorDestIndex < 0 || actorDestIndex >= solverIndices.Length)
                return false;

            int sourceIndex = solverIndices[actorSourceIndex];
            int destIndex = solverIndices[actorDestIndex];

            // Copy solver data:
            m_Solver.prevPositions[destIndex] = m_Solver.prevPositions[sourceIndex];
            m_Solver.renderablePositions[destIndex] = m_Solver.renderablePositions[sourceIndex];
            m_Solver.startPositions[destIndex] = m_Solver.positions[destIndex] = m_Solver.positions[sourceIndex];
            m_Solver.startOrientations[destIndex] = m_Solver.orientations[destIndex] = m_Solver.orientations[sourceIndex];
            m_Solver.restPositions[destIndex] = m_Solver.restPositions[sourceIndex];
            m_Solver.restOrientations[destIndex] = m_Solver.restOrientations[sourceIndex];
            m_Solver.velocities[destIndex] = m_Solver.velocities[sourceIndex];
            m_Solver.angularVelocities[destIndex] = m_Solver.velocities[sourceIndex];
            m_Solver.invMasses[destIndex] = m_Solver.invMasses[sourceIndex];
            m_Solver.invRotationalMasses[destIndex] = m_Solver.invRotationalMasses[sourceIndex];
            m_Solver.principalRadii[destIndex] = m_Solver.principalRadii[sourceIndex];
            m_Solver.phases[destIndex] = m_Solver.phases[sourceIndex];
            m_Solver.colors[destIndex] = m_Solver.colors[sourceIndex];

            return true;
        }

        /// <summary>
        /// Teleports one actor particle to a certain position in solver space.
        /// </summary>
        /// <param name="actorIndex"> Index in the actor arrays of the particle we will teeleport.</param>
        /// <param name="position"> Position to teleport the particle to, expressed in solver space.</param>
        public void TeleportParticle(int actorIndex, Vector3 position)
        {
            if (!isLoaded || actorIndex < 0 || actorIndex >= solverIndices.Length)
                return;

            int solverIndex = solverIndices[actorIndex];

            Vector4 delta = (Vector4)position - m_Solver.positions[solverIndex];
            m_Solver.positions[solverIndex] += delta;
            m_Solver.prevPositions[solverIndex] += delta;
            m_Solver.renderablePositions[solverIndex] += delta;
            m_Solver.startPositions[solverIndex] += delta;

        }

        /// <summary>
        /// Teleports the entire actor to a new location / orientation.
        /// </summary>
        /// <param name="position"> World space position to teleport the actor to.</param>
        /// <param name="rotation"> World space rotation to teleport the actor to.</param>
        public virtual void Teleport(Vector3 position, Quaternion rotation)
        {
            if (!isLoaded)
                return;

            // Subtract current transform position/rotation, then add new world space position/rotation.
            // Lastly, set the transform to the new position/rotation.
            Matrix4x4 offset = solver.transform.worldToLocalMatrix *
                               Matrix4x4.TRS(position, Quaternion.identity, Vector3.one) *
                               Matrix4x4.TRS(Vector3.zero, rotation, Vector3.one) *
                               Matrix4x4.TRS(Vector3.zero, Quaternion.Inverse(transform.rotation), Vector3.one) *
                               Matrix4x4.TRS(-transform.position, Quaternion.identity, Vector3.one) *
                               solver.transform.localToWorldMatrix;

            Quaternion rotOffset = offset.rotation;

            for (int i = 0; i < solverIndices.Length; i++)
            {
                int solverIndex = solverIndices[i];

                m_Solver.positions[solverIndex] = 
                m_Solver.prevPositions[solverIndex] = 
                m_Solver.renderablePositions[solverIndex] = 
                m_Solver.startPositions[solverIndex] = offset.MultiplyPoint3x4(m_Solver.positions[solverIndex]);

                m_Solver.orientations[solverIndex] =
                m_Solver.prevOrientations[solverIndex] =
                m_Solver.renderableOrientations[solverIndex] =
                m_Solver.startOrientations[solverIndex] = rotOffset * m_Solver.orientations[solverIndex];

                m_Solver.velocities[solverIndex] = Vector4.zero;
                m_Solver.angularVelocities[solverIndex] = Vector4.zero;
            }

            transform.position = position;
            transform.rotation = rotation;

        }

        protected virtual void SwapWithFirstInactiveParticle(int actorIndex)
        {
            // update solver indices:
            m_Solver.particleToActor[solverIndices[actorIndex]].indexInActor = m_ActiveParticleCount;
            m_Solver.particleToActor[solverIndices[m_ActiveParticleCount]].indexInActor = actorIndex;

            solverIndices.Swap(actorIndex, m_ActiveParticleCount);
        }

        /// <summary>
        /// Activates one particle.
        /// </summary>
        /// <param name="actorIndex"> Index in the actor arrays of the particle we will activate.</param>
        /// <returns>
        /// True if the particle was inactive. False if the particle was already active.
        /// </returns> 
        /// This operation preserves the relative order of all particles.
        public bool ActivateParticle(int actorIndex)
        {
            if (IsParticleActive(actorIndex))
                return false;

            SwapWithFirstInactiveParticle(actorIndex);
            m_ActiveParticleCount++;
            m_Solver.dirtyActiveParticles = true;

            return true;
        }

        /// <summary>
        /// Deactivates one particle.
        /// </summary>
        /// <param name="actorIndex"> Index in the actor arrays of the particle we will deactivate.</param>
        /// <returns>
        /// True if the particle was active. False if the particle was already inactive.
        /// </returns> 
        /// This operation does not preserve the relative order of other particles, because the last active particle will
        /// swap positions with the particle being deactivated.
        public bool DeactivateParticle(int actorIndex)
        {
            if (!IsParticleActive(actorIndex))
                return false;

            m_ActiveParticleCount--;
            SwapWithFirstInactiveParticle(actorIndex);
            m_Solver.dirtyActiveParticles = true;

            return true;
        }

        /// <summary>
        /// Returns whether a given particle is active.
        /// </summary>
        /// <param name="actorIndex"> Index in the actor arrays of the particle.</param>
        /// <returns>
        /// True if the particle is active. False if the particle is inactive.
        /// </returns> 
        public bool IsParticleActive(int actorIndex)
        {
            return actorIndex < m_ActiveParticleCount;
        }

        /// <summary>
        /// Updates particle phases in the solver at runtime, including or removing the self-collision flag.
        /// </summary>
        public virtual void SetSelfCollisions(bool selfCollisions)
        {
            if (solver != null && Application.isPlaying && isLoaded)
            {
                for (int i = 0; i < particleCount; i++)
                {
                    if (selfCollisions)
                        solver.phases[solverIndices[i]] |= (int)Oni.ParticleFlags.SelfCollide;
                    else
                        solver.phases[solverIndices[i]] &= ~(int)Oni.ParticleFlags.SelfCollide;
                }
            }
        }

        /// <summary>
        /// Marks a given constraint type as dirty. 
        /// </summary>
        /// <param name="constraintType"> Type of the constraints that need re-creation.</param>
        /// This will cause the solver to perform a constraint re-creation at the start of the next step. Needed when the constraint data in an actor changes at runtime,
        /// as a result of changing topology (torn cloth or ropes), or changes in internal constraint parameters such as compliance values. This is a relatively expensive operation,
        /// so it's best to amortize as many constraint modification operations as possible in a single step.
        public void SetConstraintsDirty(Oni.ConstraintType constraintType)
        {
            if (solver != null)
                solver.dirtyConstraints |= (1 << (int)constraintType);
        }

        /// <summary>  
        /// Returns the data representation of constraints of a given type being simulated by this solver.
        /// </summary>  
        /// <param name="type"> Type of the constraints that will be returned by this method.</param>  
        /// <returns>
        /// The runtime constraints of the type speficied. Most constraints are stored in the blueprint, with a couple notable exceptions: pin and stitch constraints
        /// are always created at runtime, so they're not stored in the blueprint but in the actor itself.
        /// </returns> 
        public IObiConstraints GetConstraintsByType(Oni.ConstraintType type)
        {
            // pin constraints are a special case, because they're not stored in a blueprint. They are created at runtime at stored in the actor itself.
            if (type == Oni.ConstraintType.Pin)
                return m_PinConstraints;

            if (sharedBlueprint != null)
                return sharedBlueprint.GetConstraintsByType(type);

            return null;
        }



        /// <summary>  
        /// Call when some particle properties have been modified and need updating.
        /// </summary>  
        /// Does not do anything by default. Call when manually modifying particle properties in the solver, should the actor need to do some book keeping.
        /// For softbodies, updates their rest state. 
        public virtual void UpdateParticleProperties()
        {
        }

        /// <summary>  
        /// Returns the index of this particle in the solver arrays.
        /// </summary>  
        /// <param name="actorIndex"> Index of the particle in the actor arrays.</param>  
        /// <returns>
        /// The index of a given particle in the solver arrays.
        /// </returns>
        /// At runtime when the blueprint is loaded, this is the same as calling actor.solverIndices[solverIndex].
        /// If the blueprint is not loaded it will return the same index passed to it: actorIndex.
        /// Note that this function does not perform any range checking.
        public int GetParticleRuntimeIndex(int actorIndex)
        {
            if (isLoaded)
                return solverIndices[actorIndex];
            return actorIndex;
        }


        /// <summary>  
        /// Given a solver particle index, returns the position of that particle in world space. 
        /// </summary>  
        /// <param name="solverIndex"> Index of the particle in the solver arrays.</param>  
        /// <returns>
        /// The position of a given particle in world space.
        /// </returns>
        public Vector3 GetParticlePosition(int solverIndex)
        {
            if (isLoaded)
                return m_Solver.transform.TransformPoint(m_Solver.renderablePositions[solverIndex]);
            return Vector3.zero;
        }

        /// <summary>  
        /// Given a solver particle index, returns the orientation of that particle in world space. 
        /// </summary>  
        /// <param name="solverIndex"> Index of the particle in the solver arrays.</param>  
        /// <returns>
        /// The orientation of a given particle in world space.
        /// </returns>
        public Quaternion GetParticleOrientation(int solverIndex)
        {
            if (isLoaded)
                return m_Solver.transform.rotation * m_Solver.renderableOrientations[solverIndex];
            return Quaternion.identity;
        }

        /**
         *  
         */

        /// <summary>  
        /// Given a solver particle index, returns the anisotropic frame of that particle in world space.
        /// </summary>  
        /// <param name="solverIndex"> Index of the particle in the solver arrays.</param>
        /// <param name="solverIndex"> First basis vector of the frame. Contains particle radius along this axis in the 4th position.</param>
        /// <param name="solverIndex"> Second basis vector of the frame. Contains particle radius along this axis in the 4th position..</param>
        /// <param name="solverIndex"> Third basis vector of the frame. Contains particle radius along this axis in the 4th position.</param>
        public void GetParticleAnisotropy(int solverIndex, ref Vector4 b1, ref Vector4 b2, ref Vector4 b3)
        {
            if (isLoaded && usesAnisotropicParticles)
            {
                int baseIndex = solverIndex * 3;

                b1 = m_Solver.transform.TransformDirection(m_Solver.anisotropies[baseIndex]);
                b2 = m_Solver.transform.TransformDirection(m_Solver.anisotropies[baseIndex + 1]);
                b3 = m_Solver.transform.TransformDirection(m_Solver.anisotropies[baseIndex + 2]);

                b1[3] = m_Solver.maxScale * m_Solver.anisotropies[baseIndex][3];
                b2[3] = m_Solver.maxScale * m_Solver.anisotropies[baseIndex + 1][3];
                b3[3] = m_Solver.maxScale * m_Solver.anisotropies[baseIndex + 2][3];
            }
            else
            {
                b1[3] = b2[3] = b3[3] = m_Solver.maxScale * m_Solver.principalRadii[solverIndex][0];
            }
        }

        /// <summary>  
        /// Given a solver particle index, returns the maximum world space radius of that particle, in any axis.
        /// </summary>  
        /// <param name="solverIndex"> Index of the particle in the solver arrays.</param>  
        /// <returns>
        /// The maximum radius of a given particle in world space.
        /// </returns>
        public float GetParticleMaxRadius(int solverIndex)
        {
            if (isLoaded)
                return m_Solver.maxScale * m_Solver.principalRadii[solverIndex][0];
            return 0;
        }


        /// <summary>  
        /// Given a solver particle index, returns the color of that particle.
        /// </summary>  
        /// <param name="solverIndex"> Index of the particle in the solver arrays.</param>  
        /// <returns>
        /// The color of the particle.
        /// </returns>
        public Color GetParticleColor(int solverIndex)
        {
            if (isLoaded)
                return m_Solver.colors[solverIndex];
            return Color.white;
        }

        /// <summary>  
        /// Sets a given phase value for all particles in the actor.
        /// </summary>  
        /// <param name="newPhase"> Phase value.</param>  
        public void SetPhase(int newPhase)
        {
            newPhase = Mathf.Clamp(newPhase, 0, (1 << 24) - 1);

            for (int i = 0; i < particleCount; ++i)
            {
                int solverIndex = solverIndices[i];
                var flags = (Oni.ParticleFlags)ObiUtils.GetFlagsFromPhase(solver.phases[solverIndex]);
                solver.phases[solverIndex] = ObiUtils.MakePhase(newPhase, flags);
            }
        }

        /// <summary>  
        /// Sets the inverse mass of each particle so that the total actor mass matches the one passed by parameter.
        /// </summary>  
        /// <param name="mass"> The actor's total mass.</param>  
        public void SetMass(float mass)
        {
            if (Application.isPlaying && isLoaded && activeParticleCount > 0)
            {
                float invMass = 1.0f / (mass / activeParticleCount);

                for (int i = 0; i < activeParticleCount; ++i)
                {
                    int solverIndex = solverIndices[i];
                    m_Solver.invMasses[solverIndex] = invMass;
                    m_Solver.invRotationalMasses[solverIndex] = invMass;
                }
            }
        }

        /// <summary>  
        /// Returns the actor's mass (sum of all particle masses), and the position of its center of mass.
        /// </summary>  
        /// <param name="com"> The actor's center of mass, expressed in solver space.</param>
        /// Particles with infinite mass (invMass = 0) are ignored.
        public float GetMass(out Vector3 com)
        {

            float actorMass = 0;
            com = Vector3.zero;

            if (Application.isPlaying && isLoaded && activeParticleCount > 0)
            {
                Vector4 com4 = Vector4.zero;

                for (int i = 0; i < activeParticleCount; ++i)
                {
                    if (solver.invMasses[solverIndices[i]] > 0)
                    {
                        float mass = 1.0f / solver.invMasses[solverIndices[i]];
                        actorMass += mass;
                        com4 += solver.positions[solverIndices[i]] * mass;
                    }
                }

                com = com4;
                if (actorMass > float.Epsilon)
                    com /= actorMass;
            }

            return actorMass;
        }

        /// <summary>  
        ///Adds an external force to all particles in the actor. 
        /// </summary>  
        /// <param name="force"> Value expressed in solver space.</param>
        /// <param name="forceMode"> Type of "force" applied.</param>
        public void AddForce(Vector3 force, ForceMode forceMode)
        {

            Vector3 com;
            float mass = GetMass(out com);

            if (!float.IsInfinity(mass))
            {

                Vector4 bodyForce = force;

                switch (forceMode)
                {
                    case ForceMode.Force:
                        {

                            bodyForce /= mass;

                            for (int i = 0; i < solverIndices.Length; ++i)
                                m_Solver.externalForces[solverIndices[i]] += bodyForce / m_Solver.invMasses[solverIndices[i]];

                        }
                        break;
                    case ForceMode.Acceleration:
                        {

                            for (int i = 0; i < solverIndices.Length; ++i)
                                m_Solver.externalForces[solverIndices[i]] += bodyForce / m_Solver.invMasses[solverIndices[i]];

                        }
                        break;
                    case ForceMode.Impulse:
                        {

                            bodyForce /= mass;

                            for (int i = 0; i < solverIndices.Length; ++i)
                                m_Solver.externalForces[solverIndices[i]] += bodyForce / m_Solver.invMasses[solverIndices[i]] / Time.fixedDeltaTime;

                        }
                        break;
                    case ForceMode.VelocityChange:
                        {

                            for (int i = 0; i < solverIndices.Length; ++i)
                                m_Solver.externalForces[solverIndices[i]] += bodyForce / m_Solver.invMasses[solverIndices[i]] / Time.fixedDeltaTime;

                        }
                        break;
                }
            }
        }

        /// <summary>  
        /// Adds a torque to the actor.
        /// </summary>  
        /// <param name="force"> Value expressed in solver space.</param>
        /// <param name="forceMode"> Type of "torque" applied.</param>
        public void AddTorque(Vector3 force, ForceMode forceMode)
        {

            Vector3 com;
            float mass = GetMass(out com);

            if (!float.IsInfinity(mass))
            {

                Vector3 bodyForce = force;

                switch (forceMode)
                {
                    case ForceMode.Force:
                        {

                            bodyForce /= mass;

                            for (int i = 0; i < solverIndices.Length; ++i)
                            {

                                Vector3 v = Vector3.Cross(bodyForce / m_Solver.invMasses[solverIndices[i]], (Vector3)m_Solver.positions[solverIndices[i]] - com);
                                m_Solver.externalForces[solverIndices[i]] += new Vector4(v.x, v.y, v.z, 0);
                            }

                        }
                        break;
                    case ForceMode.Acceleration:
                        {

                            for (int i = 0; i < solverIndices.Length; ++i)
                            {

                                Vector3 v = Vector3.Cross(bodyForce / m_Solver.invMasses[solverIndices[i]], (Vector3)m_Solver.positions[solverIndices[i]] - com);
                                m_Solver.externalForces[solverIndices[i]] += new Vector4(v.x, v.y, v.z, 0);
                            }

                        }
                        break;
                    case ForceMode.Impulse:
                        {

                            bodyForce /= mass;

                            for (int i = 0; i < solverIndices.Length; ++i)
                            {

                                Vector3 v = Vector3.Cross(bodyForce / m_Solver.invMasses[solverIndices[i]] / Time.fixedDeltaTime, (Vector3)m_Solver.positions[solverIndices[i]] - com);
                                m_Solver.externalForces[solverIndices[i]] += new Vector4(v.x, v.y, v.z, 0);
                            }

                        }
                        break;
                    case ForceMode.VelocityChange:
                        {

                            for (int i = 0; i < solverIndices.Length; ++i)
                            {

                                Vector3 v = Vector3.Cross(bodyForce / m_Solver.invMasses[solverIndices[i]] / Time.fixedDeltaTime, (Vector3)m_Solver.positions[solverIndices[i]] - com);
                                m_Solver.externalForces[solverIndices[i]] += new Vector4(v.x, v.y, v.z, 0);
                            }

                        }
                        break;
                }
            }
        }

        #region Blueprints

        private void LoadBlueprintParticles(ObiActorBlueprint bp)
        {

            Matrix4x4 l2sTransform = actorLocalToSolverMatrix;
            Quaternion l2sRotation = l2sTransform.rotation;

            for (int i = 0; i < solverIndices.Length; i++)
            {
                int k = solverIndices[i];

                if (bp.positions != null && i < bp.positions.Length)
                {
                    m_Solver.startPositions[k] = m_Solver.prevPositions[k] = m_Solver.positions[k] = l2sTransform.MultiplyPoint3x4(bp.positions[i]);
                    m_Solver.renderablePositions[k] = l2sTransform.MultiplyPoint3x4(bp.positions[i]);
                }

                if (bp.orientations != null && i < bp.orientations.Length)
                {
                    m_Solver.startOrientations[k] = m_Solver.prevOrientations[k] = m_Solver.orientations[k] = l2sRotation * bp.orientations[i];
                    m_Solver.renderableOrientations[k] = l2sRotation * bp.orientations[i];
                }

                if (bp.velocities != null && i < bp.velocities.Length)
                    m_Solver.velocities[k] = l2sTransform.MultiplyVector(bp.velocities[i]);

                if (bp.angularVelocities != null && i < bp.angularVelocities.Length)
                    m_Solver.angularVelocities[k] = l2sTransform.MultiplyVector(bp.angularVelocities[i]);

                if (bp.invMasses != null && i < bp.invMasses.Length)
                    m_Solver.invMasses[k] = bp.invMasses[i];

                if (bp.invRotationalMasses != null && i < bp.invRotationalMasses.Length)
                    m_Solver.invRotationalMasses[k] = bp.invRotationalMasses[i];

                if (bp.principalRadii != null && i < bp.principalRadii.Length)
                    m_Solver.principalRadii[k] = bp.principalRadii[i];

                if (bp.phases != null && i < bp.phases.Length)
                    m_Solver.phases[k] = ObiUtils.MakePhase(bp.phases[i],0);

                if (bp.restPositions != null && i < bp.restPositions.Length)
                    m_Solver.restPositions[k] = bp.restPositions[i];

                if (bp.restOrientations != null && i < bp.restOrientations.Length)
                    m_Solver.restOrientations[k] = bp.restOrientations[i];

                if (bp.colors != null && i < bp.colors.Length)
                    m_Solver.colors[k] = bp.colors[i];

            }

            m_ActiveParticleCount = sourceBlueprint.activeParticleCount;
            m_Solver.dirtyActiveParticles = true;

            // Push collision materials:
            UpdateCollisionMaterials();

        }

        private void UnloadBlueprintParticles()
        {
            // Update active particles. 
            m_ActiveParticleCount = 0;
            m_Solver.dirtyActiveParticles = true;
        }

        /// <summary>  
        /// Resets the position and velocity of all particles, to the values stored in the blueprint.
        /// </summary>  
        /// Note however
        /// that this does not affect constraints, so if you've torn a cloth/rope or resized a rope, calling ResetParticles won't restore
        /// the initial topology of the actor.
        public void ResetParticles()
        {
            if (isLoaded)
            {
                Matrix4x4 l2sTransform = actorLocalToSolverMatrix;
                Quaternion l2sRotation = l2sTransform.rotation;

                for (int i = 0; i < particleCount; ++i)
                {
                    int solverIndex = solverIndices[i];

                    solver.positions[solverIndex] = l2sTransform.MultiplyPoint3x4(sourceBlueprint.positions[i]);
                    solver.velocities[solverIndex] = l2sTransform.MultiplyVector(sourceBlueprint.velocities[i]);

                    if (usesOrientedParticles)
                    {
                        solver.orientations[solverIndex] = l2sRotation * sourceBlueprint.orientations[i];
                        solver.angularVelocities[solverIndex] = l2sTransform.MultiplyVector(sourceBlueprint.angularVelocities[i]);
                    }
                }
            }
        }

        #endregion

        #region State


        /// <summary>  
        /// Resets the position and velocity of all particles, to the values stored in the blueprint.
        /// </summary>  
        /// <param name="com"> The blueprint that we want to fill with current particle data.</param>
        /// Note that this will not resize the blueprint's data arrays, and that it does not perform range checking. For this reason,
        /// you must supply a blueprint large enough to store all particles' data.
        public void SaveStateToBlueprint(ObiActorBlueprint bp)
        {
            if (bp == null)
                return;

            Matrix4x4 l2sTransform = actorLocalToSolverMatrix.inverse;
            Quaternion l2sRotation = l2sTransform.rotation;

            for (int i = 0; i < solverIndices.Length; i++)
            {
                int k = solverIndices[i];

                if (m_Solver.positions != null && k < m_Solver.positions.count)
                    bp.positions[i] = l2sTransform.MultiplyPoint3x4(m_Solver.positions[k]);

                if (m_Solver.velocities != null && k < m_Solver.velocities.count)
                    bp.velocities[i] = l2sTransform.MultiplyVector(m_Solver.velocities[k]);
            }
        }

        protected void StoreState()
        {
            DestroyImmediate(state);
            state = Instantiate<ObiActorBlueprint>(sourceBlueprint);
            SaveStateToBlueprint(state);
        }

        public void ClearState()
        {
            DestroyImmediate(state);
        }

        #endregion

        #region Solver callbacks

        /// <summary>  
        /// Loads this actor's blueprint into a given solver. Automatically called by <see cref="ObiSolver"/>.
        /// </summary> 
        public virtual void LoadBlueprint(ObiSolver solver)
        {
            var bp = sharedBlueprint;

            // in case we have temporary state, load that instead of the original blueprint.
            if (Application.isPlaying)
            {
                bp = state != null ? state : sourceBlueprint;
            }

            m_Loaded = true;

            LoadBlueprintParticles(bp);
            solver.dirtyConstraints |= ~0;

            if (OnBlueprintLoaded != null)
                OnBlueprintLoaded(this, null);
        }

        /// <summary>  
        /// Unloads this actor's blueprint from a given solver. Automatically called by <see cref="ObiSolver"/>.
        /// </summary> 
        public virtual void UnloadBlueprint(ObiSolver solver)
        {
            // instantiate blueprint and store current state in the instance:
            if (Application.isPlaying)
            {
                StoreState();
            }

            m_Loaded = false;

            // unload the blueprint.
            solver.dirtyConstraints |= ~0;
            UnloadBlueprintParticles();

            if (OnBlueprintUnloaded != null)
                OnBlueprintUnloaded(this, null);
        }

        public virtual void BeginStep(float stepTime)
        {
            UpdateCollisionMaterials();

            if (OnBeginStep != null)
                OnBeginStep(this,stepTime); 
        }

        public virtual void Substep(float substepTime)
        {
            if (OnSubstep != null)
                OnSubstep(this,substepTime);
        }

        public virtual void EndStep(float substepTime)
        {
            if (OnEndStep != null)
                OnEndStep(this,substepTime);
        }

        public virtual void Interpolate() 
        {
            // Update particle renderable positions/orientations in the solver:
            if (!Application.isPlaying && isLoaded)
            {
                Matrix4x4 l2sTransform = actorLocalToSolverMatrix;
                Quaternion l2sRotation = l2sTransform.rotation;

                for (int i = 0; i < solverIndices.Length; i++)
                {
                    int k = solverIndices[i];

                    if (sourceBlueprint.positions != null && i < sourceBlueprint.positions.Length)
                    {
                        m_Solver.renderablePositions[k] = l2sTransform.MultiplyPoint3x4(sourceBlueprint.positions[i]);
                    }

                    if (sourceBlueprint.orientations != null && i < sourceBlueprint.orientations.Length)
                    {
                        m_Solver.renderableOrientations[k] = l2sRotation * sourceBlueprint.orientations[i];
                    }
                }
            }

            if (OnInterpolate != null)
                OnInterpolate(this);
        }

        public virtual void OnSolverVisibilityChanged(bool visible)
        {
        }

        #endregion


    }
}

