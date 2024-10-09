using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace Obi
{

    public abstract class ObiActorBlueprint : ScriptableObject, IObiParticleCollection
    {
        public delegate void BlueprintCallback(ObiActorBlueprint blueprint);
        public event BlueprintCallback OnBlueprintGenerate;

        [HideInInspector] [SerializeField] protected bool m_Empty = true;
        [HideInInspector] [SerializeField] protected int m_ActiveParticleCount = 0;
        [HideInInspector] [SerializeField] protected int m_InitialActiveParticleCount = 0;
        [HideInInspector] [SerializeField] protected Bounds _bounds = new Bounds();

        /**Particle components*/
        [HideInInspector] public Vector3[] positions = null;           /**< Particle positions.*/
        [HideInInspector] public Vector4[] restPositions = null;       /**< Particle rest positions, used to filter collisions.*/

        [HideInInspector] public Quaternion[] orientations = null;     /**< Particle orientations.*/
        [HideInInspector] public Quaternion[] restOrientations = null; /**< Particle rest orientations.*/

        [HideInInspector] public Vector3[] velocities = null;          /**< Particle velocities.*/
        [HideInInspector] public Vector3[] angularVelocities = null;   /**< Particle angular velocities.*/

        [HideInInspector] public float[] invMasses = null;             /**< Particle inverse masses*/
        [HideInInspector] public float[] invRotationalMasses = null;

        [HideInInspector] public int[] phases = null;                  /**< Particle phases.*/
        [HideInInspector] public Vector3[] principalRadii = null;      /**< Particle ellipsoid principal radii. These are the ellipsoid radius in each axis.*/
        [HideInInspector] public Color[] colors = null;                /**< Particle colors (not used by all actors, can be null)*/

        /** Constraint components. Each constraint type contains a list of constraint batches.*/
        [HideInInspector] public ObiDistanceConstraintsData distanceConstraintsData = null;
        [HideInInspector] public ObiBendConstraintsData bendConstraintsData = null;
        [HideInInspector] public ObiSkinConstraintsData skinConstraintsData = null;
        [HideInInspector] public ObiTetherConstraintsData tetherConstraintsData = null;
        [HideInInspector] public ObiStretchShearConstraintsData stretchShearConstraintsData = null;
        [HideInInspector] public ObiBendTwistConstraintsData bendTwistConstraintsData = null;
        [HideInInspector] public ObiShapeMatchingConstraintsData shapeMatchingConstraintsData = null;
        [HideInInspector] public ObiAerodynamicConstraintsData aerodynamicConstraintsData = null;
        [HideInInspector] public ObiChainConstraintsData chainConstraintsData = null;
        [HideInInspector] public ObiVolumeConstraintsData volumeConstraintsData = null;

        /** Particle groups.*/
        [HideInInspector] public List<ObiParticleGroup> groups = new List<ObiParticleGroup>();

        /**
         * Returns the amount of particles used by this blueprint.
         */
        public int particleCount
        {
            get { return positions != null ? positions.Length : 0; }
        }

        public int activeParticleCount
        {
            get { return m_ActiveParticleCount; }
        }

        /**
         * Returns whether this group uses oriented particles.
         */
        public bool usesOrientedParticles
        {
            get
            {
                return invRotationalMasses != null && invRotationalMasses.Length > 0 &&
                       orientations != null && orientations.Length > 0 &&
                       restOrientations != null && restOrientations.Length > 0;
            }
        }

        public virtual bool usesTethers 
        {
            get { return false; }
        }

        public bool IsParticleActive(int index)
        {
            return index < m_ActiveParticleCount;
        }

        protected virtual void SwapWithFirstInactiveParticle(int index)
        {
            positions.Swap(index, m_ActiveParticleCount);
            restPositions.Swap(index, m_ActiveParticleCount);
            orientations.Swap(index, m_ActiveParticleCount);
            restOrientations.Swap(index, m_ActiveParticleCount);
            velocities.Swap(index, m_ActiveParticleCount);
            angularVelocities.Swap(index, m_ActiveParticleCount);
            invMasses.Swap(index, m_ActiveParticleCount);
            invRotationalMasses.Swap(index, m_ActiveParticleCount);
            phases.Swap(index, m_ActiveParticleCount);
            principalRadii.Swap(index, m_ActiveParticleCount);
            colors.Swap(index, m_ActiveParticleCount);
        }

        /** 
         * Activates one particle. This operation preserves the relative order of all particles.
         */
        public bool ActivateParticle(int index)
        {
            if (IsParticleActive(index))
                return false;

            SwapWithFirstInactiveParticle(index);
            m_ActiveParticleCount++;

            return true;
        }

        /** 
         * Deactivates one particle. This operation does not preserve the relative order of other particles, because the last active particle will
         * swap positions with the particle being deactivated.
         */
        public bool DeactivateParticle(int index)
        {
            if (!IsParticleActive(index))
                return false;

            m_ActiveParticleCount--;
            SwapWithFirstInactiveParticle(index);

            return true;
        }

        public bool empty
        {
            get { return m_Empty; }
        }

        public void RecalculateBounds()
        {
            if (positions.Length > 0)
            {
                _bounds = new Bounds(positions[0],Vector3.zero);
                for (int i = 1; i < positions.Length; ++i)
                    _bounds.Encapsulate(positions[i]);
            }
            else
                _bounds = new Bounds();
        }

        public Bounds bounds
        {
            get { return _bounds; }
        }

        public IEnumerable<IObiConstraints> GetConstraints()
        {
            if (distanceConstraintsData != null && distanceConstraintsData.GetBatchCount() > 0)
                yield return distanceConstraintsData;
            if (bendConstraintsData != null && bendConstraintsData.GetBatchCount() > 0)
                yield return bendConstraintsData;
            if (skinConstraintsData != null && skinConstraintsData.GetBatchCount() > 0)
                yield return skinConstraintsData;
            if (tetherConstraintsData != null && tetherConstraintsData.GetBatchCount() > 0)
                yield return tetherConstraintsData;
            if (stretchShearConstraintsData != null && stretchShearConstraintsData.GetBatchCount() > 0)
                yield return stretchShearConstraintsData;
            if (bendTwistConstraintsData != null && bendTwistConstraintsData.GetBatchCount() > 0)
                yield return bendTwistConstraintsData;
            if (shapeMatchingConstraintsData != null && shapeMatchingConstraintsData.GetBatchCount() > 0)
                yield return shapeMatchingConstraintsData;
            if (aerodynamicConstraintsData != null && aerodynamicConstraintsData.GetBatchCount() > 0)
                yield return aerodynamicConstraintsData;
            if (chainConstraintsData != null && chainConstraintsData.GetBatchCount() > 0)
                yield return chainConstraintsData;
            if (volumeConstraintsData != null && volumeConstraintsData.GetBatchCount() > 0)
                yield return volumeConstraintsData;
        }

        public IObiConstraints GetConstraintsByType(Oni.ConstraintType type)
        {
            switch (type)
            {
                case Oni.ConstraintType.Distance: return distanceConstraintsData;
                case Oni.ConstraintType.Bending: return bendConstraintsData;
                case Oni.ConstraintType.Skin: return skinConstraintsData;
                case Oni.ConstraintType.Tether: return tetherConstraintsData;
                case Oni.ConstraintType.BendTwist: return bendTwistConstraintsData;
                case Oni.ConstraintType.StretchShear: return stretchShearConstraintsData;
                case Oni.ConstraintType.ShapeMatching: return shapeMatchingConstraintsData;
                case Oni.ConstraintType.Aerodynamics: return aerodynamicConstraintsData;
                case Oni.ConstraintType.Chain: return chainConstraintsData;
                case Oni.ConstraintType.Volume: return volumeConstraintsData;
                default: return null;
            }
        }

        public int GetParticleRuntimeIndex(int blueprintIndex)
        {
            return blueprintIndex;
        }

        public Vector3 GetParticlePosition(int index)
        {
            if (positions != null && index < positions.Length)
            {
                return positions[index];
            }
            return Vector3.zero;
        }

        public Quaternion GetParticleOrientation(int index)
        {
            if (orientations != null && index < orientations.Length)
            {
                return orientations[index];
            }
            return Quaternion.identity;
        }

        public void GetParticleAnisotropy(int index, ref Vector4 b1, ref Vector4 b2, ref Vector4 b3)
        {
            if (orientations != null && index < orientations.Length)
            {

                Quaternion orientation = orientations[index];

                b1 = orientation * Vector3.right;
                b2 = orientation * Vector3.up;
                b3 = orientation * Vector3.forward;

                b1[3] = principalRadii[index][0];
                b2[3] = principalRadii[index][1];
                b3[3] = principalRadii[index][2];

            }
            else
            {
                b1[3] = b2[3] = b3[3] = principalRadii[index][0];
            }
        }

        public float GetParticleMaxRadius(int index)
        {
            if (principalRadii != null && index < principalRadii.Length)
            {
                return principalRadii[index][0];
            }
            return 0;
        }

        public Color GetParticleColor(int index)
        {
            if (colors != null && index < colors.Length)
            {
                return colors[index];
            }
            else
                return Color.white;
        }

        public void GenerateImmediate()
        {
            var g = Generate();
            while (g.MoveNext()){}
        }

        public IEnumerator Generate()
        {
            m_Empty = true;

            m_ActiveParticleCount = 0;
            distanceConstraintsData = null;
            bendConstraintsData = null;
            skinConstraintsData = null;
            tetherConstraintsData = null;
            bendTwistConstraintsData = null;
            stretchShearConstraintsData = null;
            shapeMatchingConstraintsData = null;
            aerodynamicConstraintsData = null;
            chainConstraintsData = null;
            volumeConstraintsData = null;

            IEnumerator g = Initialize();

            while (g.MoveNext())
                yield return g.Current;

            RecalculateBounds();

            m_Empty = false;
            m_InitialActiveParticleCount = m_ActiveParticleCount;

            foreach (IObiConstraints constraints in GetConstraints())
                for (int i = 0; i < constraints.GetBatchCount(); ++i)
                    constraints.GetBatch(i).initialActiveConstraintCount = constraints.GetBatch(i).activeConstraintCount;

#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif

            if (OnBlueprintGenerate != null)
                OnBlueprintGenerate(this);
        }

        public ObiParticleGroup InsertNewParticleGroup(string name, int index)
        {
            if (index >= 0 && index <= groups.Count)
            {
                ObiParticleGroup group = ScriptableObject.CreateInstance<ObiParticleGroup>();
                group.SetSourceBlueprint(this);
                group.name = name;

#if UNITY_EDITOR
                if (!Application.isPlaying)
                {
                    AssetDatabase.AddObjectToAsset(group, this);
                    Undo.RegisterCreatedObjectUndo(group, "Insert particle group");

                    Undo.RecordObject(this, "Insert particle group");
                    groups.Insert(index, group);

                    if (EditorUtility.IsPersistent(this))
                    {
                        EditorUtility.SetDirty(this);
                        AssetDatabase.SaveAssets();
                    }
                }
                else
#endif
                {
                    groups.Insert(index, group);
                }

                return group;
            }
            return null;
        }

        public ObiParticleGroup AppendNewParticleGroup(string name)
        {
            return InsertNewParticleGroup(name, groups.Count);
        }

        public bool RemoveParticleGroupAt(int index)
        {
            if (index >= 0 && index < groups.Count)
            {
#if UNITY_EDITOR
                if (!Application.isPlaying)
                {
                    Undo.RecordObject(this, "Remove particle group");

                    var group = groups[index];
                    groups.RemoveAt(index);

                    if (group != null)
                        Undo.DestroyObjectImmediate(group);

                    if (EditorUtility.IsPersistent(this))
                    {
                        EditorUtility.SetDirty(this);
                        AssetDatabase.SaveAssets();
                    }
                }
                else
#endif
                {
                    var group = groups[index];
                    groups.RemoveAt(index);

                    if (group != null)
                        DestroyImmediate(group, true);
                }

                return true;
            }
            return false;
        }

        public bool SetParticleGroupName(int index, string name)
        {
            if (index >= 0 && index < groups.Count)
            {
#if UNITY_EDITOR
                if (!Application.isPlaying)
                {
                    Undo.RecordObject(this, "Set particle group name");
                    groups[index].name = name;

                    if (EditorUtility.IsPersistent(this))
                    {
                        EditorUtility.SetDirty(this);
                        AssetDatabase.SaveAssets();
                    }
                }
                else
#endif
                {
                    groups[index].name = name;
                }

                return true;
            }
            return false;
        }

        public void ClearParticleGroups()
        {
#if UNITY_EDITOR
            if (!Application.isPlaying)
            {
                Undo.RecordObject(this, "Clear particle groups");
                for (int i = 0; i < groups.Count; ++i)
                    if (groups[i] != null)
                        Undo.DestroyObjectImmediate(groups[i]);

                if (EditorUtility.IsPersistent(this))
                {
                    EditorUtility.SetDirty(this);
                    AssetDatabase.SaveAssets();
                }
            }
            else
#endif
            {
                for (int i = 0; i < groups.Count; ++i)
                    if (groups[i] != null)
                        DestroyImmediate(groups[i], true);
            }

            groups.Clear();
        }

        private bool IsParticleSharedInConstraint(int index, List<int> particles, bool[] selected)
        {
            bool containsCurrent = false;
            bool containsUnselected = false;

            for (int k = 0; k < particles.Count; ++k)
            {
                containsCurrent    |= particles[k] == index;
                containsUnselected |= !selected[particles[k]];

                if (containsCurrent && containsUnselected)
                {
                    return true;
                }
            }
            return false;
        }

        private bool DoesParticleShareConstraints(IObiConstraints constraints, int index, List<int> particles, bool[] selected)
        {
            bool shared = false;
            for (int i = 0; i < constraints.GetBatchCount(); ++i)
            {
                var batch = constraints.GetBatch(i);
                for (int j = 0; j < batch.activeConstraintCount; ++j)
                {
                    particles.Clear();
                    batch.GetParticlesInvolved(j, particles);

                    if (shared |= IsParticleSharedInConstraint(index, particles, selected))
                        break;
                }

                if (shared)
                    break;
            }
            return shared;
        }

        private void DeactivateConstraintsWithInactiveParticles(IObiConstraints constraints, List<int> particles)
        {
            for (int j = 0; j < constraints.GetBatchCount(); ++j)
            {
                var batch = constraints.GetBatch(j);

                for (int i = batch.activeConstraintCount - 1; i >= 0; --i)
                {
                    particles.Clear();
                    batch.GetParticlesInvolved(i, particles);
                    for (int k = 0; k < particles.Count; ++k)
                    {
                        if (!IsParticleActive(particles[k]))
                        {
                            batch.DeactivateConstraint(i);
                            break;
                        }
                    }
                }
            }
        }

        private void ParticlesSwappedInGroups(int index, int newIndex)
        {
            // Update groups:
            foreach (ObiParticleGroup group in groups)
            {
                for (int i = 0; i < group.particleIndices.Count; ++i)
                {
                    if (group.particleIndices[i] == newIndex)
                        group.particleIndices[i] = index;
                    else if (group.particleIndices[i] == index)
                        group.particleIndices[i] = newIndex;
                }
            }
        }

        public void RemoveSelectedParticles(ref bool[] selected, bool optimize = true)
        {
            List<int> particles = new List<int>();

            // iterate over all particles and get those selected ones that are only constrained to other selected ones.
            for (int i = activeParticleCount - 1; i >= 0; --i)
            {
                // if the particle is not selected for optimization, skip it.
                if (!selected[i])
                    continue;

                // look if the particle shares distance or shape matching constraints with an unselected particle.
                bool shared = false;
                if (optimize)
                {
                    shared |= DoesParticleShareConstraints(distanceConstraintsData, i, particles, selected);
                    shared |= DoesParticleShareConstraints(bendConstraintsData, i, particles, selected);
                    shared |= DoesParticleShareConstraints(shapeMatchingConstraintsData, i, particles, selected);
                }

                if (!shared)
                {
                    if (DeactivateParticle(i))
                    {
                        selected.Swap(i, m_ActiveParticleCount);

                        // Update constraints:
                        foreach (IObiConstraints constraints in GetConstraints())
                            for (int j = 0; j < constraints.GetBatchCount(); ++j)
                                constraints.GetBatch(j).ParticlesSwapped(i, m_ActiveParticleCount);

                        // Update groups:
                        ParticlesSwappedInGroups(i, m_ActiveParticleCount);
                    }
                }

            }

            // deactivate all constraints that reference inactive particles:
            foreach (IObiConstraints constraints in GetConstraints())
                DeactivateConstraintsWithInactiveParticles(constraints, particles);

        }

        public void RestoreRemovedParticles()
        {
            m_ActiveParticleCount = m_InitialActiveParticleCount;

            foreach (IObiConstraints constraints in GetConstraints())
                for (int j = 0; j < constraints.GetBatchCount(); ++j)
                    constraints.GetBatch(j).activeConstraintCount = constraints.GetBatch(j).initialActiveConstraintCount;
           
        }

        public virtual void GenerateTethers(bool[] selected) { }
        public virtual void ClearTethers() { }

        protected abstract IEnumerator Initialize();

    }
}