using UnityEngine;
using System;
using System.Collections;
using System.Runtime.InteropServices;
using Obi;

/**
 * Interface for the Oni particle physics library.
 */
public static class Oni
{

    public const int ConstraintTypeCount = 17;

    public enum ConstraintType
    {
        Tether = 0,
        Volume = 1,
        Chain = 2,
        Bending = 3,
        Distance = 4,
        ShapeMatching = 5,
        BendTwist = 6,
        StretchShear = 7,
        Pin = 8,
        ParticleCollision = 9,
        Density = 10,
        Collision = 11,
        Skin = 12,
        Aerodynamics = 13,
        Stitch = 14,
        ParticleFriction = 15,
        Friction = 16
    };

    [Flags]
    public enum ParticleFlags
    {
        GroupMask = 0x00ffffff,
        SelfCollide = 1 << 24,
        Fluid = 1 << 25,
        OneSided = 1 << 26
    }

    public enum ShapeType
    {
        Sphere = 0,
        Box = 1,
        Capsule = 2,
        Heightmap = 3,
        TriangleMesh = 4,
        EdgeMesh = 5,
        SignedDistanceField = 6
    }

    public enum MaterialCombineMode
    {
        Average = 0,
        Minimum = 1,
        Multiply = 2,
        Maximum = 3
    }

    public enum ProfileMask : uint
    {
        ThreadIdMask = 0xffff0000,
        TypeMask = 0x000000ff,
        StackLevelMask = 0x0000ff00
    }

    public struct ProfileInfo
    {
        public double start;
        public double end;
        public uint info;
        public int pad;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
        public string name;
    }

    public struct GridCell
    {
        public Vector3 center;
        public Vector3 size;
        public int count;
    }

    [Serializable]
    public struct SolverParameters
    {

        public enum Interpolation
        {
            None,
            Interpolate,
        };

        public enum Mode
        {
            Mode3D,
            Mode2D,
        };

        [Tooltip("In 2D mode, particles are simulated on the XY plane only. For use in conjunction with Unity's 2D mode.")]
        public Mode mode;

        [Tooltip("Same as Rigidbody.interpolation. Set to INTERPOLATE for cloth that is applied on a main character or closely followed by a camera. NONE for everything else.")]
        public Interpolation interpolation;

        public Vector3 gravity;

        [Tooltip("Percentage of velocity lost per second, between 0% (0) and 100% (1).")]
        [Range(0, 1)]
        public float damping;

        [Tooltip("Percentage of shock propagation applied to particle-particle collisions. Useful for particle stacking.")]
        [Range(0, 1)]
        public float shockPropagation;

        [Tooltip("Max ratio between a particle's longest and shortest axis. Use 1 for isotropic (completely round) particles.")]
        [Range(1, 5)]
        public float maxAnisotropy;

        [Tooltip("Maximum depenetration velocity applied to particles that start a frame inside an object. Low values ensure no 'explosive' collision resolution. Should be > 0 unless looking for non-physical effects.")]
        public float maxDepenetration;

        [Tooltip("Kinetic energy below which particle positions arent updated. Energy values are mass-normalized, so all particles in the solver have the same threshold.")]
        public float sleepThreshold;

        public SolverParameters(Interpolation interpolation, Vector4 gravity)
        {
            this.mode = Mode.Mode3D;
            this.gravity = gravity;
            this.interpolation = interpolation;
            damping = 0;
            shockPropagation = 0;
            maxAnisotropy = 3;
            maxDepenetration = 5;
            sleepThreshold = 0.0001f;
        }

    }

    [Serializable]
    public struct ConstraintParameters
    {

        public enum EvaluationOrder
        {
            Sequential,
            Parallel
        };

        [Tooltip("Order in which constraints are evaluated. SEQUENTIAL converges faster but is not very stable. PARALLEL is very stable but converges slowly, requiring more iterations to achieve the same result.")]
        public EvaluationOrder evaluationOrder;                             /**< Constraint evaluation order.*/

        [Tooltip("Number of relaxation iterations performed by the constraint solver. A low number of iterations will perform better, but be less accurate.")]
        public int iterations;                                              /**< Amount of solver iterations per step for this constraint group.*/

        [Tooltip("Over (or under if < 1) relaxation factor used. At 1, no overrelaxation is performed. At 2, constraints double their relaxation rate. High values reduce stability but improve convergence.")]
        [Range(0.1f, 2)]
        public float SORFactor;                                             /**< Sucessive over-relaxation factor for parallel evaluation order.*/

        [Tooltip("Whether this constraint group is solved or not.")]
        [MarshalAs(UnmanagedType.I1)]
        public bool enabled;

        public ConstraintParameters(bool enabled, EvaluationOrder order, int iterations)
        {
            this.enabled = enabled;
            this.iterations = iterations;
            this.evaluationOrder = order;
            this.SORFactor = 1;
        }

    }

    // In this particular case, size is forced to 128 bytes to ensure 16 byte memory alignment needed by Oni.
    [StructLayout(LayoutKind.Sequential, Size = 128)]
    public struct Contact
    {

        public Vector4 point; 		   /**< Speculative point of contact. */
        public Vector4 normal;         /**< Normal direction. */
        public Vector4 tangent;        /**< Tangent direction. */
        public Vector4 bitangent;	   /**< Bitangent direction. */

        public float distance;    /** distance between both colliding entities at the beginning of the timestep.*/

        public float normalImpulse;
        public float tangentImpulse;
        public float bitangentImpulse;
        public float stickImpulse;
        public float rollingFrictionImpulse;

        public int particle; /** particle index*/
        public int other;    /** particle or rigidbody index*/

    }

    public static GCHandle PinMemory(object data)
    {
        return GCHandle.Alloc(data, GCHandleType.Pinned);
    }

    public static void UnpinMemory(GCHandle handle)
    {
        if (handle.IsAllocated)
            handle.Free();
    }

#if (UNITY_IOS && !UNITY_EDITOR)
		const string LIBNAME = "__Internal";
#elif ((UNITY_ANDROID || UNITY_STANDALONE_LINUX) && !UNITY_EDITOR)
		const string LIBNAME = "Oni";
#else
    const string LIBNAME = "libOni";
#endif

// platform custom define (https://docs.unity3d.com/Manual/PlatformDependentCompilation.html)
#if (OBI_ONI_SUPPORTED)

    [DllImport(LIBNAME)]
    public static extern void UpdateColliderGrid();

    [DllImport(LIBNAME)]
    public static extern void SetColliders(IntPtr shapes, IntPtr bounds, IntPtr transforms, int count);

    [DllImport(LIBNAME)]
    public static extern void SetRigidbodies(IntPtr rigidbodies);

    [DllImport(LIBNAME)]
    public static extern void SetCollisionMaterials(IntPtr materials);

    [DllImport(LIBNAME)]
    public static extern void SetTriangleMeshData(IntPtr headers, IntPtr nodes, IntPtr triangles, IntPtr vertices);

    [DllImport(LIBNAME)]
    public static extern void SetEdgeMeshData(IntPtr headers, IntPtr nodes, IntPtr edges, IntPtr vertices);

    [DllImport(LIBNAME)]
    public static extern void SetDistanceFieldData(IntPtr headers, IntPtr nodes);

    [DllImport(LIBNAME)]
    public static extern void SetHeightFieldData(IntPtr headers, IntPtr samples);

    [DllImport(LIBNAME)]
    public static extern IntPtr CreateSolver(int capacity);

    [DllImport(LIBNAME)]
    public static extern void DestroySolver(IntPtr solver);

    [DllImport(LIBNAME)]
    public static extern void SetCapacity(IntPtr solver, int capacity);

    [DllImport(LIBNAME)]
    public static extern void InitializeFrame(IntPtr solver, ref Vector4 translation, ref Vector4 scale, ref Quaternion rotation);

    [DllImport(LIBNAME)]
    public static extern void UpdateFrame(IntPtr solver, ref Vector4 translation, ref Vector4 scale, ref Quaternion rotation, float dt);

    [DllImport(LIBNAME)]
    public static extern void ApplyFrame(IntPtr solver, float linearVelocityScale, float angularVelocityScale, float linearInertiaScale, float angularInertiaScale, float dt);

    [DllImport(LIBNAME)]
    public static extern void RecalculateInertiaTensors(IntPtr solver);

    [DllImport(LIBNAME)]
    public static extern void ResetForces(IntPtr solver);

    [DllImport(LIBNAME)]
    public static extern void SetRigidbodyLinearDeltas(IntPtr solver, IntPtr linearDeltas);

    [DllImport(LIBNAME)]
    public static extern void SetRigidbodyAngularDeltas(IntPtr solver, IntPtr angularDeltas);


    [DllImport(LIBNAME)]
    public static extern void GetBounds(IntPtr solver, ref Vector3 min, ref Vector3 max);

    [DllImport(LIBNAME)]
    public static extern int GetParticleGridSize(IntPtr solver);

    [DllImport(LIBNAME)]
    public static extern void GetParticleGrid(IntPtr solver, GridCell[] cells);

    [DllImport(LIBNAME)]
    public static extern void SetSolverParameters(IntPtr solver, ref SolverParameters parameters);

    [DllImport(LIBNAME)]
    public static extern void GetSolverParameters(IntPtr solver, ref SolverParameters parameters);

    [DllImport(LIBNAME)]
    public static extern int SetActiveParticles(IntPtr solver, int[] active, int num);

    [DllImport(LIBNAME)]
    public static extern IntPtr CollisionDetection(IntPtr solver, float delta_time);

    [DllImport(LIBNAME)]
    public static extern IntPtr Step(IntPtr solver, float delta_time);

    [DllImport(LIBNAME)]
    public static extern void ApplyPositionInterpolation(IntPtr solver, IntPtr draw_positions, IntPtr draw_orientations, float delta_seconds, float unsimulated_time);

    [DllImport(LIBNAME)]
    public static extern void UpdateSkeletalAnimation(IntPtr solver);

    [DllImport(LIBNAME)]
    public static extern int GetConstraintCount(IntPtr solver, int type);

    [DllImport(LIBNAME)]
    public static extern void SetRenderableParticlePositions(IntPtr solver, IntPtr positions);

    [DllImport(LIBNAME)]
    public static extern void SetParticlePhases(IntPtr solver, IntPtr phases);

    [DllImport(LIBNAME)]
    public static extern void SetParticleCollisionMaterials(IntPtr solver, IntPtr materialIndices);

    [DllImport(LIBNAME)]
    public static extern void SetParticlePositions(IntPtr solver, IntPtr positions);

    [DllImport(LIBNAME)]
    public static extern void SetParticlePreviousPositions(IntPtr solver, IntPtr prevPositions);

    [DllImport(LIBNAME)]
    public static extern void SetParticleOrientations(IntPtr solver, IntPtr orientations);

    [DllImport(LIBNAME)]
    public static extern void SetParticlePreviousOrientations(IntPtr solver, IntPtr prevOrientations);

    [DllImport(LIBNAME)]
    public static extern void SetRenderableParticleOrientations(IntPtr solver, IntPtr orientations);

    [DllImport(LIBNAME)]
    public static extern void SetParticleInverseMasses(IntPtr solver, IntPtr invMasses);

    [DllImport(LIBNAME)]
    public static extern void SetParticleInverseRotationalMasses(IntPtr solver, IntPtr invRotMasses);

    [DllImport(LIBNAME)]
    public static extern void SetParticlePrincipalRadii(IntPtr solver, IntPtr principalRadii);

    [DllImport(LIBNAME)]
    public static extern void SetParticleVelocities(IntPtr solver, IntPtr velocities);

    [DllImport(LIBNAME)]
    public static extern void SetParticleAngularVelocities(IntPtr solver, IntPtr angularVelocities);

    [DllImport(LIBNAME)]
    public static extern void SetParticleExternalForces(IntPtr solver, IntPtr forces);

    [DllImport(LIBNAME)]
    public static extern void SetParticleExternalTorques(IntPtr solver, IntPtr torques);

    [DllImport(LIBNAME)]
    public static extern void SetParticleWinds(IntPtr solver, IntPtr winds);

    [DllImport(LIBNAME)]
    public static extern void SetParticlePositionDeltas(IntPtr solver, IntPtr deltas);

    [DllImport(LIBNAME)]
    public static extern void SetParticleOrientationDeltas(IntPtr solver, IntPtr deltas);

    [DllImport(LIBNAME)]
    public static extern void SetParticlePositionConstraintCounts(IntPtr solver, IntPtr counts);

    [DllImport(LIBNAME)]
    public static extern void SetParticleOrientationConstraintCounts(IntPtr solver, IntPtr counts);

    [DllImport(LIBNAME)]
    public static extern void SetParticleNormals(IntPtr solver, IntPtr normals);

    [DllImport(LIBNAME)]
    public static extern void SetParticleInverseInertiaTensors(IntPtr solver, IntPtr tensors);


    [DllImport(LIBNAME)]
    public static extern void SetParticleSmoothingRadii(IntPtr solver, IntPtr radii);

    [DllImport(LIBNAME)]
    public static extern void SetParticleBuoyancy(IntPtr solver, IntPtr buoyancy);

    [DllImport(LIBNAME)]
    public static extern void SetParticleRestDensities(IntPtr solver, IntPtr rest_densities);

    [DllImport(LIBNAME)]
    public static extern void SetParticleViscosities(IntPtr solver, IntPtr viscosities);

    [DllImport(LIBNAME)]
    public static extern void SetParticleSurfaceTension(IntPtr solver, IntPtr surface_tension);

    [DllImport(LIBNAME)]
    public static extern void SetParticleVorticityConfinement(IntPtr solver, IntPtr vort_confinement);

    [DllImport(LIBNAME)]
    public static extern void SetParticleAtmosphericDragPressure(IntPtr solver, IntPtr atmospheric_drag, IntPtr atmospheric_pressure);

    [DllImport(LIBNAME)]
    public static extern void SetParticleDiffusion(IntPtr solver, IntPtr diffusion);



    [DllImport(LIBNAME)]
    public static extern void SetParticleVorticities(IntPtr solver, IntPtr vorticities);

    [DllImport(LIBNAME)]
    public static extern void SetParticleFluidData(IntPtr solver, IntPtr fluidData);

    [DllImport(LIBNAME)]
    public static extern void SetParticleUserData(IntPtr solver, IntPtr userData);

    [DllImport(LIBNAME)]
    public static extern void SetParticleAnisotropies(IntPtr solver, IntPtr anisotropies);

    [DllImport(LIBNAME)]
    public static extern int GetDeformableTriangleCount(IntPtr solver);

    [DllImport(LIBNAME)]
    public static extern void SetDeformableTriangles(IntPtr solver, int[] indices, int num, int destOffset);

    [DllImport(LIBNAME)]
    public static extern int RemoveDeformableTriangles(IntPtr solver, int num, int sourceOffset);

    [DllImport(LIBNAME)]
    public static extern void SetConstraintGroupParameters(IntPtr solver, int type, ref ConstraintParameters parameters);

    [DllImport(LIBNAME)]
    public static extern void GetConstraintGroupParameters(IntPtr solver, int type, ref ConstraintParameters parameters);

    [DllImport(LIBNAME)]
    public static extern void SetRestPositions(IntPtr solver, IntPtr restPositions);

    [DllImport(LIBNAME)]
    public static extern void SetRestOrientations(IntPtr solver, IntPtr restOrientations);

    [DllImport(LIBNAME)]
    public static extern IntPtr CreateBatch(int type);

    [DllImport(LIBNAME)]
    public static extern void DestroyBatch(IntPtr batch);

    [DllImport(LIBNAME)]
    public static extern IntPtr AddBatch(IntPtr solver, IntPtr batch);

    [DllImport(LIBNAME)]
    public static extern void RemoveBatch(IntPtr solver, IntPtr batch);

    [DllImport(LIBNAME)]
    public static extern bool EnableBatch(IntPtr batch, [MarshalAs(UnmanagedType.I1)]bool enabled);

    [DllImport(LIBNAME)]
    public static extern int GetBatchConstraintForces(IntPtr batch, float[] forces, int num, int destOffset);



    [DllImport(LIBNAME)]
    public static extern void SetBatchConstraintCount(IntPtr batch, int num);

    [DllImport(LIBNAME)]
    public static extern int GetBatchConstraintCount(IntPtr batch);

    [DllImport(LIBNAME)]
    public static extern void SetDistanceConstraints(IntPtr batch, IntPtr indices,
                                                                   IntPtr restLengths,
                                                                   IntPtr stiffnesses,
                                                                   IntPtr lambdas,
                                                                   int num);

    [DllImport(LIBNAME)]
    public static extern void SetBendingConstraints(IntPtr batch, IntPtr indices,
                                                                  IntPtr restBends,
                                                                  IntPtr bendingStiffnesses,
                                                                  IntPtr lambdas,
                                                                  int num);

    [DllImport(LIBNAME)]
    public static extern void SetSkinConstraints(IntPtr batch,
                                                 IntPtr indices,
                                                 IntPtr points,
                                                 IntPtr normals,
                                                 IntPtr radiiBackstops,
                                                 IntPtr stiffnesses,
                                                 IntPtr lambdas,
                                                 int num);

    [DllImport(LIBNAME)]
    public static extern void SetAerodynamicConstraints(IntPtr batch,
                                                        IntPtr particleIndices,
                                                        IntPtr aerodynamicCoeffs,
                                                        int num);

    [DllImport(LIBNAME)]
    public static extern void SetVolumeConstraints(IntPtr batch,
                                                   IntPtr triangleIndices,
                                                   IntPtr firstTriangle,
                                                   IntPtr numTriangles,
                                                   IntPtr restVolumes,
                                                   IntPtr pressureStiffnesses,
                                                   IntPtr lambdas,
                                                   int num);

    [DllImport(LIBNAME)]
    public static extern void SetShapeMatchingConstraints(IntPtr batch,
                                                          IntPtr shapeIndices,
                                                          IntPtr firstIndex,
                                                          IntPtr numIndices,
                                                          IntPtr explicitGroup,
                                                          IntPtr materialParameters,
                                                          IntPtr restComs,
                                                          IntPtr coms,
                                                          IntPtr orientations,
                                                          int num);

    [DllImport(LIBNAME)]
    public static extern void CalculateRestShapeMatching(IntPtr solver, IntPtr batch);


    [DllImport(LIBNAME)]
    public static extern void SetStretchShearConstraints(IntPtr batch,
                                                         IntPtr particleIndices,
                                                         IntPtr orientationIndices,
                                                         IntPtr restLengths,
                                                         IntPtr restOrientations,
                                                         IntPtr stiffnesses,
                                                         IntPtr lambdas,
                                                         int num);

    [DllImport(LIBNAME)]
    public static extern void SetBendTwistConstraints(IntPtr batch,
                                                      IntPtr orientationIndices,
                                                      IntPtr restDarboux,
                                                      IntPtr stiffnesses,
                                                      IntPtr lambdas,
                                                      int num);

    [DllImport(LIBNAME)]
    public static extern void SetTetherConstraints(IntPtr batch,
                                                   IntPtr indices,
                                                   IntPtr maxLenghtsScales,
                                                   IntPtr stiffnesses,
                                                   IntPtr lambdas,
                                                   int num);

    [DllImport(LIBNAME)]
    public static extern void SetPinConstraints(IntPtr batch,
                                                IntPtr indices,
                                                IntPtr pinOffsets,
                                                IntPtr restDarboux,
                                                IntPtr colliders,
                                                IntPtr stiffnesses,
                                                IntPtr lambdas,
                                                int num);

    [DllImport(LIBNAME)]
    public static extern void SetStitchConstraints(IntPtr batch,
                                                   IntPtr indices,
                                                   IntPtr stiffnesses,
                                                   IntPtr lambdas,
                                                   int num);

    [DllImport(LIBNAME)]
    public static extern void SetChainConstraints(IntPtr batch,
                                                  IntPtr indices,
                                                  IntPtr lengths,
                                                  IntPtr firstIndex,
                                                  IntPtr numIndex,
                                                  int num);

    [DllImport(LIBNAME)]
    public static extern void GetCollisionContacts(IntPtr solver, Contact[] contacts, int n);

    [DllImport(LIBNAME)]
    public static extern void GetParticleCollisionContacts(IntPtr solver, Contact[] contacts, int n);

    [DllImport(LIBNAME)]
    public static extern int InterpolateDiffuseParticles(IntPtr solver, IntPtr properties, IntPtr diffusePositions, IntPtr diffuseProperties, IntPtr neighbourCount, int n);

    [DllImport(LIBNAME)]
    public static extern int MakePhase(int group, ParticleFlags flags);

    [DllImport(LIBNAME)]
    public static extern int GetGroupFromPhase(int phase);

    [DllImport(LIBNAME)]
    public static extern int GetFlagsFromPhase(int phase);

    [DllImport(LIBNAME)]
    public static extern float BendingConstraintRest(float[] constraintCoordinates);

    [DllImport(LIBNAME)]
    public static extern void CompleteAll();

    [DllImport(LIBNAME)]
    public static extern void Complete(IntPtr task);

    [DllImport(LIBNAME)]
    public static extern IntPtr CreateEmpty();

    [DllImport(LIBNAME)]
    public static extern void Schedule(IntPtr task);

    [DllImport(LIBNAME)]
    public static extern void AddChild(IntPtr task, IntPtr child);

    [DllImport(LIBNAME)]
    public static extern int GetMaxSystemConcurrency();

    [DllImport(LIBNAME)]
    public static extern void ClearProfiler();

    [DllImport(LIBNAME)]
    public static extern void EnableProfiler([MarshalAs(UnmanagedType.I1)]bool cooked);

    [DllImport(LIBNAME)]
    public static extern void BeginSample(string name, byte type);

    [DllImport(LIBNAME)]
    public static extern void EndSample();

    [DllImport(LIBNAME)]
    public static extern int GetProfilingInfoCount();

    [DllImport(LIBNAME)]
    public static extern void GetProfilingInfo([Out] ProfileInfo[] info, int num);

#endif
}
