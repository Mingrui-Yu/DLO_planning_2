#if (OBI_ONI_SUPPORTED)
using UnityEngine;
using System;
using System.Collections;
using System.Runtime.InteropServices;

namespace Obi
{
    public class OniSolverImpl : ISolverImpl
    {
        private IntPtr m_OniSolver;

        // Per-type constraints array:
        IOniConstraintsImpl[] constraints;

        public IntPtr oniSolver
        {
            get { return m_OniSolver; }
        }

        public OniSolverImpl(IntPtr solver)
        {
            m_OniSolver = solver;

            constraints = new IOniConstraintsImpl[Oni.ConstraintTypeCount];
            constraints[(int)Oni.ConstraintType.Tether] = new OniTetherConstraintsImpl(this);
            constraints[(int)Oni.ConstraintType.Volume] = new OniVolumeConstraintsImpl(this);
            constraints[(int)Oni.ConstraintType.Chain] = new OniChainConstraintsImpl(this);
            constraints[(int)Oni.ConstraintType.Bending] = new OniBendConstraintsImpl(this);
            constraints[(int)Oni.ConstraintType.Distance] = new OniDistanceConstraintsImpl(this);
            constraints[(int)Oni.ConstraintType.ShapeMatching] = new OniShapeMatchingConstraintsImpl(this);
            constraints[(int)Oni.ConstraintType.BendTwist] = new OniBendTwistConstraintsImpl(this);
            constraints[(int)Oni.ConstraintType.StretchShear] = new OniStretchShearConstraintsImpl(this);
            constraints[(int)Oni.ConstraintType.Pin] = new OniPinConstraintsImpl(this);
            constraints[(int)Oni.ConstraintType.Skin] = new OniSkinConstraintsImpl(this);
            constraints[(int)Oni.ConstraintType.Aerodynamics] = new OniAerodynamicConstraintsImpl(this);
            constraints[(int)Oni.ConstraintType.Stitch] = new OniStitchConstraintsImpl(this);
        }

        public void Destroy()
        {
            Oni.DestroySolver(m_OniSolver);
            m_OniSolver = IntPtr.Zero;
        }

        public void InitializeFrame(Vector4 translation, Vector4 scale, Quaternion rotation)
        {
            Oni.InitializeFrame(oniSolver, ref translation, ref scale, ref rotation);
        }

        public void UpdateFrame(Vector4 translation, Vector4 scale, Quaternion rotation, float deltaTime)
        {
            Oni.UpdateFrame(oniSolver, ref translation, ref scale, ref rotation, deltaTime);
        }

        public void ApplyFrame(float worldLinearInertiaScale, float worldAngularInertiaScale, float deltaTime)
        {
            Oni.ApplyFrame(oniSolver, 0, 0, worldLinearInertiaScale, worldAngularInertiaScale, deltaTime);
        }

        public int GetDeformableTriangleCount()
        {
            return Oni.GetDeformableTriangleCount(m_OniSolver);
        }
        public void SetDeformableTriangles(int[] indices, int num, int destOffset)
        {
            Oni.SetDeformableTriangles(m_OniSolver, indices, num, destOffset);
        }
        public int RemoveDeformableTriangles(int num, int sourceOffset)
        {
            return Oni.RemoveDeformableTriangles(m_OniSolver, num, sourceOffset);
        }

        public void ParticleCountChanged(ObiSolver solver)
        {
            Oni.SetParticlePositions(m_OniSolver, solver.positions.GetIntPtr());
            Oni.SetParticlePreviousPositions(m_OniSolver, solver.prevPositions.GetIntPtr());
            Oni.SetRestPositions(m_OniSolver, solver.restPositions.GetIntPtr());
            Oni.SetParticleOrientations(m_OniSolver, solver.orientations.GetIntPtr());
            Oni.SetParticlePreviousOrientations(m_OniSolver, solver.prevOrientations.GetIntPtr());
            Oni.SetRestOrientations(m_OniSolver, solver.restOrientations.GetIntPtr());
            Oni.SetParticleVelocities(m_OniSolver, solver.velocities.GetIntPtr());
            Oni.SetParticleAngularVelocities(m_OniSolver, solver.angularVelocities.GetIntPtr());
            Oni.SetParticleInverseMasses(m_OniSolver, solver.invMasses.GetIntPtr());
            Oni.SetParticleInverseRotationalMasses(m_OniSolver, solver.invRotationalMasses.GetIntPtr());
            Oni.SetParticlePrincipalRadii(m_OniSolver, solver.principalRadii.GetIntPtr());
            Oni.SetParticleCollisionMaterials(m_OniSolver, solver.collisionMaterials.GetIntPtr());
            Oni.SetParticlePhases(m_OniSolver, solver.phases.GetIntPtr());
            Oni.SetRenderableParticlePositions(m_OniSolver, solver.renderablePositions.GetIntPtr());
            Oni.SetRenderableParticleOrientations(m_OniSolver, solver.renderableOrientations.GetIntPtr());
            Oni.SetParticleAnisotropies(m_OniSolver, solver.anisotropies.GetIntPtr());
            Oni.SetParticleSmoothingRadii(m_OniSolver, solver.smoothingRadii.GetIntPtr());
            Oni.SetParticleBuoyancy(m_OniSolver, solver.buoyancies.GetIntPtr());
            Oni.SetParticleRestDensities(m_OniSolver, solver.restDensities.GetIntPtr());
            Oni.SetParticleViscosities(m_OniSolver, solver.viscosities.GetIntPtr());
            Oni.SetParticleSurfaceTension(m_OniSolver, solver.surfaceTension.GetIntPtr());
            Oni.SetParticleVorticityConfinement(m_OniSolver, solver.vortConfinement.GetIntPtr());
            Oni.SetParticleAtmosphericDragPressure(m_OniSolver, solver.atmosphericDrag.GetIntPtr(), solver.atmosphericPressure.GetIntPtr());
            Oni.SetParticleDiffusion(m_OniSolver, solver.diffusion.GetIntPtr());
            Oni.SetParticleVorticities(m_OniSolver, solver.vorticities.GetIntPtr());
            Oni.SetParticleFluidData(m_OniSolver, solver.fluidData.GetIntPtr());
            Oni.SetParticleUserData(m_OniSolver, solver.userData.GetIntPtr());
            Oni.SetParticleExternalForces(m_OniSolver, solver.externalForces.GetIntPtr());
            Oni.SetParticleExternalTorques(m_OniSolver, solver.externalTorques.GetIntPtr());
            Oni.SetParticleWinds(m_OniSolver, solver.wind.GetIntPtr());
            Oni.SetParticlePositionDeltas(m_OniSolver, solver.positionDeltas.GetIntPtr());
            Oni.SetParticleOrientationDeltas(m_OniSolver, solver.orientationDeltas.GetIntPtr());
            Oni.SetParticlePositionConstraintCounts(m_OniSolver, solver.positionConstraintCounts.GetIntPtr());
            Oni.SetParticleOrientationConstraintCounts(m_OniSolver, solver.orientationConstraintCounts.GetIntPtr());
            Oni.SetParticleNormals(m_OniSolver, solver.normals.GetIntPtr());
            Oni.SetParticleInverseInertiaTensors(m_OniSolver, solver.invInertiaTensors.GetIntPtr());
        }


        public void ParticleCapacityChanged(ObiSolver solver)
        {
            Oni.SetCapacity(m_OniSolver, solver.positions.capacity);
        }

        public void SetRigidbodyArrays(ObiSolver solver)
        {
            Oni.SetRigidbodyLinearDeltas(m_OniSolver, solver.rigidbodyLinearDeltas.GetIntPtr());
            Oni.SetRigidbodyAngularDeltas(m_OniSolver, solver.rigidbodyAngularDeltas.GetIntPtr());
        }

        public void SetActiveParticles(int[] indices, int num)
        {
            Oni.SetActiveParticles(oniSolver, indices, num);
        }

        public void ResetForces()
        {
            Oni.ResetForces(oniSolver);
        }

        public void GetBounds(ref Vector3 min, ref Vector3 max)
        {
            Oni.GetBounds(oniSolver, ref min, ref max);
        }

        public void SetParameters(Oni.SolverParameters parameters)
        {
            Oni.SetSolverParameters(m_OniSolver, ref parameters);
        }

        public int GetConstraintCount(Oni.ConstraintType type)
        {
            return Oni.GetConstraintCount(m_OniSolver, (int)type);
        }

        public void GetCollisionContacts(Oni.Contact[] contacts, int count)
        {
            Oni.GetCollisionContacts(m_OniSolver, contacts, count);
        }

        public void GetParticleCollisionContacts(Oni.Contact[] contacts, int count)
        {
            Oni.GetParticleCollisionContacts(m_OniSolver, contacts, count);
        }

        public void SetConstraintGroupParameters(Oni.ConstraintType type, ref Oni.ConstraintParameters parameters)
        {
            Oni.SetConstraintGroupParameters(m_OniSolver, (int)type, ref parameters);
        }

        public IConstraintsBatchImpl CreateConstraintsBatch(Oni.ConstraintType constraintType)
        {
            return constraints[(int)constraintType].CreateConstraintsBatch();
        }

        public void DestroyConstraintsBatch(IConstraintsBatchImpl batch)
        {
            if (batch != null)
                constraints[(int)batch.constraintType].RemoveBatch(batch);
        }

        public IObiJobHandle CollisionDetection(float stepTime)
        {
            Oni.RecalculateInertiaTensors(oniSolver);
            return new OniJobHandle(Oni.CollisionDetection(oniSolver, stepTime));
        }

        public IObiJobHandle Substep(float substepTime)
        {
            return new OniJobHandle(Oni.Step(oniSolver, substepTime));
        }

        public void ApplyInterpolation(ObiNativeVector4List startPositions, ObiNativeQuaternionList startOrientations, float stepTime, float unsimulatedTime)
        {
            Oni.ApplyPositionInterpolation(oniSolver, startPositions.GetIntPtr(), startOrientations.GetIntPtr(), stepTime, unsimulatedTime);
        }

        public void InterpolateDiffuseProperties(ObiNativeVector4List properties, ObiNativeVector4List diffusePositions, ObiNativeVector4List diffuseProperties, ObiNativeIntList neighbourCount, int diffuseCount)
        {
            Oni.InterpolateDiffuseParticles(oniSolver, properties.GetIntPtr(), diffusePositions.GetIntPtr(), diffuseProperties.GetIntPtr(), neighbourCount.GetIntPtr(), diffuseCount);
        }

        public int GetParticleGridSize()
        {
           return Oni.GetParticleGridSize(oniSolver);
        }
        public void GetParticleGrid(ObiNativeAabbList cells)
        {
            //Oni.GetParticleGrid(oniSolver, cells.GetIntPtr());
        }

    }
}
#endif