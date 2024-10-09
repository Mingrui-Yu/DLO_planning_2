using System.Collections;
using System.Collections.ObjectModel;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System;

namespace Obi
{
    [CreateAssetMenu(fileName = "rope blueprint", menuName = "Obi/Rope Blueprint", order = 140)]
    public class ObiRopeBlueprint : ObiRopeBlueprintBase
    {

        public int pooledParticles = 100; 

        public const float DEFAULT_PARTICLE_MASS = 0.1f;

        protected override IEnumerator Initialize()
        {
            if (path.ControlPointCount < 2)
            {
                ClearParticleGroups();
                path.InsertControlPoint(0, Vector3.left, Vector3.left * 0.25f, Vector3.right * 0.25f, Vector3.up, DEFAULT_PARTICLE_MASS, 1, 1, 1, Color.white,"control point");
                path.InsertControlPoint(1, Vector3.right, Vector3.left * 0.25f, Vector3.right * 0.25f, Vector3.up, DEFAULT_PARTICLE_MASS, 1, 1, 1, Color.white,"control point");
            }

            path.RecalculateLenght(Matrix4x4.identity, 0.00001f, 7);

            List<Vector3> particlePositions = new List<Vector3>();
            List<float> particleThicknesses = new List<float>();
            List<float> particleInvMasses = new List<float>();
            List<int> particlePhases = new List<int>();
            List<Color> particleColors = new List<Color>();

            // In case the path is open, add a first particle. In closed paths, the last particle is also the first one.
            if (!path.Closed)
            {
                particlePositions.Add(path.points.GetPositionAtMu(path.Closed,0));
                particleThicknesses.Add(path.thicknesses.GetAtMu(path.Closed,0));
                particleInvMasses.Add(ObiUtils.MassToInvMass(path.masses.GetAtMu(path.Closed,0)));
                particlePhases.Add(path.phases.GetAtMu(path.Closed,0));
                particleColors.Add(path.colors.GetAtMu(path.Closed,0));
            }

            // Create a particle group for the first control point:
            groups[0].particleIndices.Clear();
            groups[0].particleIndices.Add(0);

            ReadOnlyCollection<float> lengthTable = path.ArcLengthTable;
            int spans = path.GetSpanCount();

            for (int i = 0; i < spans; i++)
            {
                int firstArcLengthSample = i * (path.ArcLengthSamples + 1);
                int lastArcLengthSample = (i + 1) * (path.ArcLengthSamples + 1);

                float upToSpanLength = lengthTable[firstArcLengthSample];
                float spanLength = lengthTable[lastArcLengthSample] - upToSpanLength;

                int particlesInSpan = 1 + Mathf.FloorToInt(spanLength / thickness * resolution);
                float distance = spanLength / particlesInSpan;

                for (int j = 0; j < particlesInSpan; ++j)
                {
                    float mu = path.GetMuAtLenght(upToSpanLength + distance * (j + 1));
                    particlePositions.Add(path.points.GetPositionAtMu(path.Closed,mu));
                    particleThicknesses.Add(path.thicknesses.GetAtMu(path.Closed,mu));
                    particleInvMasses.Add(ObiUtils.MassToInvMass(path.masses.GetAtMu(path.Closed,mu)));
                    particlePhases.Add(path.phases.GetAtMu(path.Closed,mu));
                    particleColors.Add(path.colors.GetAtMu(path.Closed,mu));
                }

                // Create a particle group for each control point:
                if (!(path.Closed && i == spans - 1))
                {
                    groups[i + 1].particleIndices.Clear();
                    groups[i + 1].particleIndices.Add(particlePositions.Count - 1);
                }

                if (i % 100 == 0)
                    yield return new CoroutineJob.ProgressInfo("ObiRope: generating particles...", i / (float)spans);
            }

            m_ActiveParticleCount = particlePositions.Count;
            totalParticles = m_ActiveParticleCount + pooledParticles;

            int numSegments = m_ActiveParticleCount - (path.Closed ? 0 : 1);
            if (numSegments > 0)
                m_InterParticleDistance = path.Length / (float)numSegments;
            else
                m_InterParticleDistance = 0;

            positions = new Vector3[totalParticles];
            restPositions = new Vector4[totalParticles];
            velocities = new Vector3[totalParticles];
            invMasses = new float[totalParticles];
            principalRadii = new Vector3[totalParticles];
            phases = new int[totalParticles];
            colors = new Color[totalParticles];
            restLengths = new float[totalParticles];

            for (int i = 0; i < m_ActiveParticleCount; i++)
            {
                invMasses[i] = particleInvMasses[i];
                positions[i] = particlePositions[i];
                restPositions[i] = positions[i];
                restPositions[i][3] = 1; // activate rest position.
                principalRadii[i] = Vector3.one * particleThicknesses[i] * thickness;
                phases[i] = ObiUtils.MakePhase(particlePhases[i], 0);
                colors[i] = particleColors[i];

                if (i % 100 == 0)
                    yield return new CoroutineJob.ProgressInfo("ObiRope: generating particles...", i / (float)m_ActiveParticleCount);
            }

            //Create distance constraints for the total number of particles, but only activate for the used ones.
            IEnumerator dc = CreateDistanceConstraints();

            while (dc.MoveNext())
                yield return dc.Current;

            //Create bending constraints:
            IEnumerator bc = CreateBendingConstraints();

            while (bc.MoveNext())
                yield return bc.Current;

            // Recalculate rest length:
            m_RestLength = 0;
            foreach (float length in restLengths)
                m_RestLength += length;

        }


        protected virtual IEnumerator CreateDistanceConstraints()
        {
            distanceConstraintsData = new ObiDistanceConstraintsData();

            // Add two batches: for even and odd constraints:
            distanceConstraintsData.AddBatch(new ObiDistanceConstraintsBatch());
            distanceConstraintsData.AddBatch(new ObiDistanceConstraintsBatch());

            for (int i = 0; i < totalParticles - 1; i++)
            {
                var batch = distanceConstraintsData.batches[i % 2] as ObiDistanceConstraintsBatch;

                if (i < m_ActiveParticleCount - 1)
                {
                    Vector2Int indices = new Vector2Int(i, i + 1);
                    restLengths[i] = Vector3.Distance(positions[indices.x], positions[indices.y]);
                    batch.AddConstraint(indices, restLengths[i]);
                    batch.activeConstraintCount++;
                }
                else
                {
                    restLengths[i] = m_InterParticleDistance;
                    batch.AddConstraint(Vector2Int.zero, 0);
                }

                if (i % 500 == 0)
                    yield return new CoroutineJob.ProgressInfo("ObiRope: generating structural constraints...", i / (float)(totalParticles - 1));

            }

            // if the path is closed, add the last, loop closing constraint to a new batch to avoid sharing particles.
            if (path.Closed)
            {
                var loopClosingBatch = new ObiDistanceConstraintsBatch();
                distanceConstraintsData.AddBatch(loopClosingBatch);

                Vector2Int indices = new Vector2Int(m_ActiveParticleCount-1, 0);
                restLengths[m_ActiveParticleCount - 2] = Vector3.Distance(positions[indices.x], positions[indices.y]);
                loopClosingBatch.AddConstraint(indices, restLengths[m_ActiveParticleCount - 2]);
                loopClosingBatch.activeConstraintCount++;
            }

        }

        protected virtual IEnumerator CreateBendingConstraints()
        {
            bendConstraintsData = new ObiBendConstraintsData();

            // Add three batches:
            bendConstraintsData.AddBatch(new ObiBendConstraintsBatch());
            bendConstraintsData.AddBatch(new ObiBendConstraintsBatch());
            bendConstraintsData.AddBatch(new ObiBendConstraintsBatch());

            for (int i = 0; i < totalParticles-2; i++)
            {
                var batch = bendConstraintsData.batches[i % 3] as ObiBendConstraintsBatch;

                Vector3Int indices = new Vector3Int(i, i + 2, i + 1);
                float restBend = ObiUtils.RestBendingConstraint(restPositions[indices[0]], restPositions[indices[1]], restPositions[indices[2]]);
                batch.AddConstraint(indices, restBend);

                if (i < m_ActiveParticleCount - 2)
                    batch.activeConstraintCount++;

                if (i % 500 == 0)
                    yield return new CoroutineJob.ProgressInfo("ObiRope: generating structural constraints...", i / (float)(totalParticles - 2));

            }

            // if the path is closed, add the last, loop closing constraints to a new batch to avoid sharing particles.
            if (path.Closed)
            {
                var loopClosingBatch = new ObiBendConstraintsBatch();
                bendConstraintsData.AddBatch(loopClosingBatch);

                Vector3Int indices = new Vector3Int(m_ActiveParticleCount - 2, 0, m_ActiveParticleCount - 1);
                loopClosingBatch.AddConstraint(indices, 0);
                loopClosingBatch.activeConstraintCount++;

                var loopClosingBatch2 = new ObiBendConstraintsBatch();
                bendConstraintsData.AddBatch(loopClosingBatch2);

                indices = new Vector3Int(m_ActiveParticleCount - 1, 1, 0);
                loopClosingBatch2.AddConstraint(indices, 0);
                loopClosingBatch2.activeConstraintCount++;
            }
        }

     
    }
}