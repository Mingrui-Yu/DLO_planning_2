using UnityEngine;
using System.Collections;


namespace Obi
{
    [AddComponentMenu("Physics/Obi/Obi Rope Cursor", 882)]
    [RequireComponent(typeof(ObiRope))]
    public class ObiRopeCursor : MonoBehaviour
    {
        ObiRope rope;

        [Range(0, 1)]
        [HideInInspector] [SerializeField] private float m_CursorMu;

        [Range(0, 1)]
        [HideInInspector] [SerializeField] private float m_SourceMu;

        public bool direction = true;

        ObiStructuralElement m_CursorElement = null;
        private int m_SourceIndex = -1;

        public float cursorMu
        {
            set
            {
                m_CursorMu = value;
                UpdateCursor();
            }
            get { return m_CursorMu; }
        }

        public float sourceMu
        {
            set
            {
                m_SourceMu = value;
                UpdateSource();
            }
            get { return m_SourceMu; }
        }

        public ObiStructuralElement cursorElement
        {
            get
            {
                if (m_CursorElement == null)
                    UpdateCursor();
                return m_CursorElement;
            }
        }

        public int sourceParticleIndex
        {
            get
            {
                if (m_SourceIndex < 0)
                    UpdateSource();
                return m_SourceIndex;
            }
        }

        private void Awake()
        {
            rope = GetComponent<ObiRope>();

            rope.OnElementsGenerated += Actor_OnElementsGenerated;
            if (rope.elements != null && rope.elements.Count > 0)
                Actor_OnElementsGenerated(rope);
        }

        private void Actor_OnElementsGenerated(ObiActor actor)
        {
            UpdateCursor();
            UpdateSource();
        }

        private void OnDestroy()
        {
            rope.OnElementsGenerated -= Actor_OnElementsGenerated;
        }

        public void UpdateCursor()
        {
            rope = GetComponent<ObiRope>();
            m_CursorElement = null;
            if (rope.isLoaded)
            {
                float elmMu;
                m_CursorElement = rope.GetElementAt(cursorMu, out elmMu);
            }
        }

        public void UpdateSource()
        {
            rope = GetComponent<ObiRope>();
            m_SourceIndex = -1;
            if (rope.isLoaded)
            {
                float elmMu;
                var elm = rope.GetElementAt(sourceMu, out elmMu);
                if (elm != null && rope.solver != null)
                {
                    m_SourceIndex = elmMu < 0.5f ? elm.particle1 : elm.particle2;
                }
            }
        }

        private int AddParticleAt(int index)
        {
            // Copy data from the particle where we will insert new particles, to the particles we will insert:
            int targetIndex = rope.activeParticleCount;
            rope.CopyParticle(rope.solver.particleToActor[m_SourceIndex].indexInActor, targetIndex);

            // Move the new particle to the one at the place where we will insert it:
            rope.TeleportParticle(targetIndex, rope.solver.positions[rope.solverIndices[index]]);

            // Activate the particle:
            rope.ActivateParticle(targetIndex);
            return rope.solverIndices[targetIndex];
        }

        private void RemoveParticleAt(int index)
        {
            rope.DeactivateParticle(index);
        }

        public void ChangeLength(float newLength)
        {
            if (!rope.isLoaded)
                return;

            var solver = rope.solver;

            // clamp new length to sane limits:
            newLength = Mathf.Clamp(newLength, 0, (rope.sourceBlueprint.particleCount - 1) * rope.ropeBlueprint.interParticleDistance);

            // calculate the change in rope length:
            float lengthChange = newLength - rope.restLength;

            // remove:
            if (lengthChange < 0)
            {
                lengthChange = -lengthChange;

                while (lengthChange > m_CursorElement.restLength)
                {
                    lengthChange -= m_CursorElement.restLength;

                    int index = rope.elements.IndexOf(m_CursorElement);

                    if (index >= 0)
                    {
                        // positive direction:
                        if (direction)
                        {
                            RemoveParticleAt(solver.particleToActor[m_CursorElement.particle2].indexInActor);
                            rope.elements.RemoveAt(index);

                            if (index < rope.elements.Count)
                            {
                                if (rope.elements[index].particle1 == m_CursorElement.particle2)
                                    rope.elements[index].particle1 = m_CursorElement.particle1;

                                m_CursorElement = rope.elements[index];
                            }
                            else
                                m_CursorElement = rope.elements[Mathf.Max(0,index - 1)];
                        }
                        else // negative direction:
                        {
                            RemoveParticleAt(solver.particleToActor[m_CursorElement.particle1].indexInActor);
                            rope.elements.RemoveAt(index);

                            if (index > 0)
                            {
                                if (rope.elements[index - 1].particle2 == m_CursorElement.particle1)
                                    rope.elements[index - 1].particle2 = m_CursorElement.particle2;
                                m_CursorElement = rope.elements[index - 1];
                            }
                            else
                                m_CursorElement = rope.elements[0];
                        }
                    }
                }

                // the remaining length is subtracted from the current constraint:
                if (lengthChange > 0)
                    m_CursorElement.restLength = Mathf.Max(0, m_CursorElement.restLength - lengthChange);

            }
            // add
            else
            {
                float lengthDelta = Mathf.Min(lengthChange, Mathf.Max(0, rope.ropeBlueprint.interParticleDistance - m_CursorElement.restLength));

                // extend the current element, if possible:
                if (lengthDelta > 0)
                {
                    m_CursorElement.restLength += lengthDelta;
                    lengthChange -= lengthDelta;
                }

                // once the current element has been extended, see if we must add new elements, if there's enough particles left:
                while (rope.activeParticleCount < rope.sourceBlueprint.particleCount &&
                       m_CursorElement.restLength + lengthChange > rope.ropeBlueprint.interParticleDistance)
                {
                    // calculate added length:
                    lengthDelta = Mathf.Min(lengthChange, rope.ropeBlueprint.interParticleDistance);
                    lengthChange -= lengthDelta;

                    if (direction)
                    {
                        // add new particle:
                        int newParticleSolverIndex = AddParticleAt(solver.particleToActor[m_CursorElement.particle1].indexInActor);

                        // set position of the new particle:
                        solver.positions[newParticleSolverIndex] = solver.positions[m_CursorElement.particle1] +
                                                                        (solver.positions[m_CursorElement.particle2] - solver.positions[m_CursorElement.particle1]) * lengthDelta;

                        // insert a new element:
                        ObiStructuralElement newElement = new ObiStructuralElement();
                        newElement.restLength = lengthDelta;
                        newElement.particle1 = m_CursorElement.particle1;
                        newElement.particle2 = newParticleSolverIndex;
                        m_CursorElement.particle1 = newParticleSolverIndex;
                        int index = rope.elements.IndexOf(m_CursorElement);
                        rope.elements.Insert(index, newElement);

                        m_CursorElement = newElement;
                    }
                    else
                    {
                        // add new particle:
                        int newParticleSolverIndex = AddParticleAt(solver.particleToActor[m_CursorElement.particle2].indexInActor);

                        // set position of the new particle:
                        solver.positions[newParticleSolverIndex] = solver.positions[m_CursorElement.particle2] +
                                                                   (solver.positions[m_CursorElement.particle1] - solver.positions[m_CursorElement.particle2]) * lengthDelta;

                        // insert a new element:
                        ObiStructuralElement newElement = new ObiStructuralElement();
                        newElement.restLength = lengthDelta;
                        newElement.particle1 = newParticleSolverIndex;
                        newElement.particle2 = m_CursorElement.particle2;
                        m_CursorElement.particle2 = newParticleSolverIndex;
                        int index = rope.elements.IndexOf(m_CursorElement);
                        rope.elements.Insert(index + 1, newElement);

                        m_CursorElement = newElement;
                    }
                }

                // the remaining length is added to the current constraint:
                if (lengthChange > 0)
                    m_CursorElement.restLength += lengthChange;

            }

            // recalculate rest positions and length prior to constraints (bend constraints need rest positions):
            rope.RecalculateRestPositions();
            rope.RecalculateRestLength();

            // rebuild constraints:
            rope.RebuildConstraintsFromElements();
        }
    }
}