using System;
using UnityEngine;

namespace Obi
{
    [RequireComponent(typeof(LineRenderer))]
    [RequireComponent(typeof(ObiParticlePicker))]
    public class ObiParticleDragger : MonoBehaviour
    {
        public float springStiffness = 500;
        public float springDamping = 50;
        public bool drawSpring = true;

        private LineRenderer lineRenderer;
        private ObiParticlePicker picker;
        private ObiParticlePicker.ParticlePickEventArgs pickArgs;

        void OnEnable()
        {
            lineRenderer = GetComponent<LineRenderer>();
            picker = GetComponent<ObiParticlePicker>();
            picker.OnParticlePicked.AddListener(Picker_OnParticleDragged);
            picker.OnParticleDragged.AddListener(Picker_OnParticleDragged);
            picker.OnParticleReleased.AddListener(Picker_OnParticleReleased);
        }

        void OnDisable()
        {
            picker.OnParticlePicked.RemoveListener(Picker_OnParticleDragged);
            picker.OnParticleDragged.RemoveListener(Picker_OnParticleDragged);
            picker.OnParticleReleased.RemoveListener(Picker_OnParticleReleased);
            lineRenderer.positionCount = 0;
        }

        void FixedUpdate()
        {
            ObiSolver solver = picker.solver;

            if (solver != null && pickArgs != null)
            {

                // Calculate picking position in solver space:
                Vector4 targetPosition = solver.transform.InverseTransformPoint(pickArgs.worldPosition);

                // Calculate effective inverse mass:
                float invMass = solver.invMasses[pickArgs.particleIndex];

                if (invMass > 0)
                {
                    // Calculate and apply spring force:
                    Vector4 position = solver.positions[pickArgs.particleIndex];
                    Vector4 velocity = solver.velocities[pickArgs.particleIndex];
                    solver.externalForces[pickArgs.particleIndex] = ((targetPosition - position) * springStiffness - velocity * springDamping) / invMass;


                    if (drawSpring)
                    {
                        lineRenderer.positionCount = 2;
                        lineRenderer.SetPosition(0, pickArgs.worldPosition);
                        lineRenderer.SetPosition(1, solver.transform.TransformPoint(position));
                    }
                    else
                    {
                        lineRenderer.positionCount = 0;
                    }
                }

            }
        }

        void Picker_OnParticleDragged(ObiParticlePicker.ParticlePickEventArgs e)
        {
            pickArgs = e;
        }

        void Picker_OnParticleReleased(ObiParticlePicker.ParticlePickEventArgs e)
        {
            pickArgs = null;
            lineRenderer.positionCount = 0;
        }

    }
}

