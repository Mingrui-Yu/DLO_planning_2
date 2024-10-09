using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

namespace Obi
{

    public class ObiParticlePicker : MonoBehaviour
    {

        public class ParticlePickEventArgs : EventArgs
        {

            public int particleIndex;
            public Vector3 worldPosition;

            public ParticlePickEventArgs(int particleIndex, Vector3 worldPosition)
            {
                this.particleIndex = particleIndex;
                this.worldPosition = worldPosition;
            }
        }

        [Serializable]
        public class ParticlePickUnityEvent : UnityEvent<ParticlePickEventArgs> { }

        public ObiSolver solver;
        public float radiusScale = 1;

        public ParticlePickUnityEvent OnParticlePicked;
        public ParticlePickUnityEvent OnParticleHeld;
        public ParticlePickUnityEvent OnParticleDragged;
        public ParticlePickUnityEvent OnParticleReleased;

        private Vector3 lastMousePos = Vector3.zero;
        private int pickedParticleIndex = -1;
        private float pickedParticleDepth = 0;

        void Awake()
        {
            lastMousePos = Input.mousePosition;
        }

        void LateUpdate()
        {

            if (solver != null)
            {

                // Click:
                if (Input.GetMouseButtonDown(0))
                {

                    pickedParticleIndex = -1;

                    Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);

                    float closestMu = float.MaxValue;
                    float closestDistance = float.MaxValue;

                    Matrix4x4 solver2World = solver.transform.localToWorldMatrix;

                    // Find the closest particle hit by the ray:
                    for (int i = 0; i < solver.renderablePositions.count; ++i)
                    {

                        Vector3 worldPos = solver2World.MultiplyPoint3x4(solver.renderablePositions[i]);

                        float mu;
                        Vector3 projected = ObiUtils.ProjectPointLine(worldPos, ray.origin, ray.origin + ray.direction, out mu, false);
                        float distanceToRay = Vector3.SqrMagnitude(worldPos - projected);

                        // Disregard particles behind the camera:
                        mu = Mathf.Max(0, mu);

                        float radius = solver.principalRadii[i][0] * radiusScale;

                        if (distanceToRay <= radius * radius && distanceToRay < closestDistance && mu < closestMu)
                        {
                            closestMu = mu;
                            closestDistance = distanceToRay;
                            pickedParticleIndex = i;
                        }
                    }

                    if (pickedParticleIndex >= 0)
                    {

                        pickedParticleDepth = Camera.main.transform.InverseTransformVector(solver2World.MultiplyPoint3x4(solver.renderablePositions[pickedParticleIndex]) - Camera.main.transform.position).z;

                        if (OnParticlePicked != null)
                        {
                            Vector3 worldPosition = Camera.main.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, pickedParticleDepth));
                            OnParticlePicked.Invoke(new ParticlePickEventArgs(pickedParticleIndex, worldPosition));
                        }
                    }

                }
                else if (pickedParticleIndex >= 0)
                {

                    // Drag:
                    Vector3 mouseDelta = Input.mousePosition - lastMousePos;
                    if (mouseDelta.magnitude > 0.01f && OnParticleDragged != null)
                    {

                        Vector3 worldPosition = Camera.main.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, pickedParticleDepth));
                        OnParticleDragged.Invoke(new ParticlePickEventArgs(pickedParticleIndex, worldPosition));

                    }
                    else if (OnParticleHeld != null)
                    {

                        Vector3 worldPosition = Camera.main.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, pickedParticleDepth));
                        OnParticleHeld.Invoke(new ParticlePickEventArgs(pickedParticleIndex, worldPosition));

                    }

                    // Release:				
                    if (Input.GetMouseButtonUp(0))
                    {

                        if (OnParticleReleased != null)
                        {
                            Vector3 worldPosition = Camera.main.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, pickedParticleDepth));
                            OnParticleReleased.Invoke(new ParticlePickEventArgs(pickedParticleIndex, worldPosition));
                        }

                        pickedParticleIndex = -1;

                    }
                }
            }

            lastMousePos = Input.mousePosition;
        }
    }
}
