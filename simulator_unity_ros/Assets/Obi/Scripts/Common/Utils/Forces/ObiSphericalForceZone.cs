using UnityEngine;
using System;

namespace Obi
{
	public class ObiSphericalForceZone : ObiExternalForce
	{

		public float radius = 5;
		public bool radial = true;

		public override void ApplyForcesToActor(ObiActor actor)
        {

			float sqrRadius = radius * radius;
			float finalIntensity = intensity + GetTurbulence(turbulence);

			Matrix4x4 l2sTransform = actor.solver.transform.worldToLocalMatrix * transform.localToWorldMatrix;

			Vector4 center = l2sTransform.MultiplyPoint3x4(Vector4.zero);
			Vector4 forward = l2sTransform.MultiplyVector(Vector3.forward);

			// Calculate force intensity for each actor particle:
            for (int i = 0; i < actor.activeParticleCount; ++i){

				Vector4 distanceVector = actor.solver.positions[actor.solverIndices[i]] - center;

				float sqrMag = distanceVector.sqrMagnitude;
				float falloff = Mathf.Clamp01((sqrRadius - sqrMag) / sqrRadius);

				Vector4 force;
				if (radial)
					force = distanceVector/(Mathf.Sqrt(sqrMag) + float.Epsilon) * falloff * finalIntensity;
				else
					force = forward * falloff * finalIntensity;

				if (actor.usesCustomExternalForces)
					actor.solver.wind[actor.solverIndices[i]] += force;
				else
					actor.solver.externalForces[actor.solverIndices[i]] += force;	
			}			

		}

		public void OnDrawGizmosSelected()
        {
			Gizmos.matrix = transform.localToWorldMatrix;
			Gizmos.color = new Color(0,0.7f,1,1);
			Gizmos.DrawWireSphere(Vector3.zero,radius);

			float turb = GetTurbulence(1);

			if (!radial){
				ObiUtils.DrawArrowGizmo(radius + turb,radius*0.2f,radius*0.3f,radius*0.2f);
			}else{
				Gizmos.DrawLine(new Vector3(0,0,-radius*0.5f)*turb,new Vector3(0,0,radius*0.5f)*turb);
				Gizmos.DrawLine(new Vector3(0,-radius*0.5f,0)*turb,new Vector3(0,radius*0.5f,0)*turb);
				Gizmos.DrawLine(new Vector3(-radius*0.5f,0,0)*turb,new Vector3(radius*0.5f,0,0)*turb);
			}
		}
	}
}

