using System;
using UnityEngine;

namespace Obi{

	public class ObiSphereShapeTracker : ObiShapeTracker
	{

		public ObiSphereShapeTracker(ObiCollider source, SphereCollider collider)
        {
            this.source = source;
			this.collider = collider;
		}	

		public override bool UpdateIfNeeded()
        {

			SphereCollider sphere = collider as SphereCollider;

            // TODO: testing for changes here is not needed? all we do is set variables...
			//if (sphere != null && (sphere.radius != radius || sphere.center != center))
            {
                //radius = sphere.radius;
                //center = sphere.center;

                // retrieve collision world and index:
                var world = ObiColliderWorld.GetInstance();
                int index = source.Handle.index;

                // update collider:
                var shape = world.colliderShapes[index];
                shape.type = ColliderShape.ShapeType.Sphere;
                shape.phase = source.Phase;
                shape.flags = sphere.isTrigger ? 1 : 0;
                shape.rigidbodyIndex = source.Rigidbody != null ? source.Rigidbody.handle.index : -1;
                shape.materialIndex = source.CollisionMaterial != null ? source.CollisionMaterial.handle.index : -1;
                shape.contactOffset = sphere.contactOffset + source.Thickness;
                shape.center = sphere.center;
                shape.size = Vector3.one * sphere.radius;
                world.colliderShapes[index] = shape;

                // update bounds:
                var aabb = world.colliderAabbs[index];
                aabb.FromBounds(sphere.bounds, shape.contactOffset);
                world.colliderAabbs[index] = aabb;

                // update transform:
                var trfm = world.colliderTransforms[index];
                trfm.FromTransform(sphere.transform);
                world.colliderTransforms[index] = trfm;


               /*var shape = source.colliderWorld.colliderShapes[source.shapeHandle.index];

                // update the transform
                shape.Set(collider as Collider, source.Phase, source.Thickness);

                // update the shape:
                shape.SetSphere(sphere.center, sphere.radius);

                source.colliderWorld.colliderShapes[source.shapeHandle.index] = shape;*/

                //adaptor.Set(center, radius);
                //Oni.UpdateShape(oniShape,ref adaptor);
                return true;
			}
			//return false;
		}

	}
}

