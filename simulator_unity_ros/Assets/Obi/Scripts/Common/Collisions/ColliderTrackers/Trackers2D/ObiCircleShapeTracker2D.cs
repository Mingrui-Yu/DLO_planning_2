using System;
using UnityEngine;

namespace Obi{

	public class ObiCircleShapeTracker2D : ObiShapeTracker
	{

		public ObiCircleShapeTracker2D(ObiCollider2D source, CircleCollider2D collider)
        {
            this.source = source;
			this.collider = collider;
		}	

		public override bool UpdateIfNeeded ()
        {

			CircleCollider2D sphere = collider as CircleCollider2D;

            // retrieve collision world and index:
            var world = ObiColliderWorld.GetInstance();
            int index = source.Handle.index;

            // update collider:
            var shape = world.colliderShapes[index];
            shape.is2D = 1;
            shape.type = ColliderShape.ShapeType.Sphere;
            shape.phase = source.Phase;
            shape.flags = sphere.isTrigger ? 1 : 0;
            shape.rigidbodyIndex = source.Rigidbody != null ? source.Rigidbody.handle.index : -1;
            shape.materialIndex = source.CollisionMaterial != null ? source.CollisionMaterial.handle.index : -1;
            shape.contactOffset = source.Thickness;
            shape.center = sphere.offset;
            shape.size = Vector3.one * sphere.radius;
            world.colliderShapes[index] = shape;

            // update bounds:
            var aabb = world.colliderAabbs[index];
            aabb.FromBounds(sphere.bounds, shape.contactOffset, true);
            world.colliderAabbs[index] = aabb;

            // update transform:
            var trfm = world.colliderTransforms[index];
            trfm.FromTransform(sphere.transform,true);
            world.colliderTransforms[index] = trfm;

            return true;
        }

	}
}

