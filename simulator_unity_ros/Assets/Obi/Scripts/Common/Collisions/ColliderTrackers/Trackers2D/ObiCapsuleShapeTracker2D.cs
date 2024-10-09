using System;
using UnityEngine;

namespace Obi{

	public class ObiCapsuleShapeTracker2D : ObiShapeTracker
	{
		public ObiCapsuleShapeTracker2D(ObiCollider2D source, CapsuleCollider2D collider)
        {
            this.source = source;
			this.collider = collider;
		}	
	
		public override bool UpdateIfNeeded ()
        {
			CapsuleCollider2D capsule = collider as CapsuleCollider2D;

            // retrieve collision world and index:
            var world = ObiColliderWorld.GetInstance();
            int index = source.Handle.index;

            // update collider:
            var shape = world.colliderShapes[index];
            shape.is2D = 1;
            shape.type = ColliderShape.ShapeType.Capsule;
            shape.phase = source.Phase;
            shape.flags = capsule.isTrigger ? 1 : 0;
            shape.rigidbodyIndex = source.Rigidbody != null ? source.Rigidbody.handle.index : -1;
            shape.materialIndex = source.CollisionMaterial != null ? source.CollisionMaterial.handle.index : -1;
            shape.contactOffset = source.Thickness;
            shape.center = capsule.offset;
            Vector2 size = capsule.size;
            shape.size = new Vector4((capsule.direction == CapsuleDirection2D.Horizontal ? size.y : size.x) * 0.5f,
                                      Mathf.Max(size.x, size.y),
                                      capsule.direction == CapsuleDirection2D.Horizontal ? 0 : 1, 0);
            world.colliderShapes[index] = shape;

            // update bounds:
            var aabb = world.colliderAabbs[index];
            aabb.FromBounds(capsule.bounds, shape.contactOffset,true);
            world.colliderAabbs[index] = aabb;

            // update transform:
            var trfm = world.colliderTransforms[index];
            trfm.FromTransform(capsule.transform,true);
            world.colliderTransforms[index] = trfm;

            return false;
		}

	}
}

