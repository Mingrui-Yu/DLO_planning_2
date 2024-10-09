using System;
using UnityEngine;

namespace Obi{

	public class ObiBoxShapeTracker2D : ObiShapeTracker
	{
		public ObiBoxShapeTracker2D(ObiCollider2D source, BoxCollider2D collider){
            this.source = source;
			this.collider = collider;
		}		
	
		public override bool UpdateIfNeeded (){

			BoxCollider2D box = collider as BoxCollider2D;

            var world = ObiColliderWorld.GetInstance();
            int index = source.Handle.index;

            // update collider:
            var shape = world.colliderShapes[index];
            shape.is2D = 1;
            shape.type = ColliderShape.ShapeType.Box;
            shape.phase = source.Phase;
            shape.flags = box.isTrigger ? 1 : 0;
            shape.rigidbodyIndex = source.Rigidbody != null ? source.Rigidbody.handle.index : -1;
            shape.materialIndex = source.CollisionMaterial != null ? source.CollisionMaterial.handle.index : -1;
            shape.contactOffset = source.Thickness;
            shape.center = box.offset;
            shape.size = box.size;
            world.colliderShapes[index] = shape;

            // update bounds:
            var aabb = world.colliderAabbs[index];
            aabb.FromBounds(box.bounds, shape.contactOffset, true);
            world.colliderAabbs[index] = aabb;

            // update transform:
            var trfm = world.colliderTransforms[index];
            trfm.FromTransform(box.transform, true);
            world.colliderTransforms[index] = trfm;
            return false;
		}

	}
}

