using System;
using UnityEngine;

namespace Obi{

	public class ObiBoxShapeTracker : ObiShapeTracker
	{

		public ObiBoxShapeTracker(ObiCollider source, BoxCollider collider)
        {
            this.source = source;
            this.collider = collider;
		}		
	
		public override bool UpdateIfNeeded (){

			BoxCollider box = collider as BoxCollider;

            /*if (box != null && (box.size != size || box.center != center)){
				size = box.size;
				center = box.center;
				adaptor.Set(center, size);
				Oni.UpdateShape(oniShape,ref adaptor);
				return true;
			}*/

            // retrieve collision world and index:
            var world = ObiColliderWorld.GetInstance();
            int index = source.Handle.index;

            // update collider:
            var shape = world.colliderShapes[index];
            shape.type = ColliderShape.ShapeType.Box;
            shape.phase = source.Phase;
            shape.flags = box.isTrigger ? 1 : 0;
            shape.rigidbodyIndex = source.Rigidbody        != null ? source.Rigidbody.handle.index         : -1;
            shape.materialIndex = source.CollisionMaterial != null ? source.CollisionMaterial.handle.index : -1;
            shape.contactOffset = box.contactOffset + source.Thickness;
            shape.center = box.center;
            shape.size = box.size;
            world.colliderShapes[index] = shape;

            // update bounds:
            var aabb = world.colliderAabbs[index];
            aabb.FromBounds(box.bounds, shape.contactOffset);
            world.colliderAabbs[index] = aabb;

            // update transform:
            var trfm = world.colliderTransforms[index];
            trfm.FromTransform(box.transform);
            world.colliderTransforms[index] = trfm;

            return true;
		}

	}
}

