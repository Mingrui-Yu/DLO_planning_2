using System;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;

namespace Obi{

	public class ObiDistanceFieldShapeTracker : ObiShapeTracker
	{
		public ObiDistanceField distanceField;
        ObiDistanceFieldHandle handle;


		public ObiDistanceFieldShapeTracker(ObiCollider source, Component collider, ObiDistanceField distanceField){

            this.source = source;
            this.collider = collider;
            this.distanceField = distanceField;
		}

        /**
		 * Forces the tracker to update distance field data during the next call to UpdateIfNeeded().
		 */
        public void UpdateDistanceFieldData()
        {
            ObiColliderWorld.GetInstance().DestroyDistanceField(handle);
        }

        public override bool UpdateIfNeeded ()
        {

            bool trigger = false;
            if (collider is Collider) trigger = ((Collider)collider).isTrigger;
            else if (collider is Collider2D) trigger = ((Collider2D)collider).isTrigger;

            // retrieve collision world and index:
            var world = ObiColliderWorld.GetInstance();
            int index = source.Handle.index;

            if (handle == null || !handle.isValid)
            {
                handle = world.GetOrCreateDistanceField(distanceField);
                handle.Reference();
            }

            // update collider:
            var shape = world.colliderShapes[index];
            shape.type = ColliderShape.ShapeType.SignedDistanceField;
            shape.phase = source.Phase;
            shape.flags = trigger ? 1 : 0;
            shape.rigidbodyIndex = source.Rigidbody != null ? source.Rigidbody.handle.index : -1;
            shape.materialIndex = source.CollisionMaterial != null ? source.CollisionMaterial.handle.index : -1;
            shape.contactOffset = source.Thickness;
            shape.dataIndex = handle.index;
            world.colliderShapes[index] = shape;

            // update bounds:
            var aabb = world.colliderAabbs[index];
            aabb.FromBounds(distanceField.FieldBounds.Transform(source.transform.localToWorldMatrix), shape.contactOffset);
            world.colliderAabbs[index] = aabb;

            // update transform:
            var trfm = world.colliderTransforms[index];
            trfm.FromTransform(source.transform);
            world.colliderTransforms[index] = trfm;

            return true;
        }

        public override void Destroy()
        {
            base.Destroy();

            if (handle != null && handle.Dereference())
                ObiColliderWorld.GetInstance().DestroyDistanceField(handle);
        }

    }
}

