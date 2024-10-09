using System;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;

namespace Obi{

	public class ObiMeshShapeTracker : ObiShapeTracker
	{
        ObiTriangleMeshHandle handle;

		public ObiMeshShapeTracker(ObiCollider source, MeshCollider collider){

            this.source = source;
			this.collider = collider;
		}

        /**
		 * Forces the tracker to update mesh data during the next call to UpdateIfNeeded().
		 */
        public void UpdateMeshData()
        {
            ObiColliderWorld.GetInstance().DestroyTriangleMesh(handle);
        }
	
		public override bool UpdateIfNeeded ()
        {

            MeshCollider meshCollider = collider as MeshCollider;

            // retrieve collision world and index:
            var world = ObiColliderWorld.GetInstance();
            int index = source.Handle.index;

            // get or create the mesh:
            if (handle == null || !handle.isValid)
            {
                handle = world.GetOrCreateTriangleMesh(meshCollider.sharedMesh);
                handle.Reference();
            }

            // update collider:
            var shape = world.colliderShapes[index];
            shape.type = ColliderShape.ShapeType.TriangleMesh;
            shape.phase = source.Phase;
            shape.flags = meshCollider.isTrigger ? 1 : 0;
            shape.rigidbodyIndex = source.Rigidbody != null ? source.Rigidbody.handle.index : -1;
            shape.materialIndex = source.CollisionMaterial != null ? source.CollisionMaterial.handle.index : -1;
            shape.contactOffset = meshCollider.contactOffset + source.Thickness;
            shape.dataIndex = handle.index;
            world.colliderShapes[index] = shape;

            // update bounds:
            var aabb = world.colliderAabbs[index];
            aabb.FromBounds(meshCollider.bounds, shape.contactOffset);
            world.colliderAabbs[index] = aabb;

            // update transform:
            var trfm = world.colliderTransforms[index];
            trfm.FromTransform(meshCollider.transform);
            world.colliderTransforms[index] = trfm;

            return true;
        }

		public override void Destroy()
        {
			base.Destroy();

            if (handle != null && handle.Dereference())
                ObiColliderWorld.GetInstance().DestroyTriangleMesh(handle);
		}
	}
}

