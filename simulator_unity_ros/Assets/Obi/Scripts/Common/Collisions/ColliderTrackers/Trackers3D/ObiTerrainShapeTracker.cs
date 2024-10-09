using System;
using UnityEngine;
using System.Runtime.InteropServices;

namespace Obi{

	public class ObiTerrainShapeTracker : ObiShapeTracker
	{
		private bool heightmapDataHasChanged = false;
        ObiHeightFieldHandle handle;

        public ObiTerrainShapeTracker(ObiCollider source, TerrainCollider collider){

            this.source = source;
			this.collider = collider;
		}		

		public void UpdateHeightData()
        {
            ObiColliderWorld.GetInstance().DestroyHeightField(handle);
        }
	
		public override bool UpdateIfNeeded ()
        {

            TerrainCollider terrain = collider as TerrainCollider;

            // retrieve collision world and index:
            var world = ObiColliderWorld.GetInstance();
            int index = source.Handle.index;

            int resolution = terrain.terrainData.heightmapResolution;

            // get or create the heightfield:
            if (handle == null || !handle.isValid)
            {
                handle = world.GetOrCreateHeightField(terrain.terrainData);
                handle.Reference();
            }

            // update collider:
            var shape = world.colliderShapes[index];
            shape.type = ColliderShape.ShapeType.Heightmap;
            shape.phase = source.Phase;
            shape.flags = terrain.isTrigger ? 1 : 0;
            shape.rigidbodyIndex = source.Rigidbody != null ? source.Rigidbody.handle.index : -1;
            shape.materialIndex = source.CollisionMaterial != null ? source.CollisionMaterial.handle.index : -1;
            shape.contactOffset = terrain.contactOffset + source.Thickness;
            shape.dataIndex = handle.index;
            shape.size = terrain.terrainData.size;
            shape.center = new Vector4(resolution, resolution, resolution, resolution);
            world.colliderShapes[index] = shape;

            // update bounds:
            var aabb = world.colliderAabbs[index];
            aabb.FromBounds(terrain.bounds, shape.contactOffset);
            world.colliderAabbs[index] = aabb;

            // update transform:
            var trfm = world.colliderTransforms[index];
            trfm.FromTransform(terrain.transform);
            world.colliderTransforms[index] = trfm;

            return true;
        }

		public override void Destroy()
        {
			base.Destroy();

            if (handle != null && handle.Dereference())
                ObiColliderWorld.GetInstance().DestroyHeightField(handle);
        }
	}
}

