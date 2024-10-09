using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace Obi
{
    public class ObiResourceHandle<T> where T : class
    {
        public T owner = null;               /**< reference to the owner instance*/
        public int index = -1;               /**< index of this resource in the collision world.*/
        private int referenceCount = 0;      /**< amount of references to this handle. Can be used to clean up any associated resources after it reaches zero.*/

        public bool isValid
        {
            get { return index >= 0; }
        }

        public void Invalidate()
        {
            index = -1;
            referenceCount = 0;
        }

        public void Reference()
        {
            referenceCount++;
        }

        public bool Dereference()
        {
            return --referenceCount == 0;
        }

        public ObiResourceHandle(int index = -1)
        {
            this.index = index;
            owner = null;
        }
    }

    public class ObiColliderHandle : ObiResourceHandle<ObiColliderBase>
    {
        public ObiColliderHandle(int index = -1) : base(index) { }
    }
    public class ObiCollisionMaterialHandle : ObiResourceHandle<ObiCollisionMaterial>
    {
        public ObiCollisionMaterialHandle(int index = -1) : base(index) { }
    }
    public class ObiRigidbodyHandle : ObiResourceHandle<ObiRigidbodyBase>
    {
        public ObiRigidbodyHandle(int index = -1) : base(index) { }
    }

    [ExecuteInEditMode]
    public class ObiColliderWorld
    {
        [NonSerialized] public List<IColliderWorldImpl> implementations;

        [NonSerialized] public List<ObiColliderHandle> colliderHandles;           // list of collider handles, used by ObiCollider components to retrieve them.
        [NonSerialized] public ObiNativeColliderShapeList colliderShapes;         // list of collider shapes.
        [NonSerialized] public ObiNativeAabbList colliderAabbs;                   // list of collider bounds.
        [NonSerialized] public ObiNativeAffineTransformList colliderTransforms;   // list of collider transforms.

        [NonSerialized] public List<ObiCollisionMaterialHandle> materialHandles;
        [NonSerialized] public ObiNativeCollisionMaterialList collisionMaterials; // list of collision materials.

        [NonSerialized] public List<ObiRigidbodyHandle> rigidbodyHandles;         // list of rigidbody handles, used by ObiRigidbody components to retrieve them.
        [NonSerialized] public ObiNativeRigidbodyList rigidbodies;                // list of rigidbodies.

        [NonSerialized] public ObiTriangleMeshContainer triangleMeshContainer;
        [NonSerialized] public ObiEdgeMeshContainer edgeMeshContainer;
        [NonSerialized] public ObiDistanceFieldContainer distanceFieldContainer;
        [NonSerialized] public ObiHeightFieldContainer heightFieldContainer;

        private static ObiColliderWorld instance;

        public static ObiColliderWorld GetInstance()
        {
            if (instance == null)
            {
                instance = new ObiColliderWorld();
                instance.Initialize();
            }
            return instance;
        }

        private void Initialize()
        {
            // Allocate all lists:
            if (implementations == null)
                implementations = new List<IColliderWorldImpl>();

            if (colliderHandles == null)
                colliderHandles = new List<ObiColliderHandle>();
            if (colliderShapes == null)
                colliderShapes = new ObiNativeColliderShapeList();
            if (colliderAabbs == null)
                colliderAabbs = new ObiNativeAabbList();
            if (colliderTransforms == null)
                colliderTransforms = new ObiNativeAffineTransformList();

            if (materialHandles == null)
                materialHandles = new List<ObiCollisionMaterialHandle>();
            if (collisionMaterials == null)
                collisionMaterials = new ObiNativeCollisionMaterialList();

            if (rigidbodyHandles == null)
                rigidbodyHandles = new List<ObiRigidbodyHandle>();
            if (rigidbodies == null)
                rigidbodies = new ObiNativeRigidbodyList();

            if (triangleMeshContainer == null)
                triangleMeshContainer = new ObiTriangleMeshContainer();
            if (edgeMeshContainer == null)
                edgeMeshContainer = new ObiEdgeMeshContainer();
            if (distanceFieldContainer == null)
                distanceFieldContainer = new ObiDistanceFieldContainer();
            if (heightFieldContainer == null)
                heightFieldContainer = new ObiHeightFieldContainer();
        }

        private void Destroy()
        {
            for (int i = 0; i < implementations.Count; ++i)
            {
                implementations[i].SetColliders(colliderShapes, colliderAabbs, colliderTransforms, 0);
                implementations[i].UpdateWorld();
            }

            // Invalidate all handles:
            if (colliderHandles != null)
                foreach (var handle in colliderHandles)
                    handle.Invalidate();

            if (rigidbodyHandles != null)
                foreach (var handle in rigidbodyHandles)
                    handle.Invalidate();

            if (materialHandles != null)
                foreach (var handle in materialHandles)
                    handle.Invalidate();

            // Dispose of all lists:
            implementations = null;
            colliderHandles = null;
            rigidbodyHandles = null;
            materialHandles = null;

            if (colliderShapes != null)
                colliderShapes.Dispose();
            if (colliderAabbs != null)
                colliderAabbs.Dispose();
            if (colliderTransforms != null)
                colliderTransforms.Dispose();

            if (collisionMaterials != null)
                collisionMaterials.Dispose();

            if (rigidbodies != null)
                rigidbodies.Dispose();

            if (triangleMeshContainer != null)
                triangleMeshContainer.Dispose();
            if (edgeMeshContainer != null)
                edgeMeshContainer.Dispose();
            if (distanceFieldContainer != null)
                distanceFieldContainer.Dispose();
            if (heightFieldContainer != null)
                heightFieldContainer.Dispose();

            instance = null;
        }

        private void DestroyIfUnused()
        {
            // when there are no implementations and no colliders, the world gets destroyed.
            if (colliderHandles.Count == 0 && rigidbodyHandles.Count == 0 && materialHandles.Count == 0 && implementations.Count == 0)
                Destroy();
        }

        public void RegisterImplementation(IColliderWorldImpl impl)
        {
            if (!implementations.Contains(impl))
                implementations.Add(impl);
        }

        public void UnregisterImplementation(IColliderWorldImpl impl)
        {
            implementations.Remove(impl);
            DestroyIfUnused();
        }

        public ObiColliderHandle CreateCollider()
        {
            var handle = new ObiColliderHandle(colliderHandles.Count);
            colliderHandles.Add(handle);

            colliderShapes.Add(new ColliderShape());
            colliderAabbs.Add(new Aabb());
            colliderTransforms.Add(new AffineTransform());

            return handle;
        }

        public ObiRigidbodyHandle CreateRigidbody()
        {
            var handle = new ObiRigidbodyHandle(rigidbodyHandles.Count);
            rigidbodyHandles.Add(handle);

            rigidbodies.Add(new ColliderRigidbody());

            return handle;
        }

        public ObiCollisionMaterialHandle CreateCollisionMaterial()
        {
            var handle = new ObiCollisionMaterialHandle(materialHandles.Count);
            materialHandles.Add(handle);

            collisionMaterials.Add(new CollisionMaterial());

            return handle;
        }

        public ObiTriangleMeshHandle GetOrCreateTriangleMesh(Mesh mesh)
        {
            return triangleMeshContainer.GetOrCreateTriangleMesh(mesh);
        }

        public void DestroyTriangleMesh(ObiTriangleMeshHandle meshHandle)
        {
            triangleMeshContainer.DestroyTriangleMesh(meshHandle);
        }

        public ObiEdgeMeshHandle GetOrCreateEdgeMesh(EdgeCollider2D collider)
        {
            return edgeMeshContainer.GetOrCreateEdgeMesh(collider);
        }

        public void DestroyEdgeMesh(ObiEdgeMeshHandle meshHandle)
        {
            edgeMeshContainer.DestroyEdgeMesh(meshHandle);
        }

        public ObiDistanceFieldHandle GetOrCreateDistanceField(ObiDistanceField df)
        {
            return distanceFieldContainer.GetOrCreateDistanceField(df);
        }

        public void DestroyDistanceField(ObiDistanceFieldHandle dfHandle)
        {
            distanceFieldContainer.DestroyDistanceField(dfHandle);
        }

        public ObiHeightFieldHandle GetOrCreateHeightField(TerrainData hf)
        {
            return heightFieldContainer.GetOrCreateHeightField(hf);
        }

        public void DestroyHeightField(ObiHeightFieldHandle hfHandle)
        {
            heightFieldContainer.DestroyHeightField(hfHandle);
        }

        public void DestroyCollider(ObiColliderHandle handle)
        {
            if (colliderShapes != null && handle != null && handle.isValid && handle.index < colliderHandles.Count)
            {
                int index = handle.index;
                int lastIndex = colliderHandles.Count - 1;

                // swap all collider info:
                colliderHandles.Swap(index, lastIndex);
                colliderShapes.Swap(index, lastIndex);
                colliderAabbs.Swap(index, lastIndex);
                colliderTransforms.Swap(index, lastIndex);

                // update the index of the handle we swapped with:
                colliderHandles[index].index = index;

                // invalidate our handle:
                // (after updating the swapped one!
                // in case there's just one handle in the array,
                // we need to write -1 after 0)
                handle.Invalidate();

                // remove last index:
                colliderHandles.RemoveAt(lastIndex);
                colliderShapes.count--;
                colliderAabbs.count--;
                colliderTransforms.count--;

                DestroyIfUnused();
            }

        }

        public void DestroyRigidbody(ObiRigidbodyHandle handle)
        {
            if (rigidbodies != null && handle != null && handle.isValid && handle.index < rigidbodyHandles.Count)
            {
                int index = handle.index;
                int lastIndex = rigidbodyHandles.Count - 1;

                // swap all collider info:
                rigidbodyHandles.Swap(index, lastIndex);
                rigidbodies.Swap(index, lastIndex);

                // update the index of the handle we swapped with:
                rigidbodyHandles[index].index = index;

                // invalidate our handle:
                // (after updating the swapped one!
                // in case there's just one handle in the array,
                // we need to write -1 after 0)
                handle.Invalidate();

                // remove last index:
                rigidbodyHandles.RemoveAt(lastIndex);
                rigidbodies.count--;

                DestroyIfUnused();
            }

        }

        public void DestroyCollisionMaterial(ObiCollisionMaterialHandle handle)
        {
            if (collisionMaterials != null && handle != null && handle.isValid && handle.index < materialHandles.Count)
            {
                int index = handle.index;
                int lastIndex = materialHandles.Count - 1;

                // swap all collider info:
                materialHandles.Swap(index, lastIndex);
                collisionMaterials.Swap(index, lastIndex);

                // update the index of the handle we swapped with:
                materialHandles[index].index = index;

                // invalidate our handle:
                // (after updating the swapped one!
                // in case there's just one handle in the array,
                // we need to write -1 after 0)
                handle.Invalidate();

                // remove last index:
                materialHandles.RemoveAt(lastIndex);
                collisionMaterials.count--;

                DestroyIfUnused();
            }
        }


        public void UpdateWorld()
        {
            // update all colliders:
            for (int i = 0; i < colliderHandles.Count; ++i)
                colliderHandles[i].owner.UpdateIfNeeded();

            for (int i = 0; i < implementations.Count; ++i)
            {
                // set arrays:
                implementations[i].SetColliders(colliderShapes, colliderAabbs, colliderTransforms, colliderShapes.count);
                implementations[i].SetRigidbodies(rigidbodies);
                implementations[i].SetCollisionMaterials(collisionMaterials);
                implementations[i].SetTriangleMeshData(triangleMeshContainer.headers, triangleMeshContainer.bihNodes, triangleMeshContainer.triangles, triangleMeshContainer.vertices);
                implementations[i].SetEdgeMeshData(edgeMeshContainer.headers, edgeMeshContainer.bihNodes, edgeMeshContainer.edges, edgeMeshContainer.vertices);
                implementations[i].SetDistanceFieldData(distanceFieldContainer.headers, distanceFieldContainer.dfNodes);
                implementations[i].SetHeightFieldData(heightFieldContainer.headers, heightFieldContainer.samples);

                // update world implementation:
                implementations[i].UpdateWorld();
            }
        }

        public void UpdateRigidbodies(List<ObiSolver> solvers, float stepTime)
        {
            // reset all solver's delta buffers to zero:
            foreach (ObiSolver solver in solvers)
            {
                if (solver != null)
                {
                    solver.EnsureRigidbodyArraysCapacity(rigidbodyHandles.Count);
                    solver.rigidbodyLinearDeltas.WipeToZero();
                    solver.rigidbodyAngularDeltas.WipeToZero();
                }
            }

            for (int i = 0; i < rigidbodyHandles.Count; ++i)
                rigidbodyHandles[i].owner.UpdateIfNeeded(stepTime);
        }

        public void UpdateRigidbodyVelocities(List<ObiSolver> solvers)
        {
            int count = 0;
            foreach (ObiSolver solver in solvers)
                if (solver != null) count++;

            if (count > 0)
            {
                // we want to average the deltas applied by all solvers, so calculate 1/solverCount.
                float rcpCount = 1.0f / count;

                for (int i = 0; i < rigidbodyHandles.Count; ++i)
                {
                    Vector4 linearDelta = Vector4.zero;
                    Vector4 angularDelta = Vector4.zero;

                    foreach (ObiSolver solver in solvers)
                    {
                        if (solver != null)
                        {
                            linearDelta += solver.rigidbodyLinearDeltas[i] * rcpCount;
                            angularDelta += solver.rigidbodyAngularDeltas[i] * rcpCount;
                        }
                    }

                    // update rigidbody velocities
                    rigidbodyHandles[i].owner.UpdateVelocities(linearDelta, angularDelta);
                }
            }
        }

    }
}
