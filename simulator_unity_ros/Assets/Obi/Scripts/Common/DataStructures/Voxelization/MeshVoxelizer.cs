using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Obi
{

    /**
     * Helper class that voxelizes a mesh.
     */
    public class MeshVoxelizer
    {
        public enum Voxel
        {
            Inside = 0,
            Boundary = 1 << 0,
            Outside = 1 << 1,
        }

        public Mesh input;
        public float voxelSize;
        public Voxel[,,] voxels;

        private Vector3Int origin;

        public Vector3Int Origin
        {
            get { return origin; }
        }

        public MeshVoxelizer(Mesh input, float voxelSize)
        {
            this.input = input;
            this.voxelSize = voxelSize;
        }

        private Bounds GetTriangleBounds(Vector3 v1, Vector3 v2, Vector3 v3)
        {
            Bounds b = new Bounds(v1, Vector3.zero);
            b.Encapsulate(v2);
            b.Encapsulate(v3);
            return b;
        }

        private Vector3Int GetPointVoxel(Vector3 point)
        {
            return new Vector3Int(Mathf.FloorToInt(point.x / voxelSize),
                                  Mathf.FloorToInt(point.y / voxelSize),
                                  Mathf.FloorToInt(point.z / voxelSize));
        }

        private bool VoxelExists(Vector3Int coords)
        {
            coords -= origin;
            return coords.x >= 0 && coords.y >= 0 && coords.z >= 0 &&
                   coords.x < voxels.GetLength(0) &&
                   coords.y < voxels.GetLength(1) &&
                   coords.z < voxels.GetLength(2);
        }

        private void AppendOverlappingVoxels(Bounds bounds, Vector3 v1, Vector3 v2, Vector3 v3)
        {

            Vector3Int min = GetPointVoxel(bounds.min);
            Vector3Int max = GetPointVoxel(bounds.max);

            for (int x = min.x; x <= max.x; ++x)
                for (int y = min.y; y <= max.y; ++y)
                    for (int z = min.z; z <= max.z; ++z)
                    {

                        Bounds voxel = new Bounds(new Vector3(x + 0.5f, y + 0.5f, z + 0.5f) * voxelSize, Vector3.one * voxelSize);

                        if (IsIntersecting(voxel, v1, v2, v3))
                            voxels[x - origin.x, y - origin.y, z - origin.z] = Voxel.Boundary;

                    }
        }


        public void Voxelize(Vector3 scale)
        {
            voxelSize = Mathf.Max(0.001f, voxelSize);

            // Calculate min and max voxels:
            origin = GetPointVoxel(Vector3.Scale(scale,input.bounds.min)) - new Vector3Int(1, 1, 1);
            Vector3Int max = GetPointVoxel(Vector3.Scale(scale,input.bounds.max)) + new Vector3Int(1, 1, 1);

            // Allocate voxels array, and initialize them to "inside" the mesh:
            voxels = new Voxel[max.x - origin.x + 1, max.y - origin.y + 1, max.z - origin.z + 1];

            for (int x = 0; x < voxels.GetLength(0); ++x)
                for (int y = 0; y < voxels.GetLength(1); ++y)
                    for (int z = 0; z < voxels.GetLength(2); ++z)
                        voxels[x, y, z] = Voxel.Inside;

            // Get input triangles and vertices:
            int[] triIndices = input.triangles;
            Vector3[] vertices = input.vertices;

            // Generate surface voxels:
            for (int i = 0; i < triIndices.Length; i += 3)
            {

                Vector3 v1 = Vector3.Scale(vertices[triIndices[i]],scale);
                Vector3 v2 = Vector3.Scale(vertices[triIndices[i + 1]],scale);
                Vector3 v3 = Vector3.Scale(vertices[triIndices[i + 2]],scale);

                Bounds triBounds = GetTriangleBounds(v1, v2, v3);

                AppendOverlappingVoxels(triBounds, v1, v2, v3);

            }

            // Flood fill outside the mesh. This deals with multiple disjoint regions, and non-watertight models.
            FloodFill();
        }

        private void FloodFill()
        {
            Queue<Vector3Int> queue = new Queue<Vector3Int>();
            queue.Enqueue(new Vector3Int(0, 0, 0));

            voxels[0, 0, 0] = Voxel.Outside;

            while (queue.Count > 0)
            {
                Vector3Int i = queue.Dequeue();
                Vector3Int v;

                if (i.x < voxels.GetLength(0) - 1 && voxels[i.x + 1, i.y, i.z] == Voxel.Inside)
                {
                    v = new Vector3Int(i.x + 1, i.y, i.z);
                    voxels[v.x, v.y, v.z] = Voxel.Outside;
                    queue.Enqueue(v);
                }
                if (i.x > 0 && voxels[i.x - 1, i.y, i.z] == Voxel.Inside)
                {
                    v = new Vector3Int(i.x - 1, i.y, i.z);
                    voxels[v.x, v.y, v.z] = Voxel.Outside;
                    queue.Enqueue(v);
                }
                if (i.y < voxels.GetLength(1) - 1 && voxels[i.x, i.y + 1, i.z] == Voxel.Inside)
                {
                    v = new Vector3Int(i.x, i.y + 1, i.z);
                    voxels[v.x, v.y, v.z] = Voxel.Outside;
                    queue.Enqueue(v);
                }
                if (i.y > 0 && voxels[i.x, i.y - 1, i.z] == Voxel.Inside )
                {
                    v = new Vector3Int(i.x, i.y - 1, i.z);
                    voxels[v.x, v.y, v.z] = Voxel.Outside;
                    queue.Enqueue(v);
                }

                if (i.z < voxels.GetLength(2) - 1 && voxels[i.x, i.y, i.z + 1] == Voxel.Inside)
                {
                    v = new Vector3Int(i.x, i.y, i.z + 1);
                    voxels[v.x, v.y, v.z] = Voxel.Outside;
                    queue.Enqueue(v);
                }
                if (i.z > 0 && voxels[i.x, i.y, i.z - 1] == Voxel.Inside)
                {
                    v = new Vector3Int(i.x, i.y, i.z - 1);
                    voxels[v.x, v.y, v.z] = Voxel.Outside;
                    queue.Enqueue(v);
                }

            }
        }

        public static bool IsIntersecting(Bounds box, Vector3 v1, Vector3 v2, Vector3 v3)
        {

            Vector3[] triangleVertices = new Vector3[] { v1, v2, v3 };

            double triangleMin, triangleMax;
            double boxMin, boxMax;

            // Test the box normals (x-, y- and z-axes)
            var boxNormals = new Vector3[] {
                new Vector3(1,0,0),
                new Vector3(0,1,0),
                new Vector3(0,0,1)
            };

            for (int i = 0; i < 3; i++)
            {
                Project(triangleVertices, boxNormals[i], out triangleMin, out triangleMax);

                if (triangleMax < box.min[i] || triangleMin > box.max[i])
                    return false; // No intersection possible.
            }

            Vector3[] boxVertices = new Vector3[]{box.min,box.max,
                                              new Vector3(box.min.x,box.min.y,box.max.z),
                                              new Vector3(box.min.x,box.max.y,box.min.z),
                                              new Vector3(box.max.x,box.min.y,box.min.z),
                                              new Vector3(box.min.x,box.max.y,box.max.z),
                                              new Vector3(box.max.x,box.min.y,box.max.z),
                                              new Vector3(box.max.x,box.max.y,box.min.z)};

            Vector3 triangleNormal = Vector3.Cross(v2 - v1, v3 - v1);

            // Test the triangle normal
            double triangleOffset = Vector3.Dot(triangleNormal, v1);
            Project(boxVertices, triangleNormal, out boxMin, out boxMax);

            if (boxMax < triangleOffset || boxMin > triangleOffset)
                return false; // No intersection possible.

            // Test the nine edge cross-products
            Vector3[] triangleEdges = new Vector3[] {
                v1-v2,
                v2-v3,
                v3-v1,
            };

            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    // The box normals are the same as it's edge tangents
                    Vector3 axis = Vector3.Cross(triangleEdges[i], boxNormals[j]);
                    Project(boxVertices, axis, out boxMin, out boxMax);
                    Project(triangleVertices, axis, out triangleMin, out triangleMax);

                    if (boxMax < triangleMin || boxMin > triangleMax)
                        return false; // No intersection possible
                }

            // No separating axis found.
            return true;
        }

        static void Project(IEnumerable<Vector3> points, Vector3 axis, out double min, out double max)
        {
            min = double.PositiveInfinity;
            max = double.NegativeInfinity;
            foreach (var p in points)
            {
                double val = Vector3.Dot(axis, p);
                if (val < min) min = val;
                if (val > max) max = val;
            }
        }
    }
}