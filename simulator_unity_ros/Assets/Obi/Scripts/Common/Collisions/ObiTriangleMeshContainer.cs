using System;
using System.Collections.Generic;
using UnityEngine;

namespace Obi
{
    public struct Triangle : IBounded
    {
        public int i1;
        public int i2;
        public int i3;

        Aabb b;

        public Triangle(int i1, int i2, int i3, Vector3 v1, Vector3 v2, Vector3 v3)
        {
            this.i1 = i1;
            this.i2 = i2;
            this.i3 = i3;
            b = new Aabb(v1);
            b.Encapsulate(v2);
            b.Encapsulate(v3);
        }

        public Aabb GetBounds()
        {
            return b;
        }
    }

    public class ObiTriangleMeshHandle : ObiResourceHandle<Mesh>
    {
        public ObiTriangleMeshHandle(Mesh mesh, int index = -1) : base(index) { owner = mesh; }
    }

    public struct TriangleMeshHeader //we need to use the header in the backend, so it must be a struct.
    {
        public int firstNode;
        public int nodeCount;
        public int firstTriangle;
        public int triangleCount;
        public int firstVertex;
        public int vertexCount;

        public TriangleMeshHeader(int firstNode, int nodeCount, int firstTriangle, int triangleCount, int firstVertex, int vertexCount)
        {
            this.firstNode = firstNode;
            this.nodeCount = nodeCount;
            this.firstTriangle = firstTriangle;
            this.triangleCount = triangleCount;
            this.firstVertex = firstVertex;
            this.vertexCount = vertexCount;
        }
    }

    public class ObiTriangleMeshContainer
    {
        public Dictionary<Mesh,ObiTriangleMeshHandle> handles;  /**< dictionary indexed by mesh, so that we don't generate data for the same mesh multiple times.*/

        public ObiNativeTriangleMeshHeaderList headers; /**< One header per mesh.*/
        public ObiNativeBIHNodeList bihNodes;
        public ObiNativeTriangleList triangles;
        public ObiNativeVector3List vertices;

        public ObiTriangleMeshContainer()
        {
            handles = new Dictionary<Mesh, ObiTriangleMeshHandle>();
            headers = new ObiNativeTriangleMeshHeaderList();
            bihNodes = new ObiNativeBIHNodeList();
            triangles = new ObiNativeTriangleList();
            vertices = new ObiNativeVector3List();
        }

        public ObiTriangleMeshHandle GetOrCreateTriangleMesh(Mesh source)
        {
            ObiTriangleMeshHandle handle;

            if (!handles.TryGetValue(source, out handle))
            { 
                var sourceTris = source.triangles;
                var sourceVertices = source.vertices;

                // Build a bounding interval hierarchy from the triangles:
                IBounded[] t = new IBounded[sourceTris.Length/3];
                for (int i = 0; i < t.Length; ++i)
                {
                    int t1 = sourceTris[i * 3];
                    int t2 = sourceTris[i * 3 + 1];
                    int t3 = sourceTris[i * 3 + 2];
                    t[i] = new Triangle(t1,t2,t3, sourceVertices[t1], sourceVertices[t2], sourceVertices[t3]);
                }
                var sourceBih = BIH.Build(ref t);

                Triangle[] tris = Array.ConvertAll(t, x => (Triangle)x);

                handle = new ObiTriangleMeshHandle(source, headers.count);
                handles.Add(source, handle);
                headers.Add(new TriangleMeshHeader(bihNodes.count, sourceBih.Length, triangles.count, tris.Length, vertices.count, sourceVertices.Length));

                bihNodes.AddRange(sourceBih);
                triangles.AddRange(tris);
                vertices.AddRange(sourceVertices);
            }

            return handle;
        }

        public void DestroyTriangleMesh(ObiTriangleMeshHandle handle)
        {
            if (handle != null && handle.isValid && handle.index < handles.Count)
            {
                var header = headers[handle.index];

                // Update headers:
                for (int i = 0; i < headers.count; ++i)
                {
                    var h = headers[i];
                    if (h.firstTriangle > header.firstTriangle)
                    {
                        h.firstNode -= header.nodeCount;
                        h.firstTriangle -= header.triangleCount;
                        h.firstVertex -= header.vertexCount;
                        headers[i] = h;
                    }
                }

                // update handles:
                foreach (var pair in handles)
                {
                    if (pair.Value.index > handle.index)
                        pair.Value.index --;
                }

                // Remove nodes, triangles and vertices
                bihNodes.RemoveRange(header.firstNode, header.nodeCount);
                triangles.RemoveRange(header.firstTriangle, header.triangleCount);
                vertices.RemoveRange(header.firstVertex, header.vertexCount);

                // remove header:
                headers.RemoveAt(handle.index);

                // remove the mesh from the dictionary:
                handles.Remove(handle.owner);

                // Invalidate our handle:
                handle.Invalidate();
            }
        }

        public void Dispose()
        {
            if (headers != null)
                headers.Dispose();
            if (triangles != null)
                triangles.Dispose();
            if (vertices != null)
                vertices.Dispose();
            if (bihNodes != null)
                bihNodes.Dispose();
        }

    }
}
