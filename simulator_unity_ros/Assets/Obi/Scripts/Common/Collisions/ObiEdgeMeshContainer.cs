using System;
using System.Collections.Generic;
using UnityEngine;

namespace Obi
{
    public struct Edge : IBounded
    {
        public int i1;
        public int i2;

        Aabb b;

        public Edge(int i1, int i2, Vector2 v1, Vector2 v2)
        {
            this.i1 = i1;
            this.i2 = i2;
            b = new Aabb(v1);
            b.Encapsulate(v2);
        }

        public Aabb GetBounds()
        {
            return b;
        }
    }

    public class ObiEdgeMeshHandle : ObiResourceHandle<EdgeCollider2D>
    {
        public ObiEdgeMeshHandle(EdgeCollider2D collider, int index = -1) : base(index) { owner = collider; }
    }

    public struct EdgeMeshHeader
    {
        public int firstNode;
        public int nodeCount;
        public int firstEdge;
        public int edgeCount;
        public int firstVertex;
        public int vertexCount;

        public EdgeMeshHeader(int firstNode, int nodeCount, int firstTriangle, int triangleCount, int firstVertex, int vertexCount)
        {
            this.firstNode = firstNode;
            this.nodeCount = nodeCount;
            this.firstEdge = firstTriangle;
            this.edgeCount = triangleCount;
            this.firstVertex = firstVertex;
            this.vertexCount = vertexCount;
        }
    }

    public class ObiEdgeMeshContainer
    {
        public Dictionary<EdgeCollider2D, ObiEdgeMeshHandle> handles;  /**< dictionary indexed by mesh, so that we don't generate data for the same mesh multiple times.*/

        public ObiNativeEdgeMeshHeaderList headers;
        public ObiNativeBIHNodeList bihNodes;
        public ObiNativeEdgeList edges;
        public ObiNativeVector2List vertices;

        public ObiEdgeMeshContainer()
        {
            handles = new Dictionary<EdgeCollider2D, ObiEdgeMeshHandle>();
            headers = new ObiNativeEdgeMeshHeaderList();
            bihNodes = new ObiNativeBIHNodeList();
            edges = new ObiNativeEdgeList();
            vertices = new ObiNativeVector2List();
        }

        public ObiEdgeMeshHandle GetOrCreateEdgeMesh(EdgeCollider2D source)
        {
            ObiEdgeMeshHandle handle;

            if (!handles.TryGetValue(source, out handle))
            {
                Vector2[] sourceVertices = source.points;
                int[] sourceEdges = new int[source.edgeCount * 2];

                for (int i = 0; i < source.edgeCount; ++i)
                {
                    sourceEdges[i * 2] = i;
                    sourceEdges[i * 2 + 1] = i + 1;
                }

                // Build a bounding interval hierarchy from the edges:
                IBounded[] t = new IBounded[source.edgeCount];
                for (int i = 0; i < source.edgeCount; ++i)
                {
                    t[i] = new Edge(i, i+1, sourceVertices[i], sourceVertices[i+1]);
                }
                var sourceBih = BIH.Build(ref t);

                Edge[] edgs = Array.ConvertAll(t, x => (Edge)x);

                handle = new ObiEdgeMeshHandle(source, headers.count);
                handles.Add(source, handle);
                headers.Add(new EdgeMeshHeader(bihNodes.count, sourceBih.Length, edges.count, edgs.Length, vertices.count, sourceVertices.Length));

                bihNodes.AddRange(sourceBih);
                edges.AddRange(edgs);
                vertices.AddRange(sourceVertices);
            }

            return handle;
        }

        public void DestroyEdgeMesh(ObiEdgeMeshHandle handle)
        {
            if (handle != null && handle.isValid && handle.index < handles.Count)
            {
                var header = headers[handle.index];

                // Update headers:
                for (int i = 0; i < headers.count; ++i)
                {
                    var h = headers[i];
                    if (h.firstEdge > header.firstEdge)
                    {
                        h.firstNode -= header.nodeCount;
                        h.firstEdge -= header.edgeCount;
                        h.firstVertex -= header.vertexCount;
                        headers[i] = h;
                    }
                }

                // update handles:
                foreach (var pair in handles)
                {
                    if (pair.Value.index > handle.index)
                        pair.Value.index--;
                }

                // Remove nodes, triangles and vertices
                bihNodes.RemoveRange(header.firstNode, header.nodeCount);
                edges.RemoveRange(header.firstEdge, header.edgeCount);
                vertices.RemoveRange(header.firstVertex, header.vertexCount);

                // remove header:
                headers.RemoveAt(handle.index);

                // remove the collider from the dictionary:
                handles.Remove(handle.owner);

                // Invalidate our handle:
                handle.Invalidate();
            }
        }

        public void Dispose()
        {
            if (headers != null)
                headers.Dispose();
            if (edges != null)
                edges.Dispose();
            if (vertices != null)
                vertices.Dispose();
            if (bihNodes != null)
                bihNodes.Dispose();
        }

    }
}
