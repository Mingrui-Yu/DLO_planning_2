using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Obi
{
    public class ASDF
    {
        static readonly Vector4[] corners = 
        {
            new Vector4(-1,-1,-1,-1),
            new Vector4(-1,-1,1,-1),
            new Vector4(-1,1,-1,-1),
            new Vector4(-1,1,1,-1),

            new Vector4(1,-1,-1,-1),
            new Vector4(1,-1,1,-1),
            new Vector4(1,1,-1,-1),
            new Vector4(1,1,1,-1)
        };

        static readonly Vector4[] samples =
        {
            new Vector4(0,0,0,0),
            new Vector4(1,0,0,0),
            new Vector4(-1,0,0,0),
            new Vector4(0,1,0,0),
            new Vector4(0,-1,0,0),
            new Vector4(0,0,1,0),
            new Vector4(0,0,-1,0),

            new Vector4(0,-1,-1,0),
            new Vector4(0,-1,1,0),
            new Vector4(0,1,-1,0),
            new Vector4(0,1,1,0),

            new Vector4(-1,0,-1,0),
            new Vector4(-1,0,1,0),
            new Vector4(1,0,-1,0),
            new Vector4(1,0,1,0),

            new Vector4(-1,-1,0,0),
            new Vector4(-1,1,0,0),
            new Vector4(1,-1,0,0),
            new Vector4(1,1,0,0)
        };

        const float sqrt3 = 1.73205f;

        public static IEnumerator Build(float maxError, int maxDepth, Vector3[] vertexPositions, int[] triangleIndices, List<DFNode> nodes, int yieldAfterNodeCount = 32)
        {
            // Empty vertex or triangle lists, return.
            if (maxDepth <= 0 ||
                nodes == null ||
                vertexPositions == null || vertexPositions.Length == 0 ||
                triangleIndices == null || triangleIndices.Length == 0)
                yield break;
           
            // Build a bounding interval hierarchy from the triangles, to speed up distance queries:
            IBounded[] t = new IBounded[triangleIndices.Length / 3];
            for (int i = 0; i < t.Length; ++i)
            {
                int t1 = triangleIndices[i * 3];
                int t2 = triangleIndices[i * 3 + 1];
                int t3 = triangleIndices[i * 3 + 2];
                t[i] = new Triangle(t1, t2, t3, vertexPositions[t1], vertexPositions[t2], vertexPositions[t3]);
            }
            var bih = BIH.Build(ref t);

            // Copy reordered triangles over to a new array:
            Triangle[] tris = Array.ConvertAll(t, x => (Triangle)x); 

            // Build angle weighted normals, used to determine the sign of the distance field.
            Vector3[] angleNormals = ObiUtils.CalculateAngleWeightedNormals(vertexPositions,triangleIndices);

            // Calculate bounding box of the mesh:
            Bounds bounds = new Bounds(vertexPositions[0], Vector3.zero);
            for (int i = 1; i < vertexPositions.Length; ++i)
                bounds.Encapsulate(vertexPositions[i]);

            bounds.Expand(0.2f);

            // Auxiliar variables to keep track of current tree depth:
            int depth = 0;
            int nodesToNextLevel = 1;

            // Initialize node list:
            Vector4 center = bounds.center;
            Vector3 boundsExtents = bounds.extents;
            center[3] = Mathf.Max(boundsExtents[0], Math.Max(boundsExtents[1], boundsExtents[2]));
            nodes.Clear();
            nodes.Add(new DFNode(center));


            var queue = new Queue<int>();
            queue.Enqueue(0);

            int processedNodeCount = 0;
            while (queue.Count > 0)
            {
                // get current node:
                int index = queue.Dequeue();
                var node = nodes[index];

                // measure distance at the 8 node corners:
                for (int i = 0; i < 8; ++i)
                {
                    Vector4 point = node.center + corners[i] * node.center[3];
                    point[3] = 0;
                    float distance = BIH.DistanceToSurface(bih, tris, vertexPositions,angleNormals,point);

                    if (i < 4)
                        node.distancesA[i] = distance;
                    else
                        node.distancesB[i - 4] = distance;
                }

                // only subdivide those nodes intersecting the surface:
                if (depth < maxDepth && Mathf.Abs(BIH.DistanceToSurface(bih, tris, vertexPositions, angleNormals, node.center)) < node.center[3] * sqrt3)
                {

                    // calculate mean squared error between measured distances and interpolated ones:
                    float mse = 0;
                    for (int i = 0; i < samples.Length; ++i)
                    {
                        Vector4 point = node.center + samples[i] * node.center[3];
                        float groundTruth = BIH.DistanceToSurface(bih, tris, vertexPositions, angleNormals, point);
                        float estimation = node.Sample(point);
                        float d = groundTruth - estimation;
                        mse += d * d;
                    }
                    mse /= (float)samples.Length;

                    // if error > threshold, subdivide the node:
                    if (mse > maxError)
                    {
                        node.firstChild = nodes.Count;
                        for (int i = 0; i < 8; ++i)
                        {
                            queue.Enqueue(nodes.Count);
                            nodes.Add(new DFNode(node.center + corners[i] * node.center[3] * 0.5f));
                        }
                    }

                    // keep track of current depth:
                    if (--nodesToNextLevel == 0)
                    {
                        depth++;
                        nodesToNextLevel = queue.Count;
                    }
                }

                // feed the modified node back:
                nodes[index] = node;

                // if we've processed enough nodes, yield.
                if (nodes.Count - processedNodeCount >= yieldAfterNodeCount)
                {
                    processedNodeCount = nodes.Count;
                    yield return null;
                }
            }
        }

        public static float Sample(List<DFNode> nodes, Vector3 position)
        {
            if (nodes != null && nodes.Count > 0)
            {
                var queue = new Queue<int>();
                queue.Enqueue(0);

                while (queue.Count > 0)
                {
                    // get current node:
                    var node = nodes[queue.Dequeue()];

                    if (node.firstChild > -1)
                        queue.Enqueue(node.firstChild + node.GetOctant(position));
                    else
                        return node.Sample(position);
                }
            }
            return 0;
        }
    }
}
