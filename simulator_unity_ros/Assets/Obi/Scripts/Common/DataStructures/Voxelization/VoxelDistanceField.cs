using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Obi
{

    /**
     * Generates a sparse distance field from a voxel representation of a mesh.
     */ 
    public class VoxelDistanceField
    {
        public Vector3Int[,,] distanceField;

        private MeshVoxelizer voxelizer;

        public VoxelDistanceField(MeshVoxelizer voxelizer)
        {
            this.voxelizer = voxelizer;
        }

        private bool VoxelExists(Vector3Int coords)
        {
            return coords.x >= 0 && coords.y >= 0 && coords.z >= 0 &&
                         coords.x < voxelizer.voxels.GetLength(0) &&
                         coords.y < voxelizer.voxels.GetLength(1) &&
                         coords.z < voxelizer.voxels.GetLength(2);
        }
        
        public void JumpFlood()
        {

            // create and initialize distance field:
            distanceField = new Vector3Int[voxelizer.voxels.GetLength(0),
                                           voxelizer.voxels.GetLength(1),
                                           voxelizer.voxels.GetLength(2)];

            // create auxiliar buffer for ping-pong.
            Vector3Int[,,] auxBuffer = new Vector3Int[voxelizer.voxels.GetLength(0),
                                                      voxelizer.voxels.GetLength(1),
                                                      voxelizer.voxels.GetLength(2)];

            // initialize distance field:
            for (int x = 0; x < distanceField.GetLength(0); ++x)
                for (int y = 0; y < distanceField.GetLength(1); ++y)
                    for (int z = 0; z < distanceField.GetLength(2); ++z)
                    {
                        if (voxelizer.voxels[x, y, z] == MeshVoxelizer.Voxel.Outside)
                            distanceField[x, y, z] = new Vector3Int(x, y, z);
                        else
                            distanceField[x, y, z] = new Vector3Int(-1, -1, -1);
                    }

            // calculate the maximum size of the buffer:
            float size = Mathf.Max(distanceField.GetLength(0), 
                                   distanceField.GetLength(1), 
                                   distanceField.GetLength(2));

            // calculate how many JFA passes we need to perform, based on the size of the buffer.
            int passes = Mathf.FloorToInt(Mathf.Log(size, 2));

            // jump flood algorithm:
            for (int pass = 0; pass < passes; ++pass)
            {
                int passStride = (int)Mathf.Pow(2, passes - pass - 1);
                JumpFloodPass(passStride, distanceField, auxBuffer);

                // swap buffers:
                Vector3Int[,,] temp = distanceField;
                distanceField = auxBuffer;
                auxBuffer = temp;
            }

            if (passes % 2 != 0)
                distanceField = auxBuffer;

        }

        private void JumpFloodPass(int stride, Vector3Int[,,] input, Vector3Int[,,] output)
        {

            // for each voxel:
            for (int x = 0; x < input.GetLength(0); ++x)
                for (int y = 0; y < input.GetLength(1); ++y)
                    for (int z = 0; z < input.GetLength(2); ++z)
                    {
                        // our position:
                        Vector3Int p = new Vector3Int(x, y, z);

                        // our seed:
                        Vector3Int s = input[x, y, z];

                        // copy the closest seed to the output, in case we do not update it this pass:
                        output[x, y, z] = s;

                        // this voxel is a seed, skip it.
                        if (s.x == x && s.y == y && s.z == z)
                            continue;

                        // distance to our closest seed:
                        float dist = float.MaxValue;
                        if (s.x >= 0)
                            dist = (s - p).sqrMagnitude;

                        // for each neighbor voxel:
                        for (int nx = -1; nx <= 1; ++nx)
                            for (int ny = -1; ny <= 1; ++ny)
                                for (int nz = -1; nz <= 1; ++nz)
                                {

                                    // neighbor's position:
                                    Vector3Int v = new Vector3Int(x + nx * stride,
                                                                  y + ny * stride,
                                                                  z + nz * stride);

                                    if (VoxelExists(v))
                                    {
                                        // neighbors' closest seed.
                                        Vector3Int n = input[v.x, v.y, v.z];

                                        if (n.x >= 0)
                                        {
                                            // distance to neighbor's closest seed:
                                            float newDist = (n - p).sqrMagnitude;

                                            // if the distance to the neighbor's closest seed is smaller than the distance to ours:
                                            if (newDist < dist)
                                            {
                                                output[x, y, z] = n;
                                                dist = newDist;
                                            }
                                        }
                                    }
                                }
                    }

        }
    }
}