using System;
using System.Collections.Generic;
using UnityEngine;

namespace Obi
{
    public class ObiHeightFieldHandle : ObiResourceHandle<TerrainData>
    {
        public ObiHeightFieldHandle(TerrainData data, int index = -1) : base(index) { owner = data; }
    }

    public struct HeightFieldHeader //we need to use the header in the backend, so it must be a struct.
    {
        public int firstSample;
        public int sampleCount;

        public HeightFieldHeader(int firstSample, int sampleCount)
        {
            this.firstSample = firstSample;
            this.sampleCount = sampleCount;
        }
    }

    public class ObiHeightFieldContainer
    {
        public Dictionary<TerrainData, ObiHeightFieldHandle> handles;  /**< dictionary indexed by asset, so that we don't generate data for the same distance field multiple times.*/

        public ObiNativeHeightFieldHeaderList headers; /**< One header per distance field.*/
        public ObiNativeFloatList samples;

        public ObiHeightFieldContainer()
        {
            handles = new Dictionary<TerrainData, ObiHeightFieldHandle>();
            headers = new ObiNativeHeightFieldHeaderList();
            samples = new ObiNativeFloatList();
        }

        public ObiHeightFieldHandle GetOrCreateHeightField(TerrainData source)
        {
            ObiHeightFieldHandle handle;

            if (!handles.TryGetValue(source, out handle))
            {
                // Get the heighfield into a 1d array:
                int width = source.heightmapResolution;
                int height = source.heightmapResolution;

                float[,] heights = source.GetHeights(0, 0, width, height);
                float[] buffer = new float[width * height];

                for (int y = 0; y < height; ++y)
                    for (int x = 0; x < width; ++x)
                        buffer[y * width + x] = heights[y, x];

                handle = new ObiHeightFieldHandle(source, headers.count);
                handles.Add(source, handle);
                headers.Add(new HeightFieldHeader(samples.count, buffer.Length));

                samples.AddRange(buffer);
            }

            return handle;
        }

        public void DestroyHeightField(ObiHeightFieldHandle handle)
        {
            if (handle != null && handle.isValid && handle.index < handles.Count)
            {
                var header = headers[handle.index];

                // Update headers:
                for (int i = 0; i < headers.count; ++i)
                {
                    var h = headers[i];
                    if (h.firstSample > header.firstSample)
                    {
                        h.firstSample -= header.sampleCount;
                        headers[i] = h;
                    }
                }

                // update handles:
                foreach (var pair in handles)
                {
                    if (pair.Value.index > handle.index)
                        pair.Value.index--;
                }

                // Remove nodes
                samples.RemoveRange(header.firstSample, header.sampleCount);

                // remove header:
                headers.RemoveAt(handle.index);

                // remove the heightfield from the dictionary:
                handles.Remove(handle.owner);

                // Invalidate our handle:
                handle.Invalidate();
            }
        }

        public void Dispose()
        {
            if (headers != null)
                headers.Dispose();
            if (samples != null)
                samples.Dispose();
        }

    }
}
