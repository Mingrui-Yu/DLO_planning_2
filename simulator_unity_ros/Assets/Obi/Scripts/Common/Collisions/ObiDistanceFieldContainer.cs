using System;
using System.Collections.Generic;
using UnityEngine;

namespace Obi
{

    public class ObiDistanceFieldHandle : ObiResourceHandle<ObiDistanceField>
    {
        public ObiDistanceFieldHandle(ObiDistanceField field, int index = -1) : base(index) { owner = field; }
    }

    public struct DistanceFieldHeader //we need to use the header in the backend, so it must be a struct.
    {
        public int firstNode;
        public int nodeCount;
  
        public DistanceFieldHeader(int firstNode, int nodeCount)
        {
            this.firstNode = firstNode;
            this.nodeCount = nodeCount;
        }
    }

    public class ObiDistanceFieldContainer
    {
        public Dictionary<ObiDistanceField, ObiDistanceFieldHandle> handles;  /**< dictionary indexed by asset, so that we don't generate data for the same distance field multiple times.*/

        public ObiNativeDistanceFieldHeaderList headers; /**< One header per distance field.*/
        public ObiNativeDFNodeList dfNodes;

        public ObiDistanceFieldContainer()
        {
            handles = new Dictionary<ObiDistanceField, ObiDistanceFieldHandle>();
            headers = new ObiNativeDistanceFieldHeaderList();
            dfNodes = new ObiNativeDFNodeList();
        }

        public ObiDistanceFieldHandle GetOrCreateDistanceField(ObiDistanceField source)
        {
            ObiDistanceFieldHandle handle;

            if (!handles.TryGetValue(source, out handle))
            {

                handle = new ObiDistanceFieldHandle(source, headers.count);
                handles.Add(source, handle);
                headers.Add(new DistanceFieldHeader(dfNodes.count, source.nodes.Count));

                dfNodes.AddRange(source.nodes);
            }

            return handle;
        }

        public void DestroyDistanceField(ObiDistanceFieldHandle handle)
        {
            if (handle != null && handle.isValid && handle.index < handles.Count)
            {
                var header = headers[handle.index];

                // Update headers:
                for (int i = 0; i < headers.count; ++i)
                {
                    var h = headers[i];
                    if (h.firstNode > header.firstNode)
                    {
                        h.firstNode -= header.nodeCount;
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
                dfNodes.RemoveRange(header.firstNode, header.nodeCount);

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
            if (dfNodes != null)
                dfNodes.Dispose();
        }

    }
}
