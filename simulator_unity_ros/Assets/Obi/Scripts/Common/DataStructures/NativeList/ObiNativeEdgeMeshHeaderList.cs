using System;
using UnityEngine;

namespace Obi
{
    [Serializable]
    public class ObiNativeEdgeMeshHeaderList : ObiNativeList<EdgeMeshHeader>
    {
        public ObiNativeEdgeMeshHeaderList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = new EdgeMeshHeader();
        }
    }
}

