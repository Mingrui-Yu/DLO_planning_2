using System;
using UnityEngine;

namespace Obi
{
    [Serializable]
    public class ObiNativeEdgeList : ObiNativeList<Edge>
    {
        public ObiNativeEdgeList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = new Edge();
        }
    }
}

