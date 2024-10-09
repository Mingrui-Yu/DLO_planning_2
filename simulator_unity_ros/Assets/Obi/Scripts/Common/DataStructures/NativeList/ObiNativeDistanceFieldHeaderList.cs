using System;
using UnityEngine;

namespace Obi
{
    [Serializable]
    public class ObiNativeDistanceFieldHeaderList : ObiNativeList<DistanceFieldHeader>
    {

        public ObiNativeDistanceFieldHeaderList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = new DistanceFieldHeader();
        }
    }
}

