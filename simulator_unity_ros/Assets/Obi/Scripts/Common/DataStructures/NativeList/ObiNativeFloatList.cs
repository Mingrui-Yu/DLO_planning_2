using System;
using UnityEngine;

namespace Obi
{
    [Serializable]
    public class ObiNativeFloatList : ObiNativeList<float>
    {

        public ObiNativeFloatList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = 0;
        }

    }
}

