using System;
using UnityEngine;

namespace Obi
{
    [Serializable]
    public class ObiNativeInt4List : ObiNativeList<VInt4>
    {

        public ObiNativeInt4List(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = new VInt4(0,0,0,0);
        }

        public ObiNativeInt4List(int capacity, int alignment, VInt4 defaultValue) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = defaultValue;
        }
    }
}

