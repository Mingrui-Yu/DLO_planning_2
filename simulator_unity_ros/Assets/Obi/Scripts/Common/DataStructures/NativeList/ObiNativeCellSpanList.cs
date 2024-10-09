using System;
using UnityEngine;

namespace Obi
{
    [Serializable]
    public class ObiNativeCellSpanList : ObiNativeList<CellSpan>
    {

        public ObiNativeCellSpanList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = new CellSpan();
        }

    }
}

