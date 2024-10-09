using System;
using UnityEngine;

namespace Obi
{
    public class ObiNativeIntPtrList : ObiNativeList<IntPtr>
    {

        public ObiNativeIntPtrList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = IntPtr.Zero;
        }
    }
}

