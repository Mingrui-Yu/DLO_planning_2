using System;
using UnityEngine;

namespace Obi
{
    [Serializable]
    public class ObiNativeVector2List : ObiNativeList<Vector2>
    {
   
        public ObiNativeVector2List(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = Vector2.zero;
        }

    }
}

