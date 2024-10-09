using System;
using UnityEngine;

namespace Obi
{
    [Serializable]
    public class ObiNativeVector3List : ObiNativeList<Vector3>
    {

        public ObiNativeVector3List(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = Vector3.zero;
        }

    }
}

