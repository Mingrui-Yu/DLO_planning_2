using System;
using UnityEngine;

namespace Obi
{
    [Serializable]
    public class ObiNativeRigidbodyList : ObiNativeList<ColliderRigidbody>
    {

        public ObiNativeRigidbodyList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = new ColliderRigidbody();
        }



    }
}

