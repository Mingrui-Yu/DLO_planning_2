using System;
using UnityEngine;

namespace Obi
{
    [Serializable]
    public class ObiNativeQuaternionList : ObiNativeList<Quaternion>
    {

        public ObiNativeQuaternionList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = Quaternion.identity;
        }

        public ObiNativeQuaternionList(int capacity, int alignment, Quaternion defaultValue) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = defaultValue;
        }

    }
}

