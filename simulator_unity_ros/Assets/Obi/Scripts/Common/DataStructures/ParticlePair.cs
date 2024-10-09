using UnityEngine;
using System.Collections;


namespace Obi
{
    public struct ParticlePair
    {
        public int first;
        public int second;

        public ParticlePair(int first, int second)
        {
            this.first = first;
            this.second = second;
        }

        public int this[int index]
        {
            get { return index == 0 ? first : second; }
            set { if (index == 0) first = value; else second = value; }
        }
    }
}
