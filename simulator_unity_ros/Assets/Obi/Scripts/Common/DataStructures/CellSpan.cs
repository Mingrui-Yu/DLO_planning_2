using UnityEngine;
using System.Collections;

namespace Obi
{
    public struct CellSpan
    {
        public VInt4 min;
        public VInt4 max;

        public CellSpan(VInt4 min, VInt4 max)
        {
            this.min = min;
            this.max = max;
        }
    }
}
