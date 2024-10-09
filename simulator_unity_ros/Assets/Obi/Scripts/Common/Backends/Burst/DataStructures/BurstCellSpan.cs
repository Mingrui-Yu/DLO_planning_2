#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using UnityEngine;
using Unity.Mathematics;
using System.Collections;
using System;

namespace Obi
{
    public struct BurstCellSpan : IEquatable<BurstCellSpan>
    {
        public int4 min;
        public int4 max;


        public BurstCellSpan(CellSpan span)
        {
            this.min = new int4(span.min.x, span.min.y, span.min.z, span.min.w);
            this.max = new int4(span.max.x, span.max.y, span.max.z, span.max.w);
        }

        public BurstCellSpan(int4 min, int4 max)
        {
            this.min = min;
            this.max = max;
        }

        public int level
        {
            get{return min.w;}
        }

        public bool Equals(BurstCellSpan other)
        {
            return min.Equals(other.min) && max.Equals(other.max);
        }

        public override bool Equals(object obj)
        {
            return this.Equals((BurstCellSpan)obj);
        }

        public override int GetHashCode()
        {
            return 0; // we don't have any non-mutable fields, so just return 0.
        }

        public static bool operator ==(BurstCellSpan a, BurstCellSpan b)
        {
            return a.Equals(b);
        }

        public static bool operator !=(BurstCellSpan a, BurstCellSpan b)
        {
            return !a.Equals(b);
        }


    }
}
#endif