using UnityEngine;

namespace Obi
{
    public struct Aabb
    {
        public Vector4 min;
        public Vector4 max;

        public Vector4 center
        {
            get { return min + (max - min) * 0.5f; }
        }

        public Vector4 size
        {
            get { return max - min; }
        }

        public Aabb(Vector4 min, Vector4 max)
        {
            this.min = min;
            this.max = max;
        }

        public Aabb(Vector4 point)
        {
            this.min = point;
            this.max = point;
        }

        public void Encapsulate(Vector4 point)
        {
            min = Vector4.Min(min, point);
            max = Vector4.Max(max, point); 
        }

        public void Encapsulate(Aabb other)
        {
            min = Vector4.Min(min, other.min);
            max = Vector4.Max(max, other.max);
        }

        public void FromBounds(Bounds bounds, float thickness, bool is2D = false)
        {
            Vector3 s = Vector3.one * thickness;
            min = bounds.min - s;
            max = bounds.max + s;
            if (is2D)
                max[2] = min[2] = 0;
        }
    }
}
