using UnityEngine;
using System.Collections;


namespace Obi
{
    public static class ObiVectorMath
    {
        public static void Cross(Vector3 a, Vector3 b, ref float x, ref float y, ref float z)
        {
            x = a.y * b.z - a.z * b.y;
            y = a.z * b.x - a.x * b.z;
            z = a.x * b.y - a.y * b.x;
        }

        public static void Cross(Vector3 a, Vector3 b, ref Vector3 res)
        {
            res.x = a.y * b.z - a.z * b.y;
            res.y = a.z * b.x - a.x * b.z;
            res.z = a.x * b.y - a.y * b.x;
        }

        public static void Cross(float ax, float ay, float az, float bx, float by, float bz, ref float x, ref float y, ref float z)
        {
            x = ay * bz - az * by;
            y = az * bx - ax * bz;
            z = ax * by - ay * bx;
        }

        /**
         * res = b - a
         */
        public static void Subtract(Vector3 a, Vector3 b, ref Vector3 res)
        {
            res.x = b.x - a.x;
            res.y = b.y - a.y;
            res.z = b.z - a.z;
        }

        public static void BarycentricInterpolation(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 n1, Vector3 n2, Vector3 n3, Vector3 coords, float height, ref Vector3 res)
        {
            res.x = coords.x * p1.x + coords.y * p2.x + coords.z * p3.x + (coords.x * n1.x + coords.y * n2.x + coords.z * n3.x) * height;
            res.y = coords.x * p1.y + coords.y * p2.y + coords.z * p3.y + (coords.x * n1.y + coords.y * n2.y + coords.z * n3.y) * height;
            res.z = coords.x * p1.z + coords.y * p2.z + coords.z * p3.z + (coords.x * n1.z + coords.y * n2.z + coords.z * n3.z) * height;
        }

    }
}