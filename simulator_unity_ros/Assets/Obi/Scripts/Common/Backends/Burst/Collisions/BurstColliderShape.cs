#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;
using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;

namespace Obi
{

    public struct BurstColliderShape
    {
        public float4 center;
        public float4 size; /**<     box: size of the box in each axis.
                                      sphere: radius of sphere (x,y,z),
                                      capsule: radius (x), height(y), direction (z, can be 0, 1 or 2).
                                      heightmap: width (x axis), height (y axis) and depth (z axis) in world units.*/

        public ColliderShape.ShapeType type;
        public float contactOffset;
        public int dataIndex;
        public int rigidbodyIndex;  // index of the associated rigidbody in the collision world.
        public int materialIndex;   // index of the associated material in the collision world.
        public int phase;
        public int flags;           // for now, only used for trigger (1) or regular collider (0).
        public int is2D;            // whether the collider is 2D (1) or 3D (0).
    }
}
#endif