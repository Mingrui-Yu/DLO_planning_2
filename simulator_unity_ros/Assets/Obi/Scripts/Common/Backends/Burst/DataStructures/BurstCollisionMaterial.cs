#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using UnityEngine;
using Unity.Mathematics;
using System.Collections;
using System;

namespace Obi
{
    public struct BurstCollisionMaterial // TODO: use CollisionMaterial directly.
    {
        public float dynamicFriction;
        public float staticFriction;
        public float rollingFriction;
        public float stickiness;
        public float stickDistance;
        public Oni.MaterialCombineMode frictionCombine;
        public Oni.MaterialCombineMode stickinessCombine;
        public int rollingContacts;

        public static BurstCollisionMaterial CombineWith(BurstCollisionMaterial a, BurstCollisionMaterial b)
        {
            BurstCollisionMaterial result = new BurstCollisionMaterial();
            var frictionCombineMode = (Oni.MaterialCombineMode)math.max((int)a.frictionCombine, (int)b.frictionCombine);
            var stickCombineMode = (Oni.MaterialCombineMode)math.max((int)a.stickinessCombine, (int)b.stickinessCombine);

            switch (frictionCombineMode)
            {
                case Oni.MaterialCombineMode.Average:
                    result.dynamicFriction = (a.dynamicFriction + b.dynamicFriction) * 0.5f;
                    result.staticFriction = (a.staticFriction + b.staticFriction) * 0.5f;
                    result.rollingFriction = (a.rollingFriction + b.rollingFriction) * 0.5f;
                    break;

                case Oni.MaterialCombineMode.Minimum:
                    result.dynamicFriction = math.min(a.dynamicFriction, b.dynamicFriction);
                    result.staticFriction = math.min(a.staticFriction, b.staticFriction);
                    result.rollingFriction = math.min(a.rollingFriction, b.rollingFriction);
                    break;

                case Oni.MaterialCombineMode.Maximum:
                    result.dynamicFriction = math.max(a.dynamicFriction, b.dynamicFriction);
                    result.staticFriction = math.max(a.staticFriction, b.staticFriction);
                    result.rollingFriction = math.max(a.rollingFriction, b.rollingFriction);
                    break;

                case Oni.MaterialCombineMode.Multiply:
                    result.dynamicFriction = a.dynamicFriction * b.dynamicFriction;
                    result.staticFriction = a.staticFriction * b.staticFriction;
                    result.rollingFriction = a.rollingFriction * b.rollingFriction;
                    break;
            }

            switch (stickCombineMode)
            {
                case Oni.MaterialCombineMode.Average:
                    result.stickiness = (a.stickiness + b.stickiness) * 0.5f;
                    break;

                case Oni.MaterialCombineMode.Minimum:
                    result.stickiness = math.min(a.stickiness, b.stickiness);
                    break;

                case Oni.MaterialCombineMode.Maximum:
                    result.stickiness = math.max(a.stickiness, b.stickiness);
                    break;

                case Oni.MaterialCombineMode.Multiply:
                    result.stickiness = a.stickiness * b.stickiness;
                    break;
            }

            result.stickDistance = math.max(a.stickDistance, b.stickDistance);
            result.rollingContacts = a.rollingContacts | b.rollingContacts;
            return result;
        }
    }
}
#endif