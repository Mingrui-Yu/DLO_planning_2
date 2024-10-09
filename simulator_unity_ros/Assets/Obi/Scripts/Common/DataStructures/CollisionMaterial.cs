using UnityEngine;
using System.Collections;

namespace Obi
{

    public struct CollisionMaterial 
    {
        public float dynamicFriction;
        public float staticFriction;
        public float rollingFriction;
        public float stickiness;
        public float stickDistance;
        public Oni.MaterialCombineMode frictionCombine;
        public Oni.MaterialCombineMode stickinessCombine;
        public int rollingContacts;

        public void FromObiCollisionMaterial(ObiCollisionMaterial material)
        {
            if (material != null)
            {
                dynamicFriction = material.dynamicFriction;
                staticFriction = material.staticFriction;
                stickiness = material.stickiness;
                stickDistance = material.stickDistance;
                rollingFriction = material.rollingFriction;
                frictionCombine = material.frictionCombine;
                stickinessCombine = material.stickinessCombine;
                rollingContacts = material.rollingContacts ? 1 : 0;
            }
        }

    }
}
