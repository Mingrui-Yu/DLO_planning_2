using UnityEngine;
using System;
using System.Collections;

namespace Obi{

    /**
     * Holds information about the physics properties of a particle or collider, and how it should react to collisions.
     */
    [CreateAssetMenu(fileName = "collision material", menuName = "Obi/Collision Material", order = 180)]
    public class ObiCollisionMaterial : ScriptableObject
    {
        protected ObiCollisionMaterialHandle materialHandle;

	    public float dynamicFriction;
	    public float staticFriction;
	    public float stickiness;
	    public float stickDistance;
	
	    public Oni.MaterialCombineMode frictionCombine;
	    public Oni.MaterialCombineMode stickinessCombine;

	    [Space]
	    public bool rollingContacts = false;

	    [Indent()]
	    [VisibleIf("rollingContacts")]
	    public float rollingFriction;

        public ObiCollisionMaterialHandle handle
        {
            get
            {
                CreateMaterialIfNeeded();
                return materialHandle;
            }
        }

	    public void OnEnable()
        {
            UpdateMaterial();
        }

	    public void OnDisable()
        {
            ObiColliderWorld.GetInstance().DestroyCollisionMaterial(materialHandle);
        }

        public void OnValidate()
        {
            // we can't create GameObjects in OnValidate(), so make sure the colliderworld already exists.
            UpdateMaterial();
        }

        public void UpdateMaterial()
        {
            var world = ObiColliderWorld.GetInstance();
            var mat = world.collisionMaterials[handle.index];
            mat.FromObiCollisionMaterial(this);
            world.collisionMaterials[handle.index] = mat;
        }

        protected void CreateMaterialIfNeeded()
        {
            if (materialHandle == null || !materialHandle.isValid)
            {
                var world = ObiColliderWorld.GetInstance();

                // create the material:
                materialHandle = world.CreateCollisionMaterial();
                materialHandle.owner = this;

                // copy material data from this material (use materialHandle instead of handle, to not retrigger CreateMaterialIfNeeded)
                var mat = world.collisionMaterials[materialHandle.index];
                mat.FromObiCollisionMaterial(this);
                world.collisionMaterials[materialHandle.index] = mat;
            }
        }
    }
}