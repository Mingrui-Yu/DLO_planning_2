using System;
using UnityEngine;

namespace Obi
{

    public class ObiCharacterControllerShapeTracker : ObiShapeTracker
    {

        public ObiCharacterControllerShapeTracker(ObiCollider source, CharacterController collider)
        {
            this.collider = collider;
            this.source = source;
        }

        public override bool UpdateIfNeeded()
        {

            CharacterController character = collider as CharacterController;

            // retrieve collision world and index:
            var world = ObiColliderWorld.GetInstance();
            int index = source.Handle.index;

            // update collider:
            var shape = world.colliderShapes[index];
            shape.type = ColliderShape.ShapeType.Capsule;
            shape.phase = source.Phase;
            shape.flags = character.isTrigger ? 1 : 0;
            shape.rigidbodyIndex = source.Rigidbody != null ? source.Rigidbody.handle.index : -1;
            shape.materialIndex = source.CollisionMaterial != null ? source.CollisionMaterial.handle.index : -1;
            shape.contactOffset = character.contactOffset + source.Thickness;
            shape.center = character.center;
            shape.size = new Vector4(character.radius, character.height, 1, 0);
            world.colliderShapes[index] = shape;

            // update bounds:
            var aabb = world.colliderAabbs[index];
            aabb.FromBounds(character.bounds, shape.contactOffset);
            world.colliderAabbs[index] = aabb;

            // update transform:
            var trfm = world.colliderTransforms[index];
            trfm.FromTransform(character.transform);
            world.colliderTransforms[index] = trfm;

            return true;
        }

    }
}

