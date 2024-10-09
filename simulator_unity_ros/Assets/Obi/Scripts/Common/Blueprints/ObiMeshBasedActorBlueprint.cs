using UnityEngine;
using System.Collections;


namespace Obi
{
    public abstract class ObiMeshBasedActorBlueprint : ObiActorBlueprint
    {
        public Mesh inputMesh;               /**< Mesh used to generate the blueprint.*/
        public Vector3 scale = Vector3.one;
    }
}
