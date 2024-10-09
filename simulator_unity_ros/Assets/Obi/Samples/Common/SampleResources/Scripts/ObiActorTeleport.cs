using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Obi;

public class ObiActorTeleport : MonoBehaviour
{
    public ObiActor actor;
    public Transform target;

    public void Teleport()
    {
        actor.Teleport(target.position, target.rotation);
    }
}
