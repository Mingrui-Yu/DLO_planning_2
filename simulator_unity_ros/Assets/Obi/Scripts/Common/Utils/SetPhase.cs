using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Obi
{
    [RequireComponent(typeof(ObiActor))]
    public class SetPhase : MonoBehaviour
    {
        public int phase;

        private void Awake()
        {
            GetComponent<ObiActor>().OnBlueprintLoaded += Set;
        }

        private void OnDestroy()
        {
            GetComponent<ObiActor>().OnBlueprintLoaded -= Set;
        }

        private void OnValidate()
        {
            phase = Mathf.Clamp(phase, 0, (1 << 24) - 1);
        }

        private void Set(ObiActor actor, ObiActorBlueprint blueprint)
        {
            actor.SetPhase(phase);
        }
    }
}
