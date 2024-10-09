using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace Obi
{
    [Serializable]
    public class ObiParticleGroup : ScriptableObject
    {
        public List<int> particleIndices = new List<int>() { };
        public ObiActorBlueprint m_Blueprint = null;

        public ObiActorBlueprint blueprint
        {
            get { return m_Blueprint; }
        }

        public void SetSourceBlueprint(ObiActorBlueprint blueprint)
        {
            this.m_Blueprint = blueprint;
        }

        public int Count
        {
            get { return particleIndices.Count; }
        }

        public bool ContainsParticle(int index)
        {
            return particleIndices.Contains(index);
        }
    }
}