using UnityEngine;
using System;
using System.Collections;

namespace Obi
{
    [Serializable]
    public class ObiPhaseDataChannel : ObiPathDataChannelIdentity<int>
    {
        public ObiPhaseDataChannel() : base(new ObiConstantInterpolator()) { }
    }
}