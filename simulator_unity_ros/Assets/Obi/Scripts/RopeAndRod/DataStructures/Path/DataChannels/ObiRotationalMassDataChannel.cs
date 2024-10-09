using UnityEngine;
using System;
using System.Collections;

namespace Obi
{
    [Serializable]
    public class ObiRotationalMassDataChannel : ObiPathDataChannelIdentity<float>
    {
        public ObiRotationalMassDataChannel() : base(new ObiCatmullRomInterpolator()) { }
    }
}