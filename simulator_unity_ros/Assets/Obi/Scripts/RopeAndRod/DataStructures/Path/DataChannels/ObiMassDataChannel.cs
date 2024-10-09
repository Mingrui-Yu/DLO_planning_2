using UnityEngine;
using System;
using System.Collections;

namespace Obi
{
    [Serializable]
    public class ObiMassDataChannel : ObiPathDataChannelIdentity<float>
    {
        public ObiMassDataChannel() : base(new ObiCatmullRomInterpolator()) { }
    }
}
