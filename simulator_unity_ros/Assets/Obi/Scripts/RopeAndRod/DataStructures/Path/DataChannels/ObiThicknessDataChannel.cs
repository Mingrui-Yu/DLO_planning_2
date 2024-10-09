using UnityEngine;
using System;
using System.Collections;

namespace Obi
{
    [Serializable]
    public class ObiThicknessDataChannel : ObiPathDataChannelIdentity<float>
    {
        public ObiThicknessDataChannel() : base(new ObiCatmullRomInterpolator()) { }
    }
}
