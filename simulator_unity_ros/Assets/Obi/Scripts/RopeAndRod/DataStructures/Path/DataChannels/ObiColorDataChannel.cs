using UnityEngine;
using System;
using System.Collections;

namespace Obi
{
    [Serializable]
    public class ObiColorDataChannel : ObiPathDataChannelIdentity<Color>
    {
        public ObiColorDataChannel() : base(new ObiColorInterpolator3D()) { }
    }
}
