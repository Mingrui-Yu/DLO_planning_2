using UnityEngine;
using System;
using System.Collections;

namespace Obi
{
    [Serializable]
    public class ObiNormalDataChannel : ObiPathDataChannelIdentity<Vector3>
    {
        public ObiNormalDataChannel() : base(new ObiCatmullRomInterpolator3D()) { }
    }
}
