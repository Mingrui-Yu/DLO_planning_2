using UnityEngine;
using System.Collections;
using System;

namespace Obi
{
    [Serializable]
    public class ObiSkinConstraintsData : ObiConstraints<ObiSkinConstraintsBatch>
    {
        public override ObiSkinConstraintsBatch CreateBatch(ObiSkinConstraintsBatch source = null)
        {
            return new ObiSkinConstraintsBatch();
        }
    }
}
