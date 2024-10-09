using UnityEngine;
using System.Collections;
using System;

namespace Obi
{
    [Serializable]
    public class ObiPinConstraintsData : ObiConstraints<ObiPinConstraintsBatch>
    {

        public override ObiPinConstraintsBatch CreateBatch(ObiPinConstraintsBatch source = null)
        {
            return new ObiPinConstraintsBatch();
        }
    }
}
