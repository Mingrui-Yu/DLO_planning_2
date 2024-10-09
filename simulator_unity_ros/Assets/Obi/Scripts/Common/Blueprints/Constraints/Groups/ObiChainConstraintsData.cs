using UnityEngine;
using System.Collections;
using System;

namespace Obi
{

    public interface IChainConstraintsUser
    {

        bool chainConstraintsEnabled
        {
            get;
            set;
        }

        float tightness
        {
            get;
            set;
        }
    }

    [Serializable]
    public class ObiChainConstraintsData : ObiConstraints<ObiChainConstraintsBatch>
    {
        public override ObiChainConstraintsBatch CreateBatch(ObiChainConstraintsBatch source = null)
        {
            return new ObiChainConstraintsBatch();
        }
    }
}
