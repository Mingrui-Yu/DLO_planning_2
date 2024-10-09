using UnityEngine;
using System.Collections;
using System;

namespace Obi
{

    public interface IBendTwistConstraintsUser
    {
        bool bendTwistConstraintsEnabled
        {
            get;
            set;
        }

        float torsionCompliance
        {
            get;
            set;
        }

        float bend1Compliance
        {
            get;
            set;
        }

        float bend2Compliance
        {
            get;
            set;
        }
    }

    [Serializable]
    public class ObiBendTwistConstraintsData : ObiConstraints<ObiBendTwistConstraintsBatch>
    {

        public override ObiBendTwistConstraintsBatch CreateBatch(ObiBendTwistConstraintsBatch source = null)
        {
            return new ObiBendTwistConstraintsBatch();
        }
    }
}
