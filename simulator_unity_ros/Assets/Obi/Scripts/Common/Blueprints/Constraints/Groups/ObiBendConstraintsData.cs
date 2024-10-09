using UnityEngine;
using System.Collections;
using System;

namespace Obi
{
    public interface IBendConstraintsUser
    {
        bool bendConstraintsEnabled
        {
            get;
            set;
        }

        float bendCompliance
        {
            get;
            set;
        }

        float maxBending
        {
            get;
            set;
        }

    }

    [Serializable]
    public class ObiBendConstraintsData : ObiConstraints<ObiBendConstraintsBatch>
    {
        public override ObiBendConstraintsBatch CreateBatch(ObiBendConstraintsBatch source = null)
        {
            return new ObiBendConstraintsBatch();
        }
    }
}
