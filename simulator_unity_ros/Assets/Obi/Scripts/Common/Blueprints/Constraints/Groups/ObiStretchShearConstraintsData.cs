using UnityEngine;
using System.Collections;
using System;

namespace Obi
{

    public interface IStretchShearConstraintsUser
    {
        bool stretchShearConstraintsEnabled
        {
            get;
            set;
        }

        float stretchCompliance
        {
            get;
            set;
        }

        float shear1Compliance
        {
            get;
            set;
        }

        float shear2Compliance
        {
            get;
            set;
        }

    }

    [Serializable]
    public class ObiStretchShearConstraintsData : ObiConstraints<ObiStretchShearConstraintsBatch>
    {

        public override ObiStretchShearConstraintsBatch CreateBatch(ObiStretchShearConstraintsBatch source = null)
        {
            return new ObiStretchShearConstraintsBatch();
        }
    }
}
