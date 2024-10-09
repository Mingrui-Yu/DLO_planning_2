using UnityEngine;
using System.Collections;
using System;

namespace Obi
{

    public interface ITetherConstraintsUser
    {
        bool tetherConstraintsEnabled
        {
            get;
            set;
        }

        float tetherCompliance
        {
            get;
            set;
        }

        float tetherScale
        {
            get;
            set;
        }
    }


    [Serializable]
    public class ObiTetherConstraintsData : ObiConstraints<ObiTetherConstraintsBatch>
    {
        public override ObiTetherConstraintsBatch CreateBatch(ObiTetherConstraintsBatch source = null)
        {
            return new ObiTetherConstraintsBatch();
        }
    }
}
