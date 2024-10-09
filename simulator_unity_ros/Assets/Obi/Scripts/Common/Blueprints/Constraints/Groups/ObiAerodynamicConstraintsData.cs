using UnityEngine;
using System.Collections;
using System;

namespace Obi
{

    public interface IAerodynamicConstraintsUser
    {
        bool aerodynamicsEnabled
        {
            get;
            set;
        }

        float drag
        {
            get;
            set;
        }

        float lift
        {
            get;
            set;
        }

    }

    [Serializable]
    public class ObiAerodynamicConstraintsData : ObiConstraints<ObiAerodynamicConstraintsBatch>
    {
        public override ObiAerodynamicConstraintsBatch CreateBatch(ObiAerodynamicConstraintsBatch source = null)
        {
            return new ObiAerodynamicConstraintsBatch();
        }
    }
}
