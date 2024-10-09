using UnityEngine;
using System.Collections;
using System;

namespace Obi
{
    public interface IDistanceConstraintsUser
    {
        bool distanceConstraintsEnabled
        {
            get;
            set;
        }

        float stretchingScale
        {
            get;
            set;
        }

        float stretchCompliance
        {
            get;
            set;
        }

        float maxCompression
        {
            get;
            set;
        }
    }

    [Serializable]
    public class ObiDistanceConstraintsData : ObiConstraints<ObiDistanceConstraintsBatch>
    {

        public override ObiDistanceConstraintsBatch CreateBatch(ObiDistanceConstraintsBatch source = null)
        {
            return new ObiDistanceConstraintsBatch();
        }
    }
}
