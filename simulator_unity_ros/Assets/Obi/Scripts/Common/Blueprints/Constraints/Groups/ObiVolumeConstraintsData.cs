using UnityEngine;
using System.Collections;
using System;

namespace Obi
{

    public interface IVolumeConstraintsUser
    {
        bool volumeConstraintsEnabled
        {
            get;
            set;
        }

        float compressionCompliance
        {
            get;
            set;
        }

        float pressure
        {
            get;
            set;
        }
    }

    [Serializable]
    public class ObiVolumeConstraintsData : ObiConstraints<ObiVolumeConstraintsBatch>
    {
        public override ObiVolumeConstraintsBatch CreateBatch(ObiVolumeConstraintsBatch source = null)
        {
            return new ObiVolumeConstraintsBatch();
        }
    }
}
