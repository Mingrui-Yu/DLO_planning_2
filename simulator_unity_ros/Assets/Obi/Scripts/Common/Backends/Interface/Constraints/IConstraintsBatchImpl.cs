
using UnityEngine;
using System.Collections;

namespace Obi
{
    public interface IConstraintsBatchImpl
    {
        Oni.ConstraintType constraintType
        {
            get;
        }

        IConstraints constraints
        {
            get;
        }

        bool enabled
        {
            set;
            get;
        }

        void Destroy();
        void SetConstraintCount(int constraintCount);
        int GetConstraintCount();
    }
}
