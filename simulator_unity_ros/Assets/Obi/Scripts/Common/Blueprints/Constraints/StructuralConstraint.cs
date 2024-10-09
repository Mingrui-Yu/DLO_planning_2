using UnityEngine;
using System.Collections;

namespace Obi
{
    public struct StructuralConstraint
    {
        public IStructuralConstraintBatch batchIndex;
        public int constraintIndex;
        public float force;

        public float restLength
        {
            get 
            {
                if (batchIndex == null)
                    return -1;
                return batchIndex.GetRestLength(constraintIndex); 
            }

            set
            {
                if (batchIndex != null)
                {
                    batchIndex.SetRestLength(constraintIndex, value);
                }
            }
        }

        public StructuralConstraint(IStructuralConstraintBatch batchIndex, int constraintIndex, float force)
        {
            this.batchIndex = batchIndex;
            this.constraintIndex = constraintIndex;
            this.force = force;
        }
    }
}