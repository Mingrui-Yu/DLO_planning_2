#if (OBI_ONI_SUPPORTED)
using System;
using System.Collections;
using System.Runtime.InteropServices;

namespace Obi
{
    public class OniConstraintsBatchImpl : IConstraintsBatchImpl
    {
        protected IConstraints m_Constraints;
        protected Oni.ConstraintType m_ConstraintType;
        protected IntPtr m_OniBatch;
        protected bool m_Enabled;

        public IntPtr oniBatch
        {
            get { return m_OniBatch; }
        }

        public Oni.ConstraintType constraintType
        {
            get { return m_ConstraintType; }
        }

        public IConstraints constraints
        {
            get { return m_Constraints; } 
        }

        public bool enabled
        {
            set
            {
                if (m_Enabled != value)
                {
                    m_Enabled = value;
                    Oni.EnableBatch(m_OniBatch, m_Enabled);
                }
            }
            get { return m_Enabled; }
        }

        public OniConstraintsBatchImpl(IConstraints constraints, Oni.ConstraintType type)
        {
            this.m_Constraints = constraints;
            this.m_ConstraintType = type;

            m_OniBatch = Oni.CreateBatch((int)type);
        }

        public void Destroy()
        {
            //Oni.DestroyBatch(m_OniBatch);

            // remove the constraint batch from the solver 
            // (no need to destroy it as its destruction is managed by the solver)
            // just reset the reference.
            m_OniBatch = IntPtr.Zero;
        }

        public void SetConstraintCount(int constraintCount)
        {
            Oni.SetBatchConstraintCount(m_OniBatch, constraintCount);
        }

        public int GetConstraintCount()
        {
            return Oni.GetBatchConstraintCount(m_OniBatch);
        }

    }
}
#endif