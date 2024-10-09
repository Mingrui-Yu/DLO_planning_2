#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using UnityEngine;
using Unity.Jobs;
using System;
using System.Collections;

namespace Obi
{
    public class BurstJobHandle : IObiJobHandle
    {
        public JobHandle handle = new JobHandle();

        public BurstJobHandle(JobHandle handle)
        {
            this.handle = handle;
        }

        public void Complete()
        {
            handle.Complete();
        }
    }
}
#endif

