#if (OBI_ONI_SUPPORTED)
using UnityEngine;
using System;
using System.Collections;

namespace Obi
{
    public class OniJobHandle : IObiJobHandle
    {
        public IntPtr pointer = IntPtr.Zero;

        public OniJobHandle(IntPtr pointer)
        {
            this.pointer = pointer;
        }

        public void Complete()
        {
            Oni.Complete(pointer);
        }
    }
}
#endif
