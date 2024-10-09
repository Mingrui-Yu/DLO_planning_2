#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using UnityEngine;
using Unity.Burst;
using Unity.Jobs;
using Unity.Collections;
using System.Collections;

namespace Obi
{
    [BurstCompile]
    public struct DequeueIntoArrayJob<T> : IJob where T : struct
    {
        public int StartIndex;
        public NativeQueue<T> InputQueue;
        [WriteOnly] public NativeArray<T> OutputArray;

        public void Execute()
        {
            int queueCount = InputQueue.Count;

            for (int i = StartIndex; i < StartIndex + queueCount; i++)
            {
                OutputArray[i] = InputQueue.Dequeue();
            }
        }
    }
}
#endif