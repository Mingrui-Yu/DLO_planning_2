#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using UnityEngine;
using Unity.Jobs;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Burst;
using System;
using System.Collections;

namespace Obi
{

    [BurstCompile]
    public struct ApplyBatchedCollisionConstraintsBatchJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<BurstContact> contacts;

        [NativeDisableParallelForRestriction] public NativeArray<float4> positions;
        [NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
        [NativeDisableParallelForRestriction] public NativeArray<int> counts;

        [NativeDisableParallelForRestriction] public NativeArray<quaternion> orientations;
        [NativeDisableParallelForRestriction] public NativeArray<quaternion> orientationDeltas;
        [NativeDisableParallelForRestriction] public NativeArray<int> orientationCounts;

        [ReadOnly] public Oni.ConstraintParameters constraintParameters;
        [ReadOnly] public BatchData batchData;

        public void Execute(int workItemIndex)
        {
            int start, end;
            batchData.GetConstraintRange(workItemIndex, out start, out end);

            for (int i = start; i < end; ++i)
            {
                BurstConstraintsBatchImpl.ApplyPositionDelta(contacts[i].entityA, constraintParameters.SORFactor, ref positions, ref deltas, ref counts);
                BurstConstraintsBatchImpl.ApplyPositionDelta(contacts[i].entityB, constraintParameters.SORFactor, ref positions, ref deltas, ref counts);

                BurstConstraintsBatchImpl.ApplyOrientationDelta(contacts[i].entityA, constraintParameters.SORFactor, ref orientations, ref orientationDeltas, ref orientationCounts);
                BurstConstraintsBatchImpl.ApplyOrientationDelta(contacts[i].entityB, constraintParameters.SORFactor, ref orientations, ref orientationDeltas, ref orientationCounts);
            }
            
        }

    }
}
#endif