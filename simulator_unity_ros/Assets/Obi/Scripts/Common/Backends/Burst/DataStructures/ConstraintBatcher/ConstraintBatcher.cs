#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Burst;
using Unity.Jobs;

namespace Obi
{
    public struct BatchData
    {
        public ushort batchID;             // Batch identifier. All bits will be '0', except for the one at the position of the batch.

        public int startIndex;             // first constraint in the batch
        public int constraintCount;        // amount of constraints in the batch.
        public int activeConstraintCount;  // auxiliar counter used to sort the constraints in linear time.

        public int workItemSize;           // size of each work item.
        public int workItemCount;          // number of work items.
        public bool isLast;

        public BatchData(int index, int maxBatches)
        {
            batchID = (ushort)(1 << index);
            isLast = index == (maxBatches - 1);
            constraintCount = 0;
            activeConstraintCount = 0;

            startIndex = 0;
            workItemSize = 0;
            workItemCount = 0;
        }

        public void GetConstraintRange(int workItemIndex, out int start, out int end)
        {
            start = startIndex + workItemSize * workItemIndex;
            end = startIndex + math.min(constraintCount, workItemSize * (workItemIndex + 1));
        }

    }

    public struct ConstraintBatcher : IDisposable 
    {
        public const int minWorkItemSize = 8;

        public int maxBatches;
        private BatchLUT batchLUT; // look up table for batch indices.

        public ConstraintBatcher(int maxBatches) //17 and 8
        {
            this.maxBatches = math.min(17,maxBatches);
            batchLUT = new BatchLUT(this.maxBatches);
        }

        public void Dispose()
        {
            batchLUT.Dispose();
        }

        private unsafe struct WorkItem
        {
            public fixed int constraints[minWorkItemSize];
            public int constraintCount;

            public bool Add(int constraintIndex)
            {
                // add the constraint to this work item.
                fixed (int* constraintIndices = constraints)
                {
                    constraintIndices[constraintCount] = constraintIndex;
                }

                // if we've completed the work item, close it and reuse for the next one.
                return (++constraintCount == minWorkItemSize);
            }

        }

        /**
         * Linear-time graph coloring using bitmasks and a look-up table. Used to organize contacts into batches for parallel processing.
         * input: array of unsorted constraints.
         * output:
         * - sorted constraints array.
         * - array of batchData, one per batch: startIndex, batchSize, workItemSize (at most == batchSize), numWorkItems
         * - number of active batches.
         */

        public  JobHandle BatchConstraints<T>(NativeArray<T> contacts,
                                            int particleCount,
                                            ref NativeArray<T> sortedContacts,
                                            ref NativeArray<BatchData> batchData,
                                            ref NativeArray<int> activeBatchCount,
                                            JobHandle inputDeps) where T : struct, IConstraint
        {
            if (sortedContacts.Length != contacts.Length || activeBatchCount.Length != 1)
                return inputDeps;

            var batchJob = new BatchContactsJob<T>()
            {
                batchMasks = new NativeArray<ushort>(particleCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
                batchIndices = new NativeArray<int>(contacts.Length, Allocator.TempJob, NativeArrayOptions.ClearMemory),
                lut = batchLUT,
                contacts = contacts,
                batchData = batchData,
                sortedContacts = sortedContacts,
                activeBatchCount = activeBatchCount,
                maxBatches = maxBatches
            };

            return batchJob.Schedule(inputDeps);
        }

        [BurstCompile]
        private struct BatchContactsJob<K> : IJob where K : struct, IConstraint
        {
            [DeallocateOnJobCompletion]
            public NativeArray<ushort> batchMasks;

            [DeallocateOnJobCompletion]
            public NativeArray<int> batchIndices;

            [ReadOnly] public BatchLUT lut;
            [ReadOnly] public NativeArray<K> contacts;
            public NativeArray<BatchData> batchData;
            public NativeArray<K> sortedContacts;

            public NativeArray<int> activeBatchCount;

            public int maxBatches;

            public unsafe void Execute()
            {
                // Initialize batch data array
                for (int i = 0; i < batchData.Length; ++i)
                    batchData[i] = new BatchData(i, maxBatches);

                // temporary array containing an open work item for each batch.
                WorkItem* workItems = stackalloc WorkItem[maxBatches];
                for (int i = 0; i < maxBatches; i++)
                    workItems[i] = new WorkItem();

                // find a batch for each constraint:
                for (int i = 0; i < contacts.Length; ++i)
                {
                    // OR together the batch masks of all entities involved in the constraint:
                    int batchMask = batchMasks[contacts[i].GetParticle(0)] | batchMasks[contacts[i].GetParticle(1)];

                    // look up the first free batch index for this constraint:
                    int batchIndex = batchIndices[i] = lut.batchIndex[batchMask];

                    // update the amount of constraints in the batch:
                    var batch = batchData[batchIndex];
                    batch.constraintCount++;
                    batchData[batchIndex] = batch;

                    // add the constraint to the last work item of the batch:
                    if (workItems[batchIndex].Add(i))
                    {
                        // if this work item does not belong to the last batch:
                        if (batchIndex != maxBatches - 1)
                        {
                            // tag all entities in the work item with the batch mask to close it.
                            // this way we know constraints referencing any of these entities can no longer be added to this batch.
                            for (int j = 0; j < workItems[batchIndex].constraintCount; j++)
                            {
                                K contact = contacts[workItems[batchIndex].constraints[j]];
                                batchMasks[contact.GetParticle(0)] |= batch.batchID;
                                batchMasks[contact.GetParticle(1)] |= batch.batchID;
                            }
                        }

                        // reuse the work item.
                        workItems[batchIndex].constraintCount = 0;
                    }

                }

                // fill batch data:
                activeBatchCount[0] = 0;
                int numConstraints = 0;
                for (int i = 0; i < batchData.Length; ++i)
                {
                    var batch = batchData[i];

                    // bail out when we find the first empty batch:
                    if (batch.constraintCount == 0)
                        break;

                    // calculate work item size, count, and index of the first constraint
                    batch.workItemSize = math.min(minWorkItemSize, batch.constraintCount);
                    batch.workItemCount = (batch.constraintCount + batch.workItemSize - 1) / batch.workItemSize;
                    batch.startIndex = numConstraints;

                    numConstraints += batch.constraintCount;
                    activeBatchCount[0]++;

                    batchData[i] = batch;
                }

                // write out constraints, sorted according to batches:
                for (int i = 0; i < contacts.Length; ++i)
                {
                    var batch = batchData[batchIndices[i]];
                    int sortedIndex = batch.startIndex + (batch.activeConstraintCount++);
                    sortedContacts[sortedIndex] = contacts[i];
                    batchData[batchIndices[i]] = batch;
                }

            }

        
        }

    }
}
#endif