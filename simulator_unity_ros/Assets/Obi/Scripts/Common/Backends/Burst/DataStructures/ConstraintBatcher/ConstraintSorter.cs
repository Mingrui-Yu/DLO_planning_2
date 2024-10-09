#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Burst;
using Unity.Jobs;

namespace Obi
{
    public static class ConstraintSorter
    {

        public struct ConstraintComparer<K> : IComparer<K> where K : IConstraint
        {
            // Compares by Height, Length, and Width.
            public int Compare(K x, K y)
            {
                return x.GetParticle(1).CompareTo(y.GetParticle(1));
            }
        }

        /**
         * Performs a single-threaded count sort on the constraints array using the first particle index,
         * then multiple parallel sorts over slices of the original array sorting by the second particle index.
         */
        public static JobHandle SortConstraints<T>(int particleCount,
                                                    NativeArray<T> constraints,
                                                    ref NativeArray<T> sortedConstraints,
                                                    JobHandle handle) where T : struct, IConstraint
        {
            // Count the amount of digits in the largest particle index that can be referenced by a constraint:
            NativeArray<int> totalCountUpToDigit = new NativeArray<int>(particleCount + 1, Allocator.TempJob);
            int numDigits = 0;
            int maxBodyIndex = particleCount - 1;
            {
                int val = maxBodyIndex;
                while (val > 0)
                {
                    val >>= 1;
                    numDigits++;
                }
            }

            handle = new CountSortPerFirstParticleJob<T>
            {
                input = constraints,
                output = sortedConstraints,
                maxDigits = numDigits,
                maxIndex = maxBodyIndex,
                digitCount = totalCountUpToDigit
            }.Schedule(handle);

            // Sort sub arrays with default sort.
            int numPerBatch = math.max(1, maxBodyIndex / 32);

            handle = new SortSubArraysJob<T>
            {
                InOutArray = sortedConstraints,
                NextElementIndex = totalCountUpToDigit,
                comparer =  new ConstraintComparer<T>()
            }.Schedule(totalCountUpToDigit.Length, numPerBatch, handle);

            return handle;
        }

        [BurstCompile]
        public struct CountSortPerFirstParticleJob<K> : IJob where K : struct, IConstraint
        {
            [ReadOnly][NativeDisableContainerSafetyRestriction] public NativeArray<K> input;
            public NativeArray<K> output;

            [NativeDisableContainerSafetyRestriction] public NativeArray<int> digitCount;

            public int maxDigits;
            public int maxIndex;

            public void Execute()
            {
                // no real need for a mask, just in case bad particle indices were passed that have more digits than maxDigits.
                int mask = (1 << maxDigits) - 1;

                // Count digits
                for (int i = 0; i < input.Length; i++)
                {
                    digitCount[input[i].GetParticle(0) & mask]++;
                }

                // Calculate start index for each digit
                int prev = digitCount[0];
                digitCount[0] = 0;
                for (int i = 1; i <= maxIndex; i++)
                {
                    int current = digitCount[i];
                    digitCount[i] = digitCount[i - 1] + prev;
                    prev = current;
                }

                // Copy elements into buckets based on particle index
                for (int i = 0; i < input.Length; i++)
                {
                    int index = digitCount[input[i].GetParticle(0) & mask]++;
                    if (index == 1 && input.Length == 1)
                    {
                        output[0] = input[0];
                    }
                    output[index] = input[i];
                }
            }
        }

        // Sorts slices of an array in parallel
        [BurstCompile]
        public struct SortSubArraysJob<K> : IJobParallelFor where K : struct, IConstraint
        {
            [NativeDisableContainerSafetyRestriction] public NativeArray<K> InOutArray;

            // Typically lastDigitIndex is resulting RadixSortPerBodyAJob.digitCount. nextElementIndex[i] = index of first element with bodyA index == i + 1
            [NativeDisableContainerSafetyRestriction][DeallocateOnJobCompletion] public NativeArray<int> NextElementIndex;

            [ReadOnly] public ConstraintComparer<K> comparer;

            public void Execute(int workItemIndex)
            {
                int startIndex = 0;
                if (workItemIndex > 0)
                {
                    startIndex = NextElementIndex[workItemIndex - 1];
                }

                if (startIndex < InOutArray.Length)
                {
                    int length = NextElementIndex[workItemIndex] - startIndex;
                    DefaultSortOfSubArrays(InOutArray, startIndex, length, comparer);
                }
            }

            public static void DefaultSortOfSubArrays(NativeArray<K> inOutArray, int startIndex, int length, ConstraintComparer<K> comparer)
            {
                if (length > 2)
                {
                    var slice = inOutArray.Slice(startIndex, length);
                    slice.Sort(comparer);
                }
                else if (length == 2) // just swap:
                {
                    if (inOutArray[startIndex].GetParticle(1) > inOutArray[startIndex + 1].GetParticle(1))
                    {
                        var temp = inOutArray[startIndex + 1];
                        inOutArray[startIndex + 1] = inOutArray[startIndex];
                        inOutArray[startIndex] = temp;
                    }
                }
            }
        }

    }
}
#endif