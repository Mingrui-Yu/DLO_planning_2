using UnityEngine;
using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    
    public static class GraphColoring
    {
        /**
         * General greedy graph coloring algorithm for constraints. Input:
         * - List of particle indices used by all constraints.
         * - List of per-constraint offsets of the first constrained particle in the previous array, with the total amount of particle indices in the last position.
         * 
         * The output is a color for each constraint. Constraints of the same color are guaranteed to not share any partices.
         * If particle order is important within each constraint, make sure to pass a copy for particleIndices, as the order is altered by this function.
         */

        public static int[] Colorize(int[] particleIndices, int[] constraintIndices)
        {
            int constrainCount = constraintIndices.Length - 1;
            if (constrainCount == 0)
                return new int[0];

            int[] colors = new int[constrainCount];
            bool[] availability = new bool[constrainCount];

            for (int i = 0; i < constrainCount; ++i)
            {
                // Sort particle indices for all constraints. This allows for efficient neighbour checks.
                Array.Sort(particleIndices, constraintIndices[i], constraintIndices[i + 1] - constraintIndices[i]);
                //particleIndices.Sort(constraintIndices[i], constraintIndices[i+1] - constraintIndices[i], Comparer<int>.Default);
                colors[i] = -1;
                availability[i] = true;
            }
                
            // For each constraint:
            for (int i = 0; i < constrainCount; ++i)
            {
                // Iterate over all other constraints:
                for (int j = 0; j < constrainCount; ++j)
                {
                    if (i == j) continue;

                    // Check if the constraints share any particle:
                    int sizeI = constraintIndices[i + 1] - constraintIndices[i];
                    int sizeJ = constraintIndices[j + 1] - constraintIndices[j];
                    int counterI = 0;
                    int counterJ = 0;
                    while (counterI < sizeI && counterJ < sizeJ)
                    {
                        int p1 = particleIndices[constraintIndices[i] + counterI];
                        int p2 = particleIndices[constraintIndices[j] + counterJ];

                        if (p1 > p2) counterJ++;
                        else if (p1 < p2) counterI++;
                        else
                        {
                            // Mark the neighbour color as unavailable:
                            if (colors[j] >= 0)
                                availability[colors[j]] = false;
                            break;
                        }
                    }
                }

                // Assign the first available color:
                for (colors[i] = 0; colors[i] < constrainCount; ++colors[i])
                    if (availability[colors[i]])
                        break;

                // Reset availability flags:
                for (int j = 0; j < constrainCount; ++j)
                    availability[j] = true;
            }

            return colors;
        }


    }
}
