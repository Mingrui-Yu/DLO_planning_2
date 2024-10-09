#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Burst;

namespace Obi
{

    /**
     *  MultilevelGrid is the most used spatial partitioning structure in Obi. It is:
     *
     * - Unbounded: defines no limits on the size of location of space partitioned.
     * - Sparse: only allocates memory where space has interesting features to track.
     * - Multilevel: can store several levels of spatial subdivision, from very fine to very large.
     * - Implicit: the hierarchical relationship between cells is not stored in memory,
     *   but implicitly derived from the structure itself.
     *
     *  These characteristics make it extremely flexible, memory efficient, and fast. 
     *  Its implementation is also fairly simple and concise. 
     */
    public unsafe struct NativeMultilevelGrid<T> : IDisposable where T : struct, IEquatable<T>
    {

        public const float minSize = 0.01f; 

        /**
         * A cell in the multilevel grid. Coords are 4-dimensional, the 4th component is the grid level.
         */
        public struct Cell<K> where K : struct, IEquatable<K>
        {
            int4 coords;
            UnsafeList contents;

            public Cell(int4 coords)
            {
                this.coords = coords;
                contents = new UnsafeList(Allocator.Persistent);
            }

            public int4 Coords
            {
                get { return coords; }
            }

            public int Length
            {
                get { return contents.Length; }
            }

            public void* ContentsPointer
            {
                get { return contents.Ptr; }
            }

            public K this[int index]
            {
                get
                {
                    return UnsafeUtility.ReadArrayElement<K>(contents.Ptr, index);
                }
            }

            public void Add(K entity)
            {
                contents.Add(entity);
            }

            public bool Remove(K entity)
            {
                int index = contents.IndexOf<K>(entity);
                if (index >= 0)
                {
                    contents.RemoveAtSwapBack<K>(index);
                    return true;
                }
                return false;
            }

            public void Destroy()
            {
                contents.Dispose();
            }
        }

        public NativeHashMap<int4, int> grid;
        public NativeList<Cell<T>> usedCells;
        public NativeHashMap<int, int> populatedLevels;

        public NativeMultilevelGrid(int capacity, Allocator label)
        {
            grid = new NativeHashMap<int4, int>(capacity, label);
            usedCells = new NativeList<Cell<T>>(label);
            populatedLevels = new NativeHashMap<int, int>(10, label);
        }

        public int CellCount
        {
            get { return usedCells.Length; }
        }

        public void Clear()
        {
            grid.Clear();
            usedCells.Clear();
            populatedLevels.Clear();
        }

        public void Dispose()
        {
            grid.Dispose();
            usedCells.Dispose();
            populatedLevels.Dispose();
        }

        public int GetOrCreateCell(int4 cellCoords)
        {
            int cellIndex;
            if (grid.TryGetValue(cellCoords, out cellIndex))
            {
                return cellIndex;
            }
            else
            {
                grid.TryAdd(cellCoords, usedCells.Length);
                usedCells.Add(new Cell<T>(cellCoords));

                IncreaseLevelPopulation(cellCoords.w);

                return usedCells.Length - 1;
            }
        }

        public bool TryGetCellIndex(int4 cellCoords, out int cellIndex)
        {
            return grid.TryGetValue(cellCoords, out cellIndex);
        }

        public void RemoveEmpty()
        {
            // remove empty cells from the used cells list and the grid:
            for (int i = usedCells.Length - 1; i >= 0 ; --i)
            {
                if (usedCells[i].Length == 0)
                {
                    DecreaseLevelPopulation(usedCells[i].Coords.w);
                    grid.Remove(usedCells[i].Coords);
                    usedCells.RemoveAtSwapBack(i);
                }
            }

            // update grid indices:
            for (int i = 0; i < usedCells.Length; ++i)
            {
                grid.Remove(usedCells[i].Coords);
                grid.TryAdd(usedCells[i].Coords, i);
            }
        }

        public static int GridLevelForSize(float size)
        {
            // the magic number is 1/log(2)
            return (int)math.ceil(math.log(math.max(size,minSize)) * 1.44269504089f);
        }

        public static float CellSizeOfLevel(int level)
        {
            return math.exp2(level);
        }

        /**
         * Given a cell coordinate, returns the coordinates of the cell containing it in a superior level.
         */
        public static int4 GetParentCellCoords(int4 cellCoords, int level)
        {
            float decimation = CellSizeOfLevel(level - cellCoords[3]);
            int4 cell = (int4)math.floor((float4)cellCoords / decimation);
            cell[3] = level;
            return cell;
        }

        public void RemoveFromCells(BurstCellSpan span, T content)
        {
            for (int x = span.min[0]; x <= span.max[0]; ++x)
                for (int y = span.min[1]; y <= span.max[1]; ++y)
                    for (int z = span.min[2]; z <= span.max[2]; ++z)
                    {
                        int cellIndex;
                        if (TryGetCellIndex(new int4(x, y, z, span.level), out cellIndex))
                        {
                            var oldCell = usedCells[cellIndex];
                            oldCell.Remove(content);
                            usedCells[cellIndex] = oldCell;
                        }
                    }
        }

        public void AddToCells(BurstCellSpan span, T content)
        {
            for (int x = span.min[0]; x <= span.max[0]; ++x)
                for (int y = span.min[1]; y <= span.max[1]; ++y)
                    for (int z = span.min[2]; z <= span.max[2]; ++z)
                    {
                        int cellIndex = GetOrCreateCell(new int4(x, y, z, span.level));

                        var newCell = usedCells[cellIndex];
                        newCell.Add(content);
                        usedCells[cellIndex] = newCell;
                    }
        }

        public static void GetCellCoordsForBoundsAtLevel(NativeList<int4> coords, BurstAabb bounds, int level, int maxSize = 10)
        {
            coords.Clear();
            float cellSize = CellSizeOfLevel(level);

            int3 minCell = GridHash.Quantize(bounds.min.xyz, cellSize);
            int3 maxCell = GridHash.Quantize(bounds.max.xyz, cellSize);
            maxCell = minCell + math.min(maxCell - minCell, new int3(maxSize));

            int3 size = maxCell - minCell + new int3(1);

            coords.Capacity = size.x * size.y * size.z;

            // TODO: return some sort of iterator trough the cells, not a native array.
            for (int x = minCell[0]; x <= maxCell[0]; ++x)
            {
                for (int y = minCell[1]; y <= maxCell[1]; ++y)
                {
                    for (int z = minCell[2]; z <= maxCell[2]; ++z)
                    {
                        coords.Add(new int4(x, y, z, level));
                    }
                }
            }
        }

        private void IncreaseLevelPopulation(int level)
        {

            int population = 0;
            if (populatedLevels.TryGetValue(level, out population))
            {
                populatedLevels.Remove(level);
            }

            populatedLevels.TryAdd(level, population + 1);
        }

        private void DecreaseLevelPopulation(int level)
        {

            int population = 0;
            if (populatedLevels.TryGetValue(level, out population))
            {
                population--;
                populatedLevels.Remove(level);

                if (population > 0)
                {
                    populatedLevels.TryAdd(level, population);
                }
            }

        }
    }
}
#endif
