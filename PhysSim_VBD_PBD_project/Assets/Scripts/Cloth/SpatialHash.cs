using System;
using UnityEngine;

public class SpatialHash
{
    private readonly float spacing; // cell size - particles within this distance are neighbors
    private readonly int tableSize;
    private readonly int[] cellStart; // cellStart[i] is the index in cellEntries where the entries for cell i start
    private readonly int[] cellEntries;
    private int[] queryIds; // array for the results of a single neighborhood query (grown on overflow)
    private int querySize;

    public int maxNumObjects;
    public readonly int[] firstAdjId;
    public int[] adjIds;
    public int[] adjIdsSym; // Symmetric adjacency CSR for VBD

    public SpatialHash(float spacing, int maxNumObjects)
    {
        this.spacing = spacing;
        tableSize = 5 * maxNumObjects; // TODO: change this
        cellStart = new int[tableSize + 1];
        cellEntries = new int[maxNumObjects];
        queryIds = new int[maxNumObjects];

        this.maxNumObjects = maxNumObjects;
        firstAdjId = new int[maxNumObjects + 1];
        adjIds = new int[10 * maxNumObjects];
        adjIdsSym = new int[10 * maxNumObjects];
    }

    private int HashCoords(int xi, int yi, int zi)
    {
        int h = (xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481);
        return Math.Abs(h) % tableSize;
    }

    private int IntCoord(float coord) => Mathf.FloorToInt(coord / spacing); // particles in the same cell will have the same integer coordinates

    private int HashPos(Vector3 pos) // returns the position of the particle in the hash table
    {
        return HashCoords(IntCoord(pos.x), IntCoord(pos.y), IntCoord(pos.z));
    }

    public void Create(Vector3[] pos)
    {
        int numObjects = Math.Min(pos.Length, cellEntries.Length);

        Array.Clear(cellStart, 0, cellStart.Length);
        Array.Clear(cellEntries, 0, cellEntries.Length);

        for (int i = 0; i < numObjects; i++)
        {
            int h = HashPos(pos[i]);
            cellStart[h]++; // count how many particles are in each cell
        }

        int start = 0;
        for (int i = 0; i < tableSize; i++)
        {
            start += cellStart[i];
            cellStart[i] = start; // cellStart[i] and cellStart[i+1] are now the indices in cellEntries where the entries for cell i start and end
        }
        cellStart[tableSize] = start; // guard entry for iteration

        for (int i = 0; i < numObjects; i++)
        {
            int h = HashPos(pos[i]);
            cellStart[h]--;
            cellEntries[cellStart[h]] = i; // particles in the same cell are next to each other
        }
    }

    private void Query(Vector3 pos, float maxDist) // query a block of cells around the position
    {
        int x0 = IntCoord(pos.x - maxDist);
        int y0 = IntCoord(pos.y - maxDist);
        int z0 = IntCoord(pos.z - maxDist);
        
        int x1 = IntCoord(pos.x + maxDist);
        int y1 = IntCoord(pos.y + maxDist);
        int z1 = IntCoord(pos.z + maxDist);
        querySize = 0;

        for (int xi = x0; xi <= x1; xi++)
        {
            for (int yi = y0; yi <= y1; yi++)
            {
                for (int zi = z0; zi <= z1; zi++)
                {
                    int h = HashCoords(xi, yi, zi);
                    int start = cellStart[h];
                    int end = cellStart[h + 1];

                    // The same table bucket can be reached by several grid
                    // coordinates in this block (modular hash collisions), and
                    // a single dense bucket can hold many particles, so the raw
                    // entry count may exceed maxNumObjects. Grow as needed.
                    for (int i = start; i < end; i++)
                    {
                        if (querySize >= queryIds.Length)
                            Array.Resize(ref queryIds, queryIds.Length * 2);
                        queryIds[querySize++] = cellEntries[i];
                    }
                }
            }
        }
    }

    public void QueryAll(Vector3[] pos, float maxDist)
    {
        float maxDist2 = maxDist * maxDist;
        int num = 0;

        for (int i = 0; i < maxNumObjects; i++)
        {
            firstAdjId[i] = num; // store where i's neighbors will start in adjIds
            Query(pos[i], maxDist);

            for (int j = 0; j < querySize; j++)
            {
                int id1 = queryIds[j];
                if (id1 >= i)
                    continue;

                if ((pos[i] - pos[id1]).sqrMagnitude > maxDist2)
                    continue;

                if (num >= adjIds.Length)
                    Array.Resize(ref adjIds, adjIds.Length * 2);

                adjIds[num++] = id1;
            }
        }

        firstAdjId[maxNumObjects] = num;
    }

    // Symmetric variant: each particle's neighbor list contains every other
    // particle within maxDist, regardless of id ordering.
    public void QueryAllSymmetric(Vector3[] pos, float maxDist)
    {
        float maxDist2 = maxDist * maxDist;
        int writeIdx = 0;

        for (int i = 0; i < maxNumObjects; i++)
        {
            firstAdjId[i] = writeIdx;
            Query(pos[i], maxDist);

            for (int j = 0; j < querySize; j++)
            {
                int id1 = queryIds[j];
                if (id1 == i)
                    continue;

                if ((pos[i] - pos[id1]).sqrMagnitude > maxDist2)
                    continue;

                if (writeIdx >= adjIdsSym.Length)
                    Array.Resize(ref adjIdsSym, adjIdsSym.Length * 2);

                adjIdsSym[writeIdx++] = id1;
            }
        }

        firstAdjId[maxNumObjects] = writeIdx;
    }
}