using System;
using System.Collections.Generic;
using UnityEngine;

public class SpatialHash
{
    private readonly float spacing; // cell size - particles within this distance are neighbors
    private readonly int tableSize;
    private readonly int[] cellStart; // cellStart[i] is the index in cellEntries where the entries for cell i start
    private readonly int[] cellEntries;
    private readonly int[] queryIds; // array for the results of a single neighborhood query
    private int querySize;

    public int maxNumObjects;
    public readonly int[] firstAdjId;
    public readonly List<int> adjIds;

    public SpatialHash(float spacing, int maxNumObjects)
    {
        this.spacing = spacing;
        tableSize = 5 * maxNumObjects; // TODO: change this
        cellStart = new int[tableSize + 1];
        cellEntries = new int[maxNumObjects];
        queryIds = new int[maxNumObjects];

        this.maxNumObjects = maxNumObjects;
        firstAdjId = new int[maxNumObjects + 1];
        adjIds = new List<int>(10 * maxNumObjects);
    }

    private int HashCoords(int xi, int yi, int zi)
    {
        int h = (xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481);
        return Math.Abs(h) % tableSize;
    }

    private int IntCoord(float coord) => Mathf.FloorToInt(coord / spacing);

    private int HashPos(Vector3[] pos, int id) // returns the position of the particle in the hash table
    {
        Vector3 p = pos[id];
        return HashCoords(IntCoord(p.x), IntCoord(p.y), IntCoord(p.z));
    }

    public void Create(Vector3[] pos)
    {
        int numObjects = Math.Min(pos.Length, cellEntries.Length);

        Array.Clear(cellStart, 0, cellStart.Length);
        Array.Clear(cellEntries, 0, cellEntries.Length);

        for (int i = 0; i < numObjects; i++)
        {
            int h = HashPos(pos, i);
            cellStart[h]++;
        }

        int start = 0;
        for (int i = 0; i < tableSize; i++)
        {
            start += cellStart[i];
            cellStart[i] = start;
        }
        cellStart[tableSize] = start;

        for (int i = 0; i < numObjects; i++)
        {
            int h = HashPos(pos, i);
            cellStart[h]--;
            cellEntries[cellStart[h]] = i;
        }
    }

    private void Query(Vector3[] pos, int nr, float maxDist)
    {
        Vector3 p = pos[nr];
        int x0 = IntCoord(p.x - maxDist);
        int y0 = IntCoord(p.y - maxDist);
        int z0 = IntCoord(p.z - maxDist);

        int x1 = IntCoord(p.x + maxDist);
        int y1 = IntCoord(p.y + maxDist);
        int z1 = IntCoord(p.z + maxDist);

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

                    for (int i = start; i < end; i++)
                        queryIds[querySize++] = cellEntries[i];
                }
            }
        }
    }

    public void QueryAll(Vector3[] pos, float maxDist)
    {
        adjIds.Clear();
        float maxDist2 = maxDist * maxDist;

        for (int i = 0; i < maxNumObjects; i++)
        {
            firstAdjId[i] = adjIds.Count;
            Query(pos, i, maxDist);

            for (int j = 0; j < querySize; j++)
            {
                int id1 = queryIds[j];
                if (id1 >= i)
                    continue;

                if ((pos[i] - pos[id1]).sqrMagnitude > maxDist2)
                    continue;

                adjIds.Add(id1);
            }
        }

        firstAdjId[maxNumObjects] = adjIds.Count;
    }
}