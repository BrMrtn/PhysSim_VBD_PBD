using UnityEngine;

public struct AreaSample
{
    public float current;
    public float rest;
    public float Ratio => rest > 0f ? current / rest : 0f;
}

// Area is computed straight from the grid layout (numX, numY)
public static class AreaSampler
{
    public static AreaSample Sample(Vector3[] positions, Vector3[] restPositions, int numX, int numY)
    {
        return new AreaSample
        {
            current = GridArea(positions, numX, numY),
            rest = GridArea(restPositions, numX, numY),
        };
    }

    private static float GridArea(Vector3[] p, int numX, int numY)
    {
        float area = 0f;
        for (int iy = 0; iy < numY - 1; iy++)
            for (int ix = 0; ix < numX - 1; ix++)
            {
                int a = iy * numX + ix;
                int b = iy * numX + (ix + 1);
                int c = (iy + 1) * numX + ix;
                int d = (iy + 1) * numX + (ix + 1);

                area += TriangleArea(p[a], p[b], p[d]) + TriangleArea(p[a], p[d], p[c]);
            }
        return area;
    }

    private static float TriangleArea(Vector3 p0, Vector3 p1, Vector3 p2)
        => 0.5f * Vector3.Cross(p1 - p0, p2 - p0).magnitude;
}
