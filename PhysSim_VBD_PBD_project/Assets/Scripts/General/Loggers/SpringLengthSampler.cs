using UnityEngine;

// Per-spring snapshot of a chain. Spring i is the stretching spring between
// vertex i and vertex i+1; currentLengths[i] is its present length and
// restLengths[i] its rest length, so currentLengths[i] / restLengths[i] is the
// stretch ratio used to see how far a spring expands under high mass ratios.
public struct SpringLengthSample
{
    public float[] currentLengths;
    public float[] restLengths;
    public int count;
}

// Bending constraints are ignored: only the consecutive (i, i+1) stretching
// springs are counted.
public static class SpringLengthSampler
{
    public static SpringLengthSample Sample(XPBDSolver s)
    {
        var r = NewSample(s.numVerts);
        FillCurrentLengths(s.positions, r);

        var cons = s.constraints;
        if (cons != null)
            for (int i = 0; i < cons.Length; i++)
            {
                ref var con = ref cons[i];
                if (con.p2Idx - con.p1Idx == 1) // stretching spring between adjacent vertices
                    r.restLengths[con.p1Idx] = con.restLength;
            }

        return r;
    }

    public static SpringLengthSample Sample(VBDSolver s) =>
        SampleCsr(s.positions, s.numVerts, s.springListStart, s.springEdges);

    public static SpringLengthSample Sample(NewtonSolver s) =>
        SampleCsr(s.positions, s.numVerts, s.springListStart, s.springEdges);

    // Cloth grid: samples the structural (stretching) springs connecting
    // grid-adjacent vertices, horizontally (ix, ix+1) and vertically
    // (iy, iy+1). Shear and bending springs are ignored, mirroring the chain
    // sampler which counts only stretching springs. Rest lengths come from
    // restPositions, so the result is solver-agnostic and works for both XPBD
    // and VBD cloth. Springs are laid out horizontals-first, then verticals.
    public static SpringLengthSample SampleClothGrid(Vector3[] positions, Vector3[] restPositions, int numX, int numY)
    {
        int horizontal = numY * Mathf.Max(0, numX - 1);
        int vertical = numX * Mathf.Max(0, numY - 1);

        var r = new SpringLengthSample
        {
            count = horizontal + vertical,
            currentLengths = new float[horizontal + vertical],
            restLengths = new float[horizontal + vertical],
        };

        int k = 0;
        for (int iy = 0; iy < numY; iy++)
            for (int ix = 0; ix < numX - 1; ix++)
            {
                int a = iy * numX + ix;
                int b = iy * numX + (ix + 1);
                r.currentLengths[k] = (positions[b] - positions[a]).magnitude;
                r.restLengths[k] = (restPositions[b] - restPositions[a]).magnitude;
                k++;
            }

        for (int iy = 0; iy < numY - 1; iy++)
            for (int ix = 0; ix < numX; ix++)
            {
                int a = iy * numX + ix;
                int b = (iy + 1) * numX + ix;
                r.currentLengths[k] = (positions[b] - positions[a]).magnitude;
                r.restLengths[k] = (restPositions[b] - restPositions[a]).magnitude;
                k++;
            }

        return r;
    }

    private static SpringLengthSample SampleCsr(Vector3[] positions, int numVerts,
        int[] springListStart, VertexSpringEdge[] springEdges)
    {
        var r = NewSample(numVerts);
        FillCurrentLengths(positions, r);

        for (int i = 0; i < numVerts - 1; i++)
        {
            int start = springListStart[i];
            int end = springListStart[i + 1];
            for (int e = start; e < end; e++)
                if (springEdges[e].otherIdx == i + 1) // stretching spring to the next vertex
                {
                    r.restLengths[i] = springEdges[e].restLength;
                    break;
                }
        }

        return r;
    }

    private static SpringLengthSample NewSample(int numVerts)
    {
        int springs = Mathf.Max(0, numVerts - 1);
        return new SpringLengthSample
        {
            count = springs,
            currentLengths = new float[springs],
            restLengths = new float[springs],
        };
    }

    private static void FillCurrentLengths(Vector3[] positions, SpringLengthSample r)
    {
        for (int i = 0; i < r.count; i++)
            r.currentLengths[i] = (positions[i + 1] - positions[i]).magnitude;
    }
}
