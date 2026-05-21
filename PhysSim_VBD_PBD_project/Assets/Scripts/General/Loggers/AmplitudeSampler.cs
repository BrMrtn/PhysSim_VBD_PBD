using UnityEngine;

public struct AmplitudeSample
{
    public Vector3 endPosition;
    public Vector3 anchorPosition;
}

public static class AmplitudeSampler
{
    public static AmplitudeSample Sample(XPBDSolver s) => Sample(s.positions, s.numVerts);
    public static AmplitudeSample Sample(VBDSolver s) => Sample(s.positions, s.numVerts);
    public static AmplitudeSample Sample(NewtonSolver s) => Sample(s.positions, s.numVerts);

    private static AmplitudeSample Sample(Vector3[] positions, int numVerts)
    {
        AmplitudeSample r = default;
        r.anchorPosition = positions[0];
        r.endPosition = positions[numVerts - 1];
        return r;
    }
}
