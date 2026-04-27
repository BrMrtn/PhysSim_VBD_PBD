using UnityEngine;

public struct DistanceConstraint
{
    public int p1Idx;
    public int p2Idx;
    public float restLength;
    public float compliance;

    public DistanceConstraint(int particle1Index, int particle2Index, float restLength, float compliance)
    {
        p1Idx = particle1Index;
        p2Idx = particle2Index;
        this.restLength = restLength;
        this.compliance = compliance;
    }
}