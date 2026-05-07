using UnityEngine;

public struct Spring
{
    public int p1Idx;
    public int p2Idx;
    public float restLength;
    public float stiffness;

    public Spring(int particle1Index, int particle2Index, float restLength, float stiffness)
    {
        p1Idx = particle1Index;
        p2Idx = particle2Index;
        this.restLength = restLength;
        this.stiffness = stiffness;
    }
}