using UnityEngine;

public struct BendingConstraint
{
    public int p1Idx;
    public int p2Idx;
    public int p3Idx;
    public int p4Idx;
    public float restLength;

    public BendingConstraint(int p1, int p2, int p3, int p4, float restLength)
    {
        p1Idx = p1;
        p2Idx = p2;
        p3Idx = p3;
        p4Idx = p4;
        this.restLength = restLength;
    }
}
