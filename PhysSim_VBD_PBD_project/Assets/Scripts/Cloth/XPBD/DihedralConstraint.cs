// A dihedral bending constraint over a hinge: shared edge (p1,p2) with wing
// vertices p3 (triangle p1,p2,p3) and p4 (triangle p1,p2,p4). The constraint is
// C = angle(p1,p2,p3,p4) - restAngle, solved with XPBD compliance.
public struct DihedralConstraint
{
    public int p1Idx;
    public int p2Idx;
    public int p3Idx;
    public int p4Idx;
    public float restAngle;
    public float compliance;
}
