// One incident dihedral hinge from the point of view of a single vertex, stored
// in the same CSR layout as VertexSpringEdge. The hinge's four canonical
// vertices are (i1,i2 = shared edge; i3,i4 = wings); `role` (0..3) says which of
// them *this* vertex is, so the per-vertex solve can pick out its own gradient.
public struct VertexBendElement
{
    public int i1, i2, i3, i4;
    public int role;
    public float restAngle;
    public float stiffness;
}
