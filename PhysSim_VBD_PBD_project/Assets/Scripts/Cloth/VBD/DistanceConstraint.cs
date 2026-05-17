// Per-vertex view of a spring: stores the OTHER endpoint plus its rest length
// and stiffness. VBDSolver's Gauss-Seidel pass walks these directly out of a
// CSR (springListStart + springEdges), so each spring is duplicated once per
// endpoint. That trades 2x memory for: no indirection into a global springs[]
// table, no branch on which endpoint is the current vertex, and sequential
// per-vertex access.
public struct VertexSpringEdge
{
    public int otherIdx;
    public float restLength;
    public float stiffness;
}
