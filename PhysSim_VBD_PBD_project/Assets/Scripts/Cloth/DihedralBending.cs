using UnityEngine;

// Shared geometric kernel for dihedral bending. A hinge is two triangles
// (x1,x2,x3) and (x1,x2,x4) sharing edge (x1,x2); x3 and x4 are the two wing
// vertices. Given the four positions this returns the dihedral angle C =
// acos(n1 . n2) and the gradient of that angle with respect to each vertex
// (dC/dxi), following Mueller et al. 2007 "Position Based Dynamics", App. A.
//
// Both solvers consume the *same* gradients; they differ only in what they
// build from them - XPBD forms a single scalar Lagrange multiplier, VBD forms a
// per-vertex force and a local 3x3 Hessian. Keeping the geometry here lets the
// architectural difference between the two solvers stand out on its own.
public static class DihedralBending
{
    // Returns false for a degenerate configuration (a zero-area triangle, where
    // the normal - and hence the angle gradient - is undefined).
    public static bool ComputeGradients(
        Vector3 x1, Vector3 x2, Vector3 x3, Vector3 x4,
        out float angle,
        out Vector3 g1, out Vector3 g2, out Vector3 g3, out Vector3 g4)
    {
        angle = 0f;
        g1 = g2 = g3 = g4 = Vector3.zero;

        // Work relative to x1.
        Vector3 p2 = x2 - x1;
        Vector3 p3 = x3 - x1;
        Vector3 p4 = x4 - x1;

        Vector3 n1 = Vector3.Cross(p2, p3);
        Vector3 n2 = Vector3.Cross(p2, p4);
        float len1 = n1.magnitude;
        float len2 = n2.magnitude;
        if (len1 < 1e-12f || len2 < 1e-12f) return false;

        n1 /= len1;
        n2 /= len2;

        // d = cos(angle). Clamp away from +/-1 so the 1/sqrt(1-d^2) chain-rule
        // factor below stays finite at a flat or fully folded hinge.
        float d = Mathf.Clamp(Vector3.Dot(n1, n2), -1f + 1e-6f, 1f - 1e-6f);
        angle = Mathf.Acos(d);

        // qi = d(d)/d(xi): gradients of the cosine of the dihedral angle.
        Vector3 q3 = (Vector3.Cross(p2, n2) + Vector3.Cross(n1, p2) * d) / len1;
        Vector3 q4 = (Vector3.Cross(p2, n1) + Vector3.Cross(n2, p2) * d) / len2;
        Vector3 q2 = -(Vector3.Cross(p3, n2) + Vector3.Cross(n1, p3) * d) / len1
                     - (Vector3.Cross(p4, n1) + Vector3.Cross(n2, p4) * d) / len2;
        Vector3 q1 = -q2 - q3 - q4;

        // Chain rule: d(angle)/d(xi) = -1/sqrt(1-d^2) * qi.
        float scale = -1f / Mathf.Sqrt(1f - d * d);
        g1 = scale * q1;
        g2 = scale * q2;
        g3 = scale * q3;
        g4 = scale * q4;
        return true;
    }
}
