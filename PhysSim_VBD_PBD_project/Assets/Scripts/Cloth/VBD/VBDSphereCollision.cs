using UnityEngine;
using static UnityEditor.PlayerSettings;

// Mirror of XPBDSphereCollision so the same scene setup works with VBD.
// Hooks the solver's OnSubstep event (fired once per substep, after the VBD
// iterations) and projects vertices that are inside the sphere back to the
// surface, with optional friction.
[RequireComponent(typeof(VBDCloth))]
public class VBDSphereCollision : MonoBehaviour
{
    public enum CollisionMode
    {
        EnergyBased,
        PostProjection
    }

    public CollisionMode mode = CollisionMode.EnergyBased;
    public SphereCollider sphereCollider;
    public float stiffness = 1e6f;
    [Range(0f, 1f)] public float friction = 0.3f;

    public float frictionEpsDisp = 1e-4f;

    private VBDSolver solver;

    private Vector3 cachedCenter;
    private float cachedRadius;

    void Start()
    {
        var clothSim = GetComponent<VBDCloth>();
        if (clothSim != null)
        {
            solver = clothSim.Solver;
            solver.OnVertexSolve += HandleVertexSolve;
            solver.OnSubstep += HandleSubstep;
            solver.OnPreSubstep += CacheCphere;
        }
    }

    void OnDestroy()
    {
        if (solver != null)
        {
            solver.OnVertexSolve -= HandleVertexSolve;
            solver.OnSubstep -= HandleSubstep;
            solver.OnPreSubstep -= CacheCphere;
        }
    }

    private void CacheCphere()
    {
        if (sphereCollider == null) return;
        cachedCenter = sphereCollider.transform.TransformPoint(sphereCollider.center);
        cachedRadius = sphereCollider.radius * sphereCollider.transform.lossyScale.x * 1.1f;
    }

    void HandleVertexSolve(int i, Vector3 pos, ref Vector3 f,
                            ref float h00, ref float h11, ref float h22,
                            ref float h01, ref float h02, ref float h12)
    {
        if (mode != CollisionMode.EnergyBased) return;
        if (sphereCollider == null) return;

        Vector3 delta = pos - cachedCenter;
        float dist = delta.magnitude;
        if (dist > cachedRadius || dist < 1e-6f) return;

        float penetration = cachedRadius - dist;
        Vector3 normal = delta / dist;

        float lambda = stiffness * penetration;
        f += lambda * normal;

        h00 += stiffness * normal.x * normal.x;
        h11 += stiffness * normal.y * normal.y;
        h22 += stiffness * normal.z * normal.z;
        h01 += stiffness * normal.x * normal.y;
        h02 += stiffness * normal.x * normal.z;
        h12 += stiffness * normal.y * normal.z;

        // Fliping flag for skipping Chebyshev acceleration
        solver.isColliding[i] = true;

        // Friction (paper Eq. 14, 15, 16)
        if (friction > 0f && lambda > 0f)
        {
            Vector3 dx = pos - solver.previousPosition[i];
            Vector3 uT = dx - Vector3.Dot(dx, normal) * normal;
            float uMag = uT.magnitude;

            float f1OverU;
            if (uMag < frictionEpsDisp)
            {
                float r = uMag / frictionEpsDisp;
                f1OverU = (2f - r) / frictionEpsDisp;
            }
            else
                f1OverU = 1f / uMag;

            float c = friction * lambda * f1OverU;
            f -= c * uT;

            h00 += c * (1f - normal.x * normal.x);
            h11 += c * (1f - normal.y * normal.y);
            h22 += c * (1f - normal.z * normal.z);
            h01 -= c * normal.x * normal.y;
            h02 -= c * normal.x * normal.z;
            h12 -= c * normal.y * normal.z;
        }
    }

    void HandleSubstep()
    {
        if (mode != CollisionMode.PostProjection) return;
        if (sphereCollider == null) return;

        Vector3 center = sphereCollider.transform.TransformPoint(sphereCollider.center);
        float radius = sphereCollider.radius * sphereCollider.transform.lossyScale.x * 1.1f;

        int numVerts = solver.numVerts;
        Vector3[] positions = solver.positions;
        Vector3[] prevPositions = solver.previousPosition;
        float[] invMasses = solver.invMasses;

        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] == 0f) continue;

            Vector3 delta = positions[i] - center;
            float dist = delta.magnitude;
            if (dist >= radius || dist < 1e-6f) continue;

            Vector3 normal = delta / dist;
            positions[i] = center + normal * radius;

            // Friction logic
            Vector3 disp = positions[i] - prevPositions[i];
            Vector3 dispNormal = Vector3.Dot(disp, normal) * normal;
            Vector3 dispTangent = disp - dispNormal;
            positions[i] -= dispTangent * friction;
        }
    }
}
