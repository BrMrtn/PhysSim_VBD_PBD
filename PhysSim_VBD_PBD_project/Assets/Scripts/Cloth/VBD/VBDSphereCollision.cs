using UnityEngine;
using static UnityEditor.PlayerSettings;

// Mirror of XPBDSphereCollision so the same scene setup works with VBD.
// Hooks the solver's OnSubstep event (fired once per substep, after the VBD
// iterations) and projects vertices that are inside the sphere back to the
// surface, with optional friction.
[RequireComponent(typeof(VBDCloth))]
public class VBDSphereCollision : MonoBehaviour
{
    public SphereCollider sphereCollider;
    public float stiffness = 1e6f;
    public float friction = 0.3f;

    private VBDSolver solver;

    void Start()
    {
        var clothSim = GetComponent<VBDCloth>();
        if (clothSim != null)
        {
            solver = clothSim.Solver;
            solver.OnVertexSolve += HandleVertexSolve;
        }
    }

    void OnDestroy()
    {
        if (solver != null)
            solver.OnVertexSolve -= HandleVertexSolve;
    }

    void HandleVertexSolve(int i, Vector3 pos, ref Vector3 f, ref float h00, ref float h11, ref float h22, ref float h01, ref float h02, ref float h12)
    {
        if (sphereCollider == null) return;

        Vector3 center = sphereCollider.transform.TransformPoint(sphereCollider.center);
        float radius = sphereCollider.radius * sphereCollider.transform.lossyScale.x * 1.1f;

        Vector3 delta = pos - center;
        float dist = delta.magnitude;

        if (dist < radius && dist > 1e-6f)
        {
            float penetration = radius - dist;
            Vector3 normal = delta / dist;

            // 1. Force: f_collision = k * penetration * normal
            Vector3 collisionForce = stiffness * penetration * normal;
            f += collisionForce;

            // 2. Hessian: H_collision = k * normal * normal^T (Constant Normal Approximation)
            // This is the outer product of the normal with itself
            h00 += stiffness * normal.x * normal.x;
            h11 += stiffness * normal.y * normal.y;
            h22 += stiffness * normal.z * normal.z;
            h01 += stiffness * normal.x * normal.y;
            h02 += stiffness * normal.x * normal.z;
            h12 += stiffness * normal.y * normal.z;

            // 3. Mark for acceleration skipping
            solver.isColliding[i] = true;

            // 4. Simple Friction Logic (can also be handled as a dissipative potential)
            if (friction > 0)
            {
                Vector3 relativeVel = (pos - solver.previousPosition[i]);
                Vector3 tangentVel = relativeVel - Vector3.Dot(relativeVel, normal) * normal;
                f -= tangentVel * friction * stiffness; // Damping-like friction
            }
        }
    }
}
