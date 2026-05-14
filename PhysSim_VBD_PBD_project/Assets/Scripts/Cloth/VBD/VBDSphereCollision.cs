using UnityEngine;

// Mirror of XPBDSphereCollision so the same scene setup works with VBD.
// Hooks the solver's OnSubstep event (fired once per substep, after the VBD
// iterations) and projects vertices that are inside the sphere back to the
// surface, with optional friction.
[RequireComponent(typeof(VBDCloth))]
public class VBDSphereCollision : MonoBehaviour
{
    public SphereCollider sphereCollider;
    public float friction = 0.0f;

    private VBDSolver solver;

    void Start()
    {
        var clothSim = GetComponent<VBDCloth>();
        if (clothSim != null)
        {
            solver = clothSim.Solver;
            solver.OnSubstep += HandleCollision;
        }
    }

    void OnDestroy()
    {
        if (solver != null)
            solver.OnSubstep -= HandleCollision;
    }

    void HandleCollision()
    {
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
            if (dist >= radius || dist == 0f) continue;

            Vector3 normal = delta / dist;
            positions[i] = center + normal * radius;

            // Friction: damp the tangential component of the displacement
            // accumulated over this substep.
            Vector3 displacement = positions[i] - prevPositions[i];
            Vector3 dispNormal = Vector3.Dot(displacement, normal) * normal;
            Vector3 dispTangent = displacement - dispNormal;
            positions[i] -= dispTangent * friction;
        }
    }
}
