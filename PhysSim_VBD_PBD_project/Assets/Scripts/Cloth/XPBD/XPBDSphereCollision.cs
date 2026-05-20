using UnityEngine;

[RequireComponent(typeof(XPBDCloth))]
public class XPBDSphereCollision : MonoBehaviour
{
    public SphereCollider sphereCollider;
    [Range(0f, 1f)] public float friction = 0.3f;

    private XPBDSolver solver;

    void Start()
    {
        var cloth = GetComponent<XPBDCloth>();
        if (cloth != null)
        {
            solver = cloth.Solver;
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
        Vector3[] prevPositions = solver.previousPositions;
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
