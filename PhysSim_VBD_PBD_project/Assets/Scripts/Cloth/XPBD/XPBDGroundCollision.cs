using UnityEngine;

// Models the ground as a half-plane at y = ground.position.y with normal (0,1,0).
[RequireComponent(typeof(XPBDCloth))]
public class XPBDGroundCollision : MonoBehaviour
{
    public Transform ground;
    [Range(0f, 1f)] public float friction = 0.3f;

    private XPBDSolver solver;

    void Start()
    {
        var clothSim = GetComponent<XPBDCloth>();
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
        if (ground == null) return;

        float groundY = ground.position.y + 0.1f;

        int numVerts = solver.numVerts;
        Vector3[] positions = solver.positions;
        Vector3[] prevPositions = solver.previousPositions;
        float[] invMasses = solver.invMasses;

        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] == 0f) continue;

            Vector3 p = positions[i];
            if (p.y >= groundY) continue;

            p.y = groundY;

            // Friction logic: damp tangential (x,z) displacement.
            Vector3 disp = p - prevPositions[i];
            p.x -= disp.x * friction;
            p.z -= disp.z * friction;

            positions[i] = p;
        }
    }
}
