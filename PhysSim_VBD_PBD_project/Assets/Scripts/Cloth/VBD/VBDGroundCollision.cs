using UnityEngine;

// Mirror of XPBDGroundCollision so the same scene setup works with VBD.
// Models the ground as a half-plane at y = ground.position.y with normal (0,1,0).
[RequireComponent(typeof(VBDCloth))]
public class VBDGroundCollision : MonoBehaviour
{
    public enum CollisionMode
    {
        EnergyBased,
        PostProjection
    }

    public CollisionMode mode = CollisionMode.EnergyBased;
    public Transform ground;
    public float stiffness = 1e6f;
    [Range(0f, 1f)] public float friction = 0.3f;

    public float frictionEpsDisp = 1e-4f;

    private VBDSolver solver;

    private float cachedGroundY;

    void Start()
    {
        var cloth = GetComponent<VBDCloth>();
        if (cloth != null)
        {
            solver = cloth.Solver;
            solver.OnVertexSolve += HandleVertexSolve;
            solver.OnSubstep += HandleSubstep;
            solver.OnPreSubstep += CacheGround;
        }
    }

    void OnDestroy()
    {
        if (solver != null)
        {
            solver.OnVertexSolve -= HandleVertexSolve;
            solver.OnSubstep -= HandleSubstep;
            solver.OnPreSubstep -= CacheGround;
        }
    }

    private void CacheGround()
    {
        if (ground == null) return;
        cachedGroundY = ground.position.y + 0.2f;
    }

    void HandleVertexSolve(int i, Vector3 pos, ref Vector3 f,
                           ref float h00, ref float h11, ref float h22,
                           ref float h01, ref float h02, ref float h12)
    {
        if (mode != CollisionMode.EnergyBased) return;
        if (ground == null) return;

        float penetration = cachedGroundY - pos.y;
        if (penetration <= 0f) return;

        // Normal n = (0, 1, 0): only the y-component and h11 receive contributions.
        float lambda = stiffness * penetration;
        f.y += lambda;
        h11 += stiffness;

        // Fliping flag for skipping Chebyshev acceleration
        solver.isColliding[i] = true;

        // Friction (paper Eq. 14, 15, 16)
        if (friction > 0f && lambda > 0f)
        {
            Vector3 dx = pos - solver.previousPositions[i];
            // uT = dx - (dx . n) n  with n=(0,1,0)  ->  (dx.x, 0, dx.z)
            Vector3 uT = new Vector3(dx.x, 0f, dx.z);
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

            // I - n n^T = diag(1, 0, 1) for n=(0,1,0).
            h00 += c;
            h22 += c;
        }
    }

    void HandleSubstep()
    {
        if (mode != CollisionMode.PostProjection) return;
        if (ground == null) return;

        float groundY = ground.position.y;

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
