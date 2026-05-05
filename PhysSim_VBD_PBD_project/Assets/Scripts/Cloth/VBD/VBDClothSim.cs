using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class VBDClothSim : MonoBehaviour
{
    public int numSubsteps = 1;
    public int numIterations = 30;

    public float stretchingStiffness = 1e5f;
    public float shearStiffness = 1e4f;
    public float bendingStiffness = 1e3f;

    [Header("Acceleration (Section 4.6 of VBD paper)")]
    public bool useChebyshevAcceleration = true;
    [Range(0f, 1f)] public float chebyshevRho = 0.95f;

    [Header("Initial Guess (Section 4.5 of VBD paper)")]
    [Tooltip("Adaptive initial guess that blends in the previous-frame acceleration. Helps when the cloth is in static equilibrium under gravity (avoids penetrating its support before the first iteration).")]
    public bool adaptiveInitialGuess = true;
    [Tooltip("Cap (in m/s^2) on the gravity-direction component of the previous-frame acceleration that gets folded into the initial guess.")]
    public float adaptiveAccelerationCap = 10f;

    [Header("Numerics")]
    [Tooltip("Small value added to the diagonal of the local Hessian for numerical safety.")]
    public float hessianRegularization = 1e-8f;
    [Tooltip("Optional cap on a single per-vertex Newton step (m). 0 disables the cap.")]
    public float maxLocalStep = 0f;
    public bool addInitNoise = false;

    [Header("Self Collision")]
    public bool handleSelfCollisions = true;
    [Range(0f, 1f)] public float selfCollisionFriction = 0.0f;

    [Header("Logging")]
    public bool logMsPerFrame = true;

    public event Action OnUpdate;

    [HideInInspector] public int numVerts;
    [HideInInspector] public int numX;
    [HideInInspector] public int numY;
    [HideInInspector] public Vector3[] positions;
    [HideInInspector] public Vector3[] velocities;
    [HideInInspector] public Vector3[] previousPosition;
    [HideInInspector] public Vector3[] restPositions;
    [HideInInspector] public float[] invMasses;
    [HideInInspector] public Spring[] springs;

    public struct Spring
    {
        public int i, j;
        public float restLength;
        public float stiffness;
    }

    private const int LogEveryNFrames = 10;

    private Vector3 gravity = new Vector3(0f, -9.81f, 0f);
    private Vector3 gravityDir;
    private float gravityNorm;

    private MeshFilter meshFilter;
    private Mesh mesh;
    private Transform tr;

    private float spacing;
    private float thickness;

    private Vector3[] inertia;                // y_i = x_i + h v_i (fixed during a substep's iterations)
    private Vector3[] previousVelocities;     // v_{n-1} for adaptive initial guess
    private Vector3[] prevIterPositions;      // x at start of current iteration (snapshot for Chebyshev)
    private Vector3[] prevPrevIterPositions;  // x at start of previous iteration (Chebyshev)
    private bool hasPrevVelocities;

    private int[] meshToGrid;
    private Vector3[] renderVertices;
    private SpatialHash spatialHash;

    private List<int>[] vertexAdjacency;      // for each vertex: indices into springs[]
    private int[][] vertexColors;             // colour groups; vertices in same group have no shared spring

    private string performanceText = "VBD Simulation: -- ms/frame";
    private GUIStyle performanceStyle;

    void Awake()
    {
        meshFilter = gameObject.GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;
        tr = transform;
        mesh.MarkDynamic();

        Destroy(gameObject.GetComponent<MeshCollider>());

        Vector3[] localVerts = mesh.vertices;
        numVerts = localVerts.Length;
        renderVertices = new Vector3[numVerts];

        float eps = 0.001f;
        List<float> xCoords = GetDistinctSortedCoordinates(localVerts, v => v.x, eps);
        List<float> yCoords = GetDistinctSortedCoordinates(localVerts, v => v.y, eps);
        numX = xCoords.Count;
        numY = yCoords.Count;
        if (numX > 1) spacing = xCoords[1] - xCoords[0];
        thickness = spacing;

        positions = new Vector3[numVerts];
        velocities = new Vector3[numVerts];
        previousVelocities = new Vector3[numVerts];
        previousPosition = new Vector3[numVerts];
        restPositions = new Vector3[numVerts];
        inertia = new Vector3[numVerts];
        prevIterPositions = new Vector3[numVerts];
        prevPrevIterPositions = new Vector3[numVerts];
        invMasses = new float[numVerts];
        Array.Fill(invMasses, 1f);

        BuildSimulationGrid(xCoords, yCoords);
        Array.Copy(positions, restPositions, numVerts);

        BuildMeshToGrid(localVerts, xCoords, yCoords, eps);
        InitSprings();
        BuildVertexAdjacency();
        BuildVertexColoring();

        if (addInitNoise)
            for (int i = 0; i < numVerts; i++)
                if (invMasses[i] > 0f)
                    positions[i] += UnityEngine.Random.insideUnitSphere * 0.001f;

        spatialHash = new SpatialHash(spacing, numVerts);

        gravityNorm = gravity.magnitude;
        gravityDir = gravityNorm > 0f ? gravity / gravityNorm : Vector3.zero;

        hasPrevVelocities = false;
    }

    void Update()
    {
        bool shouldLogPerformance = logMsPerFrame && Time.frameCount % LogEveryNFrames == 0;
        double simStartTime = shouldLogPerformance ? Time.realtimeSinceStartupAsDouble : 0;

        float dt = 1f / 24f;
        float sdt = dt / Mathf.Max(1, numSubsteps);
        float invSdt2 = 1f / (sdt * sdt);
        float maxVelocity = 0.2f * thickness / sdt;

        if (handleSelfCollisions)
        {
            spatialHash.Create(positions);
            float maxTravelDistance = maxVelocity * dt;
            spatialHash.QueryAll(positions, maxTravelDistance);
        }

        for (int sub = 0; sub < numSubsteps; sub++)
        {
            // 1. Variational implicit Euler initial guess
            //    y = x + h v + h^2 a_ext  (a_ext folded in via the explicit gravity update on v)
            //    Optionally use the previous-frame acceleration to skew the guess
            //    away from "free fall", which gives much better starting points
            //    for cloth that's already supported (see VBD paper Section 4.5).
            for (int i = 0; i < numVerts; i++)
            {
                if (invMasses[i] == 0f)
                {
                    previousPosition[i] = positions[i];
                    inertia[i] = positions[i];
                    continue;
                }

                velocities[i] += gravity * sdt;

                float v = velocities[i].magnitude;
                if (v > maxVelocity) velocities[i] *= maxVelocity / v;

                previousPosition[i] = positions[i];
                inertia[i] = positions[i] + velocities[i] * sdt;

                if (adaptiveInitialGuess && hasPrevVelocities)
                {
                    // a~  = clamp( (v - v_prev)/h . g_hat, 0, |g| ) * g_hat
                    Vector3 aApprox = (velocities[i] - previousVelocities[i]) / sdt;
                    float aAlongG = Vector3.Dot(aApprox, gravityDir);
                    float cap = Mathf.Min(adaptiveAccelerationCap, gravityNorm);
                    aAlongG = Mathf.Clamp(aAlongG, 0f, cap);

                    positions[i] = previousPosition[i]
                                 + sdt * previousVelocities[i]
                                 + sdt * sdt * aAlongG * gravityDir;
                }
                else
                {
                    positions[i] = inertia[i];
                }
            }

            // 2. Newton iterations on the per-vertex blocks (Algorithm 1 of the paper).
            float omega = 1f;
            float rho2 = chebyshevRho * chebyshevRho;

            for (int iter = 0; iter < numIterations; iter++)
            {
                // Snapshot for Chebyshev acceleration before the Gauss-Seidel sweep.
                if (useChebyshevAcceleration)
                    Array.Copy(positions, prevIterPositions, numVerts);

                // Gauss-Seidel sweep: vertices within the same colour share no spring,
                // so their local solves are mutually independent.
                for (int c = 0; c < vertexColors.Length; c++)
                {
                    int[] group = vertexColors[c];
                    for (int g = 0; g < group.Length; g++)
                    {
                        int v = group[g];
                        if (invMasses[v] == 0f) continue;
                        SolveVertex(v, invSdt2);
                    }
                }

                if (useChebyshevAcceleration)
                {
                    if (iter == 0) omega = 1f;
                    else if (iter == 1) omega = 2f / (2f - rho2);
                    else omega = 4f / (4f - rho2 * omega);

                    // x_new = omega * (x - x_{k-2}) + x_{k-2}
                    // (At iter == 1 the "k-2" snapshot is the initial guess, which
                    // is what TinyVBD does too.)
                    if (omega > 1f && iter >= 1)
                    {
                        for (int i = 0; i < numVerts; i++)
                        {
                            if (invMasses[i] == 0f) continue;
                            positions[i] = omega * (positions[i] - prevPrevIterPositions[i])
                                         + prevPrevIterPositions[i];
                        }
                    }

                    // shift snapshots: prevPrev <- prev
                    Array.Copy(prevIterPositions, prevPrevIterPositions, numVerts);
                }
            }

            if (handleSelfCollisions) SolveSelfCollisions();
            OnUpdate?.Invoke();

            // 3. Velocity update + cache the previous v for next step's adaptive guess.
            for (int i = 0; i < numVerts; i++)
            {
                if (invMasses[i] == 0f)
                {
                    previousVelocities[i] = Vector3.zero;
                    velocities[i] = Vector3.zero;
                    continue;
                }
                previousVelocities[i] = velocities[i];
                velocities[i] = (positions[i] - previousPosition[i]) / sdt;
            }
            hasPrevVelocities = true;
        }

        // Push to render mesh.
        Matrix4x4 worldToLocal = tr.worldToLocalMatrix;
        for (int i = 0; i < numVerts; i++)
            renderVertices[i] = worldToLocal.MultiplyPoint3x4(positions[meshToGrid[i]]);

        mesh.SetVertices(renderVertices, 0, numVerts, UnityEngine.Rendering.MeshUpdateFlags.DontRecalculateBounds);
        mesh.RecalculateNormals();

        if (shouldLogPerformance)
        {
            double simEndTime = Time.realtimeSinceStartupAsDouble;
            double msPerFrame = (simEndTime - simStartTime) * 1000.0;
            performanceText = $"VBD Simulation: {msPerFrame:F2} ms/frame";
        }
    }

    // ---------------------------------------------------------------------
    // Per-vertex local Newton step
    //
    //   E_v(x_v) = 0.5 * (m_v / h^2) || x_v - y_v ||^2  + sum_e Psi_e(x_v)
    //
    //   g_v = (m_v / h^2) (x_v - y_v) + sum_e dPsi_e/dx_v
    //   H_v = (m_v / h^2) I            + sum_e d2Psi_e/dx_v dx_v
    //
    // For a Hookean spring between v and j with rest length L0 and stiffness k:
    //   d   = x_v - x_j,   L = ||d||,   n = d / L
    //   dPsi/dx_v   = ± k (L - L0) n         (sign + when we are vertex i of the spring)
    //   d2Psi/dx_v2 = k * I - k (L0/L) (I - n n^T)
    // (TinyVBD / Gaia mass-spring use exactly this form.)
    // ---------------------------------------------------------------------
    private void SolveVertex(int vId, float invSdt2)
    {
        float mass = 1f / invMasses[vId];
        float mh2 = mass * invSdt2;

        // f := -g (the "force"). H := the local Hessian.
        Vector3 f = mh2 * (inertia[vId] - positions[vId]);
        Mat3 H = Mat3.Diagonal(mh2);

        var inc = vertexAdjacency[vId];
        int incCount = inc.Count;
        for (int k = 0; k < incCount; k++)
        {
            Spring s = springs[inc[k]];
            int other = (s.i == vId) ? s.j : s.i;
            float sign = (s.i == vId) ? +1f : -1f;

            Vector3 d = positions[vId] - positions[other];
            float L = d.magnitude;
            if (L < 1e-8f) continue;

            Vector3 n = d / L;
            float L0 = s.restLength;
            float ks = s.stiffness;

            // -gradient contribution
            f -= sign * (ks * (L - L0)) * n;

            // Hessian contribution: ks * I - (ks * L0 / L) * (I - n n^T)
            float kL0overL = ks * (L0 / L);
            // diag part:  (ks - kL0overL)  on diagonal
            float diagPart = ks - kL0overL;
            // outer part: + kL0overL * (n n^T)
            // Combined directly:
            H.m00 += diagPart + kL0overL * (n.x * n.x);
            H.m11 += diagPart + kL0overL * (n.y * n.y);
            H.m22 += diagPart + kL0overL * (n.z * n.z);
            float nxy = kL0overL * n.x * n.y;
            float nxz = kL0overL * n.x * n.z;
            float nyz = kL0overL * n.y * n.z;
            H.m01 += nxy; H.m10 += nxy;
            H.m02 += nxz; H.m20 += nxz;
            H.m12 += nyz; H.m21 += nyz;
        }

        // Tikhonov regularizer on the diagonal for the rare singular cases.
        H.m00 += hessianRegularization;
        H.m11 += hessianRegularization;
        H.m22 += hessianRegularization;

        if (!H.TrySolve(f, out Vector3 dx)) return;

        if (maxLocalStep > 0f)
        {
            float dxLen2 = dx.sqrMagnitude;
            float maxStep2 = maxLocalStep * maxLocalStep;
            if (dxLen2 > maxStep2)
                dx *= maxLocalStep / Mathf.Sqrt(dxLen2);
        }

        positions[vId] += dx;
    }

    // ---------------------------------------------------------------------
    // Topology / spring construction. Mirrors XPBDClothSim so the two solvers
    // operate on the same elastic model.
    // ---------------------------------------------------------------------
    private void InitSprings()
    {
        var list = new List<Spring>();

        AddSpring(list, 0, 0, 1, 0, stretchingStiffness); // stretch horizontal
        AddSpring(list, 0, 0, 0, 1, stretchingStiffness); // stretch vertical
        AddSpring(list, 0, 0, 1, 1, shearStiffness);      // shear /
        AddSpring(list, 1, 0, 0, 1, shearStiffness);      // shear \
        AddSpring(list, 0, 0, 2, 0, bendingStiffness);    // bend horizontal
        AddSpring(list, 0, 0, 0, 2, bendingStiffness);    // bend vertical

        springs = list.ToArray();
    }

    private void AddSpring(List<Spring> list, int oi0, int oj0, int oi1, int oj1, float stiffness)
    {
        for (int iy = 0; iy < numY; iy++)
            for (int ix = 0; ix < numX; ix++)
            {
                int i0 = ix + oi0, j0 = iy + oj0;
                int i1 = ix + oi1, j1 = iy + oj1;
                if (i0 < numX && j0 < numY && i1 < numX && j1 < numY)
                {
                    int p1 = j0 * numX + i0;
                    int p2 = j1 * numX + i1;
                    list.Add(new Spring
                    {
                        i = p1,
                        j = p2,
                        restLength = Vector3.Distance(positions[p1], positions[p2]),
                        stiffness = stiffness
                    });
                }
            }
    }

    private void BuildVertexAdjacency()
    {
        vertexAdjacency = new List<int>[numVerts];
        for (int i = 0; i < numVerts; i++) vertexAdjacency[i] = new List<int>(12);
        for (int s = 0; s < springs.Length; s++)
        {
            vertexAdjacency[springs[s].i].Add(s);
            vertexAdjacency[springs[s].j].Add(s);
        }
    }

    // Greedy graph coloring on the spring graph. Two vertices that share a
    // spring must be in different colors so the per-color sweep can run as a
    // proper Gauss-Seidel pass (no race on the right-hand side).
    private void BuildVertexColoring()
    {
        var neighbors = new List<int>[numVerts];
        for (int i = 0; i < numVerts; i++) neighbors[i] = new List<int>(12);
        for (int s = 0; s < springs.Length; s++)
        {
            int a = springs[s].i, b = springs[s].j;
            neighbors[a].Add(b);
            neighbors[b].Add(a);
        }

        int[] colors = new int[numVerts];
        Array.Fill(colors, -1);
        int maxColor = -1;

        // small reusable scratch
        var used = new List<bool>();

        for (int v = 0; v < numVerts; v++)
        {
            used.Clear();
            for (int t = 0; t <= maxColor + 1; t++) used.Add(false);

            var nb = neighbors[v];
            for (int k = 0; k < nb.Count; k++)
            {
                int c = colors[nb[k]];
                if (c >= 0)
                {
                    while (used.Count <= c) used.Add(false);
                    used[c] = true;
                }
            }

            int chosen = 0;
            while (chosen < used.Count && used[chosen]) chosen++;
            colors[v] = chosen;
            if (chosen > maxColor) maxColor = chosen;
        }

        var groups = new List<int>[maxColor + 1];
        for (int c = 0; c <= maxColor; c++) groups[c] = new List<int>();
        for (int v = 0; v < numVerts; v++) groups[colors[v]].Add(v);

        vertexColors = new int[groups.Length][];
        for (int c = 0; c < groups.Length; c++) vertexColors[c] = groups[c].ToArray();
    }

    // ---------------------------------------------------------------------
    // Self-collision: same projection-based scheme XPBDClothSim uses, run
    // once at the end of each substep. VBD can in principle absorb contact
    // into the local Hessian, but a positional projection keeps the two
    // solvers' contact handling identical, which is what we want for the
    // thesis comparison.
    // ---------------------------------------------------------------------
    private void SolveSelfCollisions()
    {
        float thickness2 = thickness * thickness;

        for (int id0 = 0; id0 < numVerts; id0++)
        {
            if (invMasses[id0] == 0f) continue;
            int first = spatialHash.firstAdjId[id0];
            int last = spatialHash.firstAdjId[id0 + 1];

            for (int j = first; j < last; j++)
            {
                int id1 = spatialHash.adjIds[j];
                if (invMasses[id1] == 0f) continue;

                Vector3 delta = positions[id0] - positions[id1];
                float dist2 = delta.sqrMagnitude;
                if (dist2 == 0f || dist2 > thickness2) continue;

                float restDist2 = (restPositions[id0] - restPositions[id1]).sqrMagnitude;
                if (dist2 > restDist2) continue;

                float minDist = thickness;
                if (restDist2 < thickness2) minDist = Mathf.Sqrt(restDist2);

                float dist = Mathf.Sqrt(dist2);
                Vector3 correction = delta * ((minDist - dist) / dist);
                positions[id0] += 0.5f * correction;
                positions[id1] -= 0.5f * correction;

                if (selfCollisionFriction > 0f)
                {
                    Vector3 v0 = positions[id0] - previousPosition[id0];
                    Vector3 v1 = positions[id1] - previousPosition[id1];
                    Vector3 vAvg = 0.5f * (v0 + v1);
                    positions[id0] += (vAvg - v0) * selfCollisionFriction;
                    positions[id1] += (vAvg - v1) * selfCollisionFriction;
                }
            }
        }
    }

    // ---------------------------------------------------------------------
    // Grid construction (mirrors XPBDClothSim)
    // ---------------------------------------------------------------------
    private void BuildSimulationGrid(List<float> xCoords, List<float> yCoords)
    {
        for (int iy = 0; iy < numY; iy++)
            for (int ix = 0; ix < numX; ix++)
            {
                int idx = iy * numX + ix;
                Vector3 localPos = new Vector3(xCoords[ix], yCoords[iy], 0f);
                positions[idx] = tr.TransformPoint(localPos);
            }
    }

    private void BuildMeshToGrid(Vector3[] localVerts, List<float> xCoords, List<float> yCoords, float eps)
    {
        meshToGrid = new int[numVerts];
        for (int i = 0; i < numVerts; i++)
        {
            int ix = xCoords.FindIndex(x => Mathf.Abs(x - localVerts[i].x) < eps);
            int iy = yCoords.FindIndex(y => Mathf.Abs(y - localVerts[i].y) < eps);
            meshToGrid[i] = iy * numX + ix;
        }
    }

    private static List<float> GetDistinctSortedCoordinates(Vector3[] vertices, Func<Vector3, float> selector, float epsilon)
    {
        var coords = new List<float>();
        foreach (Vector3 v in vertices)
        {
            float value = selector(v);
            if (!coords.Any(c => Mathf.Abs(c - value) < epsilon))
                coords.Add(value);
        }
        coords.Sort();
        return coords;
    }

    void OnGUI()
    {
        if (!logMsPerFrame) return;
        if (performanceStyle == null)
        {
            performanceStyle = new GUIStyle(GUI.skin.label);
            performanceStyle.fontSize = 14;
            performanceStyle.normal.textColor = Color.white;
        }
        GUI.Label(new Rect(10, 10, 420, 30), performanceText, performanceStyle);
    }

    // ---------------------------------------------------------------------
    // Inline 3x3 matrix struct.
    //
    // The local Hessian is symmetric but can become indefinite under heavy
    // compression (a spring with L < L0 contributes a non-PSD term in the
    // directions perpendicular to n). The cofactor inversion below works on
    // any non-singular 3x3, and the inertia term m/h^2 * I usually keeps the
    // sum well-conditioned.
    // ---------------------------------------------------------------------
    private struct Mat3
    {
        public float m00, m01, m02;
        public float m10, m11, m12;
        public float m20, m21, m22;

        public static Mat3 Diagonal(float d)
        {
            return new Mat3 { m00 = d, m11 = d, m22 = d };
        }

        public bool TrySolve(Vector3 b, out Vector3 x)
        {
            float c00 = m11 * m22 - m12 * m21;
            float c01 = m02 * m21 - m01 * m22;
            float c02 = m01 * m12 - m02 * m11;

            float det = m00 * c00 + m10 * c01 + m20 * c02;

            if (Mathf.Abs(det) < 1e-20f || float.IsNaN(det) || float.IsInfinity(det))
            {
                x = Vector3.zero;
                return false;
            }

            float c10 = m12 * m20 - m10 * m22;
            float c11 = m00 * m22 - m02 * m20;
            float c12 = m02 * m10 - m00 * m12;
            float c20 = m10 * m21 - m11 * m20;
            float c21 = m01 * m20 - m00 * m21;
            float c22 = m00 * m11 - m01 * m10;

            float invDet = 1f / det;
            x = new Vector3(
                (c00 * b.x + c01 * b.y + c02 * b.z) * invDet,
                (c10 * b.x + c11 * b.y + c12 * b.z) * invDet,
                (c20 * b.x + c21 * b.y + c22 * b.z) * invDet
            );
            return true;
        }
    }
}
