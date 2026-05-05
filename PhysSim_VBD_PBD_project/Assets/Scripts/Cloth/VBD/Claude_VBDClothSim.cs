using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Claude_VBDClothSim : MonoBehaviour
{
    public int numSubsteps = 1;
    public int numIterations = 15;

    public float stretchingStiffness = 1e6f;
    public float shearStiffness = 1e4f;
    public float bendingStiffness = 1e3f;

    // --- Chebyshev semi-iterative acceleration (from VBD paper / strand code) ---
    public bool useAcceleration = false;
    [Range(0f, 1f)] public float accelerationRho = 0.5f;

    // --- Misc ---
    public bool logMsPerFrame = true;
    public float selfCollisionFriction = 0.0f;
    public bool addInitNoise = false;
    public event Action OnUpdate;

    private const int logEveryNFrames = 10;
    private float spacing;
    private float thickness;

    private string performanceText = "VBD Simulation: -- ms/frame";
    private GUIStyle performanceStyle;

    private Vector3 gravity = new Vector3(0, -9.81f, 0);

    private MeshFilter meshFilter;
    private Mesh mesh;
    private Transform tr;

    [HideInInspector] public int numVerts;
    [HideInInspector] public int numX;
    [HideInInspector] public int numY;

    // Position state (analogues of mVertPos / mVertPrevPos / mVelocity / inertia / prevprevPos in C++ VBD)
    [HideInInspector] public Vector3[] positions;          // mVertPos
    [HideInInspector] public Vector3[] previousPosition;   // mVertPrevPos (start of substep)
    [HideInInspector] public Vector3[] velocities;         // mVelocity
    [HideInInspector] public Vector3[] previousVelocities; // mVelocitiesPrev (used by initial guess)
    [HideInInspector] public Vector3[] inertia;            // x_prev + dt * v
    [HideInInspector] public Vector3[] prevprevPos;        // position before last solve iteration (Chebyshev)
    [HideInInspector] public Vector3[] prevIterPos;        // scratch for iteration bookkeeping
    [HideInInspector] public Vector3[] restPositions;
    [HideInInspector] public float[] masses;
    [HideInInspector] public float[] invMasses;

    /// <summary>Single mass-spring constraint between two cloth vertices.</summary>
    public struct Spring
    {
        public int v1;
        public int v2;
        public float restLength;
        public float stiffness;
    }

    [HideInInspector] public Spring[] springs;
    [HideInInspector] public List<int>[] vertAdjacentSprings;

    private bool hasVelocitiesPrev = false;
    private bool hasApproxAcceleration = false;

    private int[] meshToGrid;
    private Vector3[] renderVertices;

    // -------------------------------------------------------------------------
    // Setup
    // -------------------------------------------------------------------------
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
        previousPosition = new Vector3[numVerts];
        velocities = new Vector3[numVerts];
        previousVelocities = new Vector3[numVerts];
        inertia = new Vector3[numVerts];
        prevprevPos = new Vector3[numVerts];
        prevIterPos = new Vector3[numVerts];
        restPositions = new Vector3[numVerts];
        masses = new float[numVerts];
        invMasses = new float[numVerts];
        Array.Fill(masses, 1f);
        Array.Fill(invMasses, 1f);

        int topLeftIdx = (numY - 1) * numX;
        int topRightIdx = (numY - 1) * numX + (numX - 1);

        invMasses[topLeftIdx] = 0f;
        invMasses[topRightIdx] = 0f;


        BuildSimulationGrid(xCoords, yCoords);
        Array.Copy(positions, restPositions, numVerts);

        BuildMeshToGrid(localVerts, xCoords, yCoords, eps);
        InitSprings();

        if (addInitNoise)
            for (int i = 0; i < numVerts; i++)
                if (invMasses[i] > 0f)
                    positions[i] += UnityEngine.Random.insideUnitSphere * 0.001f;
    }

    // -------------------------------------------------------------------------
    // Per-frame update
    // -------------------------------------------------------------------------
    void Update()
    {
        bool shouldLogPerformance = logMsPerFrame && Time.frameCount % logEveryNFrames == 0;
        double simStartTime = shouldLogPerformance ? Time.realtimeSinceStartupAsDouble : 0;

        float dt = 1f / 24f;
        float sdt = dt / numSubsteps;

        for (int step = 0; step < numSubsteps; step++)
        {
            ForwardStep(sdt);

            float omega = 1f;
            for (int iter = 0; iter < numIterations; iter++)
            {
                // Save iteration-start positions (used by Chebyshev acceleration).
                Array.Copy(positions, prevIterPos, numVerts);

                Solve(sdt);

                if (useAcceleration)
                {
                    omega = GetAcceleratorOmega(iter + 1, accelerationRho, omega);
                    ApplyAccelerator(omega);
                    // prevprevPos = the iter-start position from this iteration,
                    // which becomes "prev-prev" for the next iteration.
                    Array.Copy(prevIterPos, prevprevPos, numVerts);
                }
            }

            OnUpdate?.Invoke();

            UpdateVelocity(sdt);
        }

        // Push back to mesh.
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

    // -------------------------------------------------------------------------
    // VBD forward step (gravity + inertia + initial guess)
    // -------------------------------------------------------------------------
    void ForwardStep(float dt)
    {
        // 1) Apply gravity to velocities (free vertices only).
        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] == 0f) { velocities[i] = Vector3.zero; continue; }
            velocities[i] += gravity * dt;
        }

        // 2) Inertia (predicted position with current velocity).
        for (int i = 0; i < numVerts; i++)
            inertia[i] = positions[i] + velocities[i] * dt;

        // 3) Save start-of-substep position before any iterative update.
        Array.Copy(positions, previousPosition, numVerts);

        // 4) Initial guess.
        //    - First substep: use plain inertia.
        //    - Subsequent substeps: VBD's adaptive initial guess that projects
        //      the previous-frame approximate acceleration onto gravity dir
        //      and uses it to bias the prediction (matches strand C++ code).
        if (hasVelocitiesPrev)
        {
            Vector3 gravDir = new Vector3(0f, -1f, 0f);
            float gravNorm = gravity.magnitude;
            float invDt = 1f / dt;
            float dt2 = dt * dt;
            for (int i = 0; i < numVerts; i++)
            {
                if (invMasses[i] == 0f) { positions[i] = previousPosition[i]; continue; }

                Vector3 accApprox = (velocities[i] - previousVelocities[i]) * invDt;
                float accelComponent = Vector3.Dot(accApprox, gravDir);
                if (accelComponent > gravNorm) accelComponent = gravNorm;
                if (accelComponent <= 1e-5f) accelComponent = 0f;

                positions[i] = previousPosition[i]
                             + dt * previousVelocities[i]
                             + dt2 * gravDir * accelComponent;
            }
            hasApproxAcceleration = true;
        }
        else
        {
            Array.Copy(inertia, positions, numVerts);
        }
    }

    // -------------------------------------------------------------------------
    // VBD inner solve: one Gauss-Seidel block-descent pass over all vertices.
    // -------------------------------------------------------------------------
    void Solve(float dt)
    {
        float dtSqrReciprocal = 1f / (dt * dt);

        for (int iV = 0; iV < numVerts; iV++)
        {
            if (invMasses[iV] == 0f) continue;

            // ---- Inertia term ----
            // f = m/dt^2 * (inertia - x),  H = m/dt^2 * I
            float massDtSqr = masses[iV] * dtSqrReciprocal;

            float h00 = massDtSqr, h01 = 0f, h02 = 0f;
            float h11 = massDtSqr, h12 = 0f;
            float h22 = massDtSqr;

            Vector3 f = (inertia[iV] - positions[iV]) * massDtSqr;

            // ---- Spring contributions from all incident springs ----
            var adjSprings = vertAdjacentSprings[iV];
            for (int s = 0; s < adjSprings.Count; s++)
            {
                Spring spring = springs[adjSprings[s]];
                int v1 = spring.v1;
                int v2 = spring.v2;

                Vector3 diff = positions[v1] - positions[v2];
                float len = diff.magnitude;
                if (len < 1e-10f) continue;

                float l0 = spring.restLength;
                float k = spring.stiffness;

                float invL = 1f / len;
                Vector3 d = diff * invL;        // unit direction
                float ratio = l0 * invL;          // l0 / l

                // h_spring = k * ((1 - l0/l) I + (l0/l) d d^T)
                //          = coeff1 * I + coeff2 * d d^T
                float coeff1 = k * (1f - ratio);
                float coeff2 = k * ratio;

                h00 += coeff1 + coeff2 * d.x * d.x;
                h11 += coeff1 + coeff2 * d.y * d.y;
                h22 += coeff1 + coeff2 * d.z * d.z;
                h01 += coeff2 * d.x * d.y;
                h02 += coeff2 * d.x * d.z;
                h12 += coeff2 * d.y * d.z;

                // Force from this spring on iV.
                // F_v1 = k (l0 - l)/l * diff,  F_v2 = -F_v1
                float fCoeff = k * (l0 - len) * invL;
                if (v1 == iV) f += diff * fCoeff;
                else f -= diff * fCoeff;
            }

            // ---- Solve symmetric 3x3 system H dx = f ----
            Vector3 dx = SolveSymmetric3x3(h00, h11, h22, h01, h02, h12, f);

            positions[iV] += dx;
        }
    }

    /// <summary>
    /// Solves H * x = f for symmetric 3x3 H by direct cofactor inversion.
    /// Returns the zero vector if H is (near-)singular.
    /// </summary>
    Vector3 SolveSymmetric3x3(float h00, float h11, float h22,
                              float h01, float h02, float h12,
                              Vector3 f)
    {
        float det = h00 * (h11 * h22 - h12 * h12)
                  - h01 * (h01 * h22 - h12 * h02)
                  + h02 * (h01 * h12 - h11 * h02);

        if (Mathf.Abs(det) < 1e-20f) return Vector3.zero;
        float invDet = 1f / det;

        // Symmetric inverse (cofactor / det).
        float c00 = (h11 * h22 - h12 * h12) * invDet;
        float c11 = (h00 * h22 - h02 * h02) * invDet;
        float c22 = (h00 * h11 - h01 * h01) * invDet;
        float c01 = -(h01 * h22 - h12 * h02) * invDet;
        float c02 = (h01 * h12 - h11 * h02) * invDet;
        float c12 = -(h00 * h12 - h01 * h02) * invDet;

        return new Vector3(
            c00 * f.x + c01 * f.y + c02 * f.z,
            c01 * f.x + c11 * f.y + c12 * f.z,
            c02 * f.x + c12 * f.y + c22 * f.z
        );
    }

    // -------------------------------------------------------------------------
    // Velocity update (BDF1 / implicit Euler style)
    // -------------------------------------------------------------------------
    void UpdateVelocity(float dt)
    {
        Array.Copy(velocities, previousVelocities, numVerts);
        float invDt = 1f / dt;
        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] == 0f) { velocities[i] = Vector3.zero; continue; }
            velocities[i] = (positions[i] - previousPosition[i]) * invDt;
        }
        hasVelocitiesPrev = true;
    }

    // -------------------------------------------------------------------------
    // Chebyshev semi-iterative acceleration (VBD paper)
    // -------------------------------------------------------------------------
    float GetAcceleratorOmega(int order, float rho, float prevOmega)
    {
        if (order == 1) return 1f;
        if (order == 2) return 2f / (2f - rho * rho);
        return 4f / (4f - rho * rho * prevOmega);
    }

    void ApplyAccelerator(float omega)
    {
        if (omega <= 1f) return;
        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] == 0f) continue;
            // x_new = omega * (x_curr - x_prevprev) + x_prevprev
            positions[i] = omega * (positions[i] - prevprevPos[i]) + prevprevPos[i];
        }
    }

    // -------------------------------------------------------------------------
    // Diagnostics
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    // Cloth-specific setup (from XPBD reference - grid, mapping, springs)
    // -------------------------------------------------------------------------
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

    private void InitSprings()
    {
        var springsList = new List<Spring>();
        vertAdjacentSprings = new List<int>[numVerts];
        for (int i = 0; i < numVerts; i++) vertAdjacentSprings[i] = new List<int>();

        // Same connectivity pattern as the XPBD reference:
        AddSprings(springsList, 0, 0, 1, 0, stretchingStiffness); // stretch horizontal
        AddSprings(springsList, 0, 0, 0, 1, stretchingStiffness); // stretch vertical
        AddSprings(springsList, 0, 0, 1, 1, shearStiffness);      // shear  /
        AddSprings(springsList, 1, 0, 0, 1, shearStiffness);      // shear  \
        AddSprings(springsList, 0, 0, 2, 0, bendingStiffness);    // bend horizontal
        AddSprings(springsList, 0, 0, 0, 2, bendingStiffness);    // bend vertical

        springs = springsList.ToArray();
    }

    private void AddSprings(List<Spring> springsList,
                            int offset_i0, int offset_j0,
                            int offset_i1, int offset_j1,
                            float stiffness)
    {
        for (int iy = 0; iy < numY; iy++)
            for (int ix = 0; ix < numX; ix++)
            {
                int i0 = ix + offset_i0;
                int j0 = iy + offset_j0;
                int i1 = ix + offset_i1;
                int j1 = iy + offset_j1;

                if (i0 < numX && j0 < numY && i1 < numX && j1 < numY)
                {
                    int p1 = j0 * numX + i0;
                    int p2 = j1 * numX + i1;

                    int springId = springsList.Count;
                    springsList.Add(new Spring
                    {
                        v1 = p1,
                        v2 = p2,
                        restLength = Vector3.Distance(positions[p1], positions[p2]),
                        stiffness = stiffness
                    });
                    vertAdjacentSprings[p1].Add(springId);
                    vertAdjacentSprings[p2].Add(springId);
                }
            }
    }
}