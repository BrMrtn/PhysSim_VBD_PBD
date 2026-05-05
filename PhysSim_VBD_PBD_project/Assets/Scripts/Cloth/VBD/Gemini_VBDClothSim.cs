using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Gemini_VBDClothSim : MonoBehaviour
{
    public int numSubsteps = 1; // VBD typically uses fewer substeps and more iterations
    public int vbdIterations = 100; // Block coordinate descent iterations

    // Kept as compliance for inspector parity with XPBD, converted internally
    public float stretchingCompliance = 1e-6f;
    public float shearCompliance = 0.0001f;
    public float bendingCompliance = 0.001f;

    public bool logMsPerFrame = true;
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
    [HideInInspector] public Vector3[] positions;
    [HideInInspector] public Vector3[] velocities;
    [HideInInspector] public Vector3[] previousPosition;
    [HideInInspector] public Vector3[] inertiaPositions; // Needed for VBD inertia
    [HideInInspector] public Vector3[] restPositions;
    [HideInInspector] public float[] masses; // VBD uses mass directly in Hessian
    [HideInInspector] public float[] invMasses;

    private int[] meshToGrid;
    private Vector3[] renderVertices;

    // VBD needs vertex-centric adjacency, not a flat constraint list
    public struct VBDEdge
    {
        public int neighborIdx;
        public float restLength;
        public float stiffness;
    }

    // Array of arrays is faster to iterate than List of Lists in Unity's hot path
    private VBDEdge[][] vertexEdges;

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
        previousPosition = new Vector3[numVerts];
        inertiaPositions = new Vector3[numVerts];
        restPositions = new Vector3[numVerts];

        masses = new float[numVerts];
        invMasses = new float[numVerts];
        Array.Fill(masses, 1f);
        Array.Fill(invMasses, 1f);

        int topLeftIdx = (numY - 1) * numX;
        int topRightIdx = (numY - 1) * numX + (numX - 1);

        invMasses[topLeftIdx] = 0f; invMasses[topRightIdx] = 0f;

        BuildSimulationGrid(xCoords, yCoords);
        Array.Copy(positions, restPositions, numVerts);

        BuildMeshToGrid(localVerts, xCoords, yCoords, eps);
        InitConstraints();

        if (addInitNoise)
            for (int i = 0; i < numVerts; i++)
                if (invMasses[i] > 0f)
                    positions[i] += UnityEngine.Random.insideUnitSphere * 0.001f;
    }

    void Update()
    {
        bool shouldLogPerformance = logMsPerFrame && Time.frameCount % logEveryNFrames == 0;
        double simStartTime = shouldLogPerformance ? Time.realtimeSinceStartupAsDouble : 0;

        float dt = 1 / 24f; // Time.deltaTime
        float sdt = dt / numSubsteps;
        float invSdt2 = 1.0f / (sdt * sdt);

        for (int step = 0; step < numSubsteps; step++)
        {
            // 1. Predict Inertia Positions
            for (int i = 0; i < numVerts; i++)
            {
                if (invMasses[i] == 0f) continue;
                velocities[i] += gravity * sdt;

                previousPosition[i] = positions[i]; // Store x^n
                inertiaPositions[i] = positions[i] + velocities[i] * sdt; // x_tilde
                positions[i] = inertiaPositions[i]; // Start solver at inertia
            }

            // 2. Vertex Block Descent Solver
            for (int iter = 0; iter < vbdIterations; iter++)
            {
                for (int i = 0; i < numVerts; i++)
                {
                    if (invMasses[i] == 0f) continue;

                    float m = masses[i];

                    // Initialize Hessian (H) and Negative Gradient (f) with inertia term
                    Mat3 H = Mat3.Identity(m * invSdt2);
                    Vector3 f = (m * invSdt2) * (inertiaPositions[i] - positions[i]);

                    VBDEdge[] edges = vertexEdges[i];
                    for (int e = 0; e < edges.Length; e++)
                    {
                        int j = edges[e].neighborIdx;
                        float l0 = edges[e].restLength;
                        float k = edges[e].stiffness;

                        Vector3 diff = positions[i] - positions[j];
                        float l = diff.magnitude;

                        if (l < 1e-6f) continue;

                        float l0_over_l = l0 / l;

                        // f += (stiffness * (l0 - l) / l) * diff; 
                        // Which is equivalent to: k * (l0/l - 1) * diff
                        f += (k * (l0_over_l - 1f)) * diff;

                        // Evaluate Hessian for this edge
                        // h_edge = stiffness * ( I - (l0/l)*( I - (diff*diff^T)/(l*l) ) )
                        float coeff1 = k * (1f - l0_over_l);
                        float coeff2 = k * l0_over_l / (l * l);

                        // Accumulate symmetric 3x3 Hessian
                        H.m00 += coeff1 + coeff2 * (diff.x * diff.x);
                        H.m11 += coeff1 + coeff2 * (diff.y * diff.y);
                        H.m22 += coeff1 + coeff2 * (diff.z * diff.z);

                        H.m01 += coeff2 * (diff.x * diff.y);
                        H.m02 += coeff2 * (diff.x * diff.z);
                        H.m12 += coeff2 * (diff.y * diff.z);
                    }

                    // Mirror symmetric components
                    H.m10 = H.m01;
                    H.m20 = H.m02;
                    H.m21 = H.m12;

                    // Newton step: dx = H^-1 * f
                    Vector3 dx = H.Solve(f);
                    positions[i] += dx;
                }
            }

            OnUpdate?.Invoke();

            // 3. Update Velocities
            for (int i = 0; i < numVerts; i++)
            {
                if (invMasses[i] == 0f) continue;
                velocities[i] = (positions[i] - previousPosition[i]) / sdt;
            }
        }

        // Render update
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

    void OnGUI()
    {
        if (!logMsPerFrame) return;

        if (performanceStyle == null)
        {
            performanceStyle = new GUIStyle(GUI.skin.label);
            performanceStyle.fontSize = 14;
            performanceStyle.normal.textColor = Color.white;
        }

        GUI.Label(new Rect(10, 50, 420, 30), performanceText, performanceStyle);
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

    private void InitConstraints()
    {
        var tempAdjacency = new List<VBDEdge>[numVerts];
        for (int i = 0; i < numVerts; i++) tempAdjacency[i] = new List<VBDEdge>();

        // Convert compliances to stiffness (k = 1 / alpha)
        float kStretch = stretchingCompliance > 0 ? 1f / stretchingCompliance : float.MaxValue;
        float kShear = shearCompliance > 0 ? 1f / shearCompliance : float.MaxValue;
        float kBend = bendingCompliance > 0 ? 1f / bendingCompliance : float.MaxValue;

        AddEdges(tempAdjacency, 0, 0, 1, 0, kStretch); // stretch horizontal
        AddEdges(tempAdjacency, 0, 0, 0, 1, kStretch); // stretch vertical
        AddEdges(tempAdjacency, 0, 0, 1, 1, kShear);   // shear down
        AddEdges(tempAdjacency, 1, 0, 0, 1, kShear);   // shear up
        AddEdges(tempAdjacency, 0, 0, 2, 0, kBend);    // bend horizontal
        AddEdges(tempAdjacency, 0, 0, 0, 2, kBend);    // bend vertical

        // Bake to arrays for cache locality and iteration speed
        vertexEdges = new VBDEdge[numVerts][];
        for (int i = 0; i < numVerts; i++)
        {
            vertexEdges[i] = tempAdjacency[i].ToArray();
        }
    }

    private void AddEdges(List<VBDEdge>[] adjacency, int offset_i0, int offset_j0, int offset_i1, int offset_j1, float stiffness)
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

                    float dist = Vector3.Distance(positions[p1], positions[p2]);

                    // Add to p1's adjacency
                    adjacency[p1].Add(new VBDEdge { neighborIdx = p2, restLength = dist, stiffness = stiffness });
                    // Add to p2's adjacency (undirected graph)
                    adjacency[p2].Add(new VBDEdge { neighborIdx = p1, restLength = dist, stiffness = stiffness });
                }
            }
    }

    /// <summary>
    /// Lightweight 3x3 Matrix struct specifically for Cramer's Rule inversions in the Newton step.
    /// Eliminates the need for heavy matrix libraries like Eigen in C#.
    /// </summary>
    private struct Mat3
    {
        public float m00, m01, m02;
        public float m10, m11, m12;
        public float m20, m21, m22;

        public static Mat3 Identity(float scalar)
        {
            return new Mat3
            {
                m00 = scalar,
                m11 = scalar,
                m22 = scalar
            };
        }

        public Vector3 Solve(Vector3 b)
        {
            // Determinant of the 3x3 matrix
            float det = m00 * (m11 * m22 - m12 * m21)
                      - m01 * (m10 * m22 - m12 * m20)
                      + m02 * (m10 * m21 - m11 * m20);

            if (Mathf.Abs(det) < 1e-8f) return Vector3.zero;

            float invDet = 1.0f / det;

            // Cramer's rule for x, y, z
            float dx = b.x * (m11 * m22 - m12 * m21)
                     - m01 * (b.y * m22 - m12 * b.z)
                     + m02 * (b.y * m21 - m11 * b.z);

            float dy = m00 * (b.y * m22 - m12 * b.z)
                     - b.x * (m10 * m22 - m12 * m20)
                     + m02 * (m10 * b.z - b.y * m20);

            float dz = m00 * (m11 * b.z - b.y * m21)
                     - m01 * (m10 * b.z - b.y * m20)
                     + b.x * (m10 * m21 - m11 * m20);

            return new Vector3(dx * invDet, dy * invDet, dz * invDet);
        }
    }
}