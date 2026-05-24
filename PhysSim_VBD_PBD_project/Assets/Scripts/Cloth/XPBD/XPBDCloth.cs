using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Rendering;

public class XPBDCloth : MonoBehaviour
{
    public float frameRate = 30f;
    public int numSubsteps = 15;
    public int numIterations = 1;

    public float stretchingCompliance = 1e-6f;
    public float shearCompliance = 0.0001f;
    public float bendingCompliance = 0.001f;

    // When set, the i,i+2 distance "bending" springs are replaced by true
    // dihedral-angle bending constraints over each interior hinge.
    public bool useDihedralBending = false;
    public float dihedralBendingCompliance = 0.01f;

    public float totalMass = 1f;

    public bool handleSelfCollisions = false;
    public float selfCollisionFriction = 0f;
    public float velCapPerFrame = 3f;

    public float rayleighMassDamping = 0f;
    public float rayleighStiffnessDamping = 0f;

    public bool addInitNoise = false;
    public bool logMsPerFrame = true;
    public bool logEnergy = false;
    public bool logArea = false;
    public bool logSpringLength = false;

    public XPBDSolver Solver { get; private set; }
    private EnergyLogger energyLogger;
    private AreaLogger areaLogger;
    private SpringLengthLogger springLengthLogger;
    private bool initialLogged;

    private float dt;
    private float spacing;
    private float thickness;

    private const int logEveryNFrames = 10;
    private string performanceText = "XPBD Simulation: -- ms/frame";
    private GUIStyle performanceStyle;

    private MeshFilter meshFilter;
    private Mesh mesh;
    private Transform tr;

    [HideInInspector] public int numVerts;
    [HideInInspector] public int numX;
    [HideInInspector] public int numY;

    private int[] meshToGrid;
    private Vector3[] renderVertices;

    void Awake()
    {
        meshFilter = gameObject.GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;
        tr = transform;
        dt = 1f / Mathf.Max(1f, frameRate);
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

        Solver = new XPBDSolver(numVerts)
        {
            numSubsteps = numSubsteps,
            numIterations = numIterations,
            handleSelfCollisions = handleSelfCollisions,
            selfCollisionFriction = selfCollisionFriction,
            thickness = thickness,
            velCapPerFrame = velCapPerFrame,
            rayleighMassDamping = rayleighMassDamping,
            rayleighStiffnessDamping = rayleighStiffnessDamping
        };

        BuildSimulationGrid(xCoords, yCoords);

        // The spatial hash and contact thickness operate on WORLD positions, so
        // the cell size must be the world spacing, not the local mesh spacing.
        // Imported FBX meshes carry a non-unit import scale, which makes those
        // two differ and would collapse every particle into a few hash cells.
        if (numX > 1)
        {
            spacing = (Solver.positions[1] - Solver.positions[0]).magnitude;
            thickness = spacing;
            Solver.thickness = thickness;
        }

        AssignMasses();
        Array.Copy(Solver.positions, Solver.restPositions, numVerts);

        BuildMeshToGrid(localVerts, xCoords, yCoords, eps);
        InitConstraints();

        if (addInitNoise)
            for (int i = 0; i < numVerts; i++)
                if (Solver.invMasses[i] > 0f)
                    Solver.positions[i] += UnityEngine.Random.insideUnitSphere * 0.001f;

        if (handleSelfCollisions) Solver.CreateSpatialHash(spacing);

        if (logEnergy)
        {
            energyLogger = gameObject.AddComponent<EnergyLogger>();
            energyLogger.label = "XPBDCloth";
            energyLogger.overlayY = 30f;
            energyLogger.Sampler = () => EnergySampler.Sample(Solver);
        }

        if (logArea)
        {
            areaLogger = gameObject.AddComponent<AreaLogger>();
            areaLogger.label = "XPBDCloth";
            areaLogger.overlayY = 50f;
            areaLogger.showOverlay = false;
            areaLogger.Sampler = () => AreaSampler.Sample(Solver.positions, Solver.restPositions, numX, numY);
        }

        if (logSpringLength)
        {
            springLengthLogger = gameObject.AddComponent<SpringLengthLogger>();
            springLengthLogger.label = "XPBDCloth";
            springLengthLogger.overlayY = 70f;
            springLengthLogger.showOverlay = false;
            springLengthLogger.writePerSpring = false;
            springLengthLogger.Sampler = () => SpringLengthSampler.SampleClothGrid(Solver.positions, Solver.restPositions, numX, numY);
        }
    }

    void Update()
    {
        bool shouldLogPerformance = logMsPerFrame && Time.frameCount % logEveryNFrames == 0;
        double simStartTime = shouldLogPerformance ? Time.realtimeSinceStartupAsDouble : 0;

        // Capture the initial (pre-step) configuration once at frame 0 / time 0.
        if (logSpringLength && !initialLogged)
        {
            springLengthLogger.LogInitial();
            initialLogged = true;
        }

        Solver.Step(dt);
        if (logEnergy) energyLogger.Log(dt);
        if (logArea) areaLogger.Log(dt);
        if (logSpringLength) springLengthLogger.Log(dt);

        Matrix4x4 worldToLocal = tr.worldToLocalMatrix;
        for (int i = 0; i < numVerts; i++)
            renderVertices[i] = worldToLocal.MultiplyPoint3x4(Solver.positions[meshToGrid[i]]);

        mesh.SetVertices(renderVertices, 0, numVerts, MeshUpdateFlags.DontRecalculateBounds);
        mesh.RecalculateNormals();

        if (shouldLogPerformance)
        {
            double simEndTime = Time.realtimeSinceStartupAsDouble;
            double msPerFrame = (simEndTime - simStartTime) * 1000.0;
            performanceText = $"XPBD Simulation: {msPerFrame:F2} ms/frame";
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

        GUI.Label(new Rect(10, 10, 420, 30), performanceText, performanceStyle);
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
                Solver.positions[idx] = tr.TransformPoint(localPos);
            }
    }

    private void AssignMasses()
    {
        var tributaryArea = new float[numVerts];
        Vector3[] pos = Solver.positions;

        for (int iy = 0; iy < numY - 1; iy++)
            for (int ix = 0; ix < numX - 1; ix++)
            {
                int a = iy * numX + ix;
                int b = iy * numX + (ix + 1);
                int c = (iy + 1) * numX + ix;
                int d = (iy + 1) * numX + (ix + 1);

                float quarter = 0.25f * (TriangleArea(pos[a], pos[b], pos[d]) + TriangleArea(pos[a], pos[d], pos[c]));
                tributaryArea[a] += quarter;
                tributaryArea[b] += quarter;
                tributaryArea[c] += quarter;
                tributaryArea[d] += quarter;
            }

        float totalArea = 0f;
        for (int i = 0; i < numVerts; i++) totalArea += tributaryArea[i];
        if (totalArea <= 0f) return;

        float density = totalMass / totalArea;
        for (int i = 0; i < numVerts; i++)
        {
            float m = density * tributaryArea[i];
            Solver.invMasses[i] = m > 0f ? 1f / m : 0f;
        }
    }

    private static float TriangleArea(Vector3 p0, Vector3 p1, Vector3 p2)
        => 0.5f * Vector3.Cross(p1 - p0, p2 - p0).magnitude;

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
        var constraintsList = new List<DistanceConstraint>();

        AddConstraint(constraintsList, 0, 0, 1, 0, stretchingCompliance);     // stretch horizontal
        AddConstraint(constraintsList, 0, 0, 0, 1, stretchingCompliance);     // stretch vertical
        AddConstraint(constraintsList, 0, 0, 1, 1, shearCompliance);          // shear down
        AddConstraint(constraintsList, 1, 0, 0, 1, shearCompliance);          // shear up

        if (!useDihedralBending)
        {
            AddConstraint(constraintsList, 0, 0, 2, 0, bendingCompliance);    // bend horizontal
            AddConstraint(constraintsList, 0, 0, 0, 2, bendingCompliance);    // bend vertical
        }

        Solver.constraints = constraintsList.ToArray();

        if (useDihedralBending)
        {
            var dihedrals = new List<DihedralConstraint>();
            AddDihedralConstraints(dihedrals);
            Solver.dihedralConstraints = dihedrals.ToArray();
        }
    }

    // Builds one dihedral hinge per interior triangle edge, using the same
    // (a,b,d)+(a,d,c) triangulation as the mass/area code. Each edge shared by
    // two triangles becomes a hinge whose wings are the two opposite vertices.
    private void AddDihedralConstraints(List<DihedralConstraint> list)
    {
        var firstWing = new Dictionary<long, int>();

        void ProcessEdge(int u, int v, int wing)
        {
            int a = Mathf.Min(u, v), b = Mathf.Max(u, v);
            long key = ((long)a << 32) | (uint)b;
            if (firstWing.TryGetValue(key, out int w0))
            {
                DihedralBending.ComputeGradients(
                    Solver.positions[a], Solver.positions[b],
                    Solver.positions[w0], Solver.positions[wing],
                    out float restAngle, out _, out _, out _, out _);
                list.Add(new DihedralConstraint
                {
                    p1Idx = a, p2Idx = b, p3Idx = w0, p4Idx = wing,
                    restAngle = restAngle,
                    compliance = dihedralBendingCompliance
                });
            }
            else firstWing[key] = wing;
        }

        void ProcessTriangle(int t0, int t1, int t2)
        {
            ProcessEdge(t0, t1, t2);
            ProcessEdge(t1, t2, t0);
            ProcessEdge(t2, t0, t1);
        }

        for (int iy = 0; iy < numY - 1; iy++)
            for (int ix = 0; ix < numX - 1; ix++)
            {
                int a = iy * numX + ix;
                int b = iy * numX + (ix + 1);
                int c = (iy + 1) * numX + ix;
                int d = (iy + 1) * numX + (ix + 1);
                ProcessTriangle(a, b, d);
                ProcessTriangle(a, d, c);
            }
    }

    private void AddConstraint(List<DistanceConstraint> constraintsList, int offset_i0, int offset_j0, int offset_i1, int offset_j1, float compliance)
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

                    float restLength = Vector3.Distance(Solver.positions[p1], Solver.positions[p2]);

                    constraintsList.Add(new DistanceConstraint
                    {
                        p1Idx = p1,
                        p2Idx = p2,
                        restLength = restLength,
                        compliance = compliance * restLength
                    });
                }
            }
    }
}
