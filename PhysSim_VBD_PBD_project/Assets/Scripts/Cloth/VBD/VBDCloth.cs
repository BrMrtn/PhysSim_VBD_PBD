using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Rendering;

public class VBDCloth : MonoBehaviour
{
    public float frameRate = 30f;
    public int numSubsteps = 1;
    public int numIterations = 15;

    // Chebyshev semi-iterative acceleration
    public bool useAcceleration = false;
    [Range(0f, 1f)] public float accelerationRho = 0.5f;

    public float stretchingStiffness = 1e6f;
    public float shearStiffness = 1e4f;
    public float bendingStiffness = 1e3f;

    public float totalMass = 1f;

    public bool handleSelfCollisions = false;
    public float selfCollisionStiffness = 1e7f;
    public float velCapPerFrame = 3f;

    public float rayleighMassDamping = 0f;
    public float rayleighStiffnessDamping = 0f;

    public bool useLineSearch = true;
    public int maxLineSearchIters = 8;

    public bool addInitNoise = false;
    public bool logMsPerFrame = true;
    public bool logEnergy = false;
    public bool logArea = false;
    public bool logResidual = false;
    public bool logSpringLength = false;

    public VBDSolver Solver { get; private set; }
    private EnergyLogger energyLogger;
    private AreaLogger areaLogger;
    private ResidualLogger residualLogger;
    private SpringLengthLogger springLengthLogger;
    private bool initialLogged;

    private float dt;
    private float spacing;
    private float thickness;

    private const int logEveryNFrames = 10;
    private string performanceText = "VBD Simulation: -- ms/frame";
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

        float eps = 0.001f;
        List<float> xCoords = GetDistinctSortedCoordinates(localVerts, v => v.x, eps);
        List<float> yCoords = GetDistinctSortedCoordinates(localVerts, v => v.y, eps);
        numX = xCoords.Count;
        numY = yCoords.Count;

        if (numX * numY != numVerts)
        {
            Debug.LogError($"VBDCloth: Expected a grid mesh with numX * numY = {numVerts}, but got numX = {numX}, numY = {numY}.");
            return;
        }

        renderVertices = new Vector3[numVerts];

        if (numX > 1) spacing = xCoords[1] - xCoords[0];
        thickness = spacing;

        Solver = new VBDSolver(numVerts)
        {
            numSubsteps = numSubsteps,
            numIterations = numIterations,
            useAcceleration = useAcceleration,
            accelerationRho = accelerationRho,
            handleSelfCollisions = handleSelfCollisions,
            selfCollisionStiffness = selfCollisionStiffness,
            thickness = thickness,
            velCapPerFrame = velCapPerFrame,
            rayleighMassDamping = rayleighMassDamping,
            rayleighStiffnessDamping = rayleighStiffnessDamping,
            useLineSearch = useLineSearch,
            maxLineSearchIters = maxLineSearchIters,
            computeResidual = logResidual
        };

        BuildSimulationGrid(xCoords, yCoords);
        AssignMasses();
        Array.Copy(Solver.positions, Solver.restPositions, numVerts);

        BuildMeshToGrid(localVerts, xCoords, yCoords, eps);
        InitSprings();

        if (addInitNoise)
            for (int i = 0; i < numVerts; i++)
                if (Solver.invMasses[i] > 0f)
                    Solver.positions[i] += UnityEngine.Random.insideUnitSphere * 0.00001f;

        if (handleSelfCollisions) Solver.CreateSpatialHash(spacing);

        if (logEnergy)
        {
            energyLogger = gameObject.AddComponent<EnergyLogger>();
            energyLogger.label = "VBDCloth";
            energyLogger.overlayY = 30f;
            energyLogger.Sampler = () => EnergySampler.Sample(Solver);
        }

        if (logArea)
        {
            areaLogger = gameObject.AddComponent<AreaLogger>();
            areaLogger.label = "VBDCloth";
            areaLogger.overlayY = 50f;
            areaLogger.showOverlay = false;
            areaLogger.Sampler = () => AreaSampler.Sample(Solver.positions, Solver.restPositions, numX, numY);
        }

        if (logResidual)
        {
            residualLogger = gameObject.AddComponent<ResidualLogger>();
            residualLogger.label = "VBDCloth";
            residualLogger.overlayY = 70f;
            residualLogger.Sampler = () => ResidualSampler.Sample(Solver);
        }

        if (logSpringLength)
        {
            springLengthLogger = gameObject.AddComponent<SpringLengthLogger>();
            springLengthLogger.label = "VBDCloth";
            springLengthLogger.overlayY = 90f;
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
        if (logResidual) residualLogger.Log(dt);
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
            Solver.masses[i] = m;
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

    private void InitSprings()
    {
        var perVertEdges = new List<VertexSpringEdge>[numVerts];
        for (int i = 0; i < numVerts; i++) perVertEdges[i] = new List<VertexSpringEdge>();

        AddSprings(perVertEdges, 0, 0, 1, 0, stretchingStiffness); // stretch horizontal
        AddSprings(perVertEdges, 0, 0, 0, 1, stretchingStiffness); // stretch vertical
        AddSprings(perVertEdges, 0, 0, 1, 1, shearStiffness);      // shear  /
        AddSprings(perVertEdges, 1, 0, 0, 1, shearStiffness);      // shear  \
        AddSprings(perVertEdges, 0, 0, 2, 0, bendingStiffness);    // bend horizontal
        AddSprings(perVertEdges, 0, 0, 0, 2, bendingStiffness);    // bend vertical

        // Flatten per-vertex lists into CSR (springListStart + springEdges).
        var listStart = new int[numVerts + 1];
        int running = 0;
        for (int i = 0; i < numVerts; i++)
        {
            listStart[i] = running;
            running += perVertEdges[i].Count;
        }
        listStart[numVerts] = running;

        var flat = new VertexSpringEdge[running];
        for (int i = 0; i < numVerts; i++)
        {
            var list = perVertEdges[i];
            int s = listStart[i];
            for (int j = 0; j < list.Count; j++)
                flat[s + j] = list[j];
        }

        Solver.springEdges = flat;
        Solver.springListStart = listStart;
    }

    private void AddSprings(List<VertexSpringEdge>[] perVertEdges,
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

                    float restLen = Vector3.Distance(Solver.positions[p1], Solver.positions[p2]);
                    float scaledStiffness = stiffness / restLen;

                    perVertEdges[p1].Add(new VertexSpringEdge { otherIdx = p2, restLength = restLen, stiffness = scaledStiffness });
                    perVertEdges[p2].Add(new VertexSpringEdge { otherIdx = p1, restLength = restLen, stiffness = scaledStiffness });
                }
            }
    }
}
