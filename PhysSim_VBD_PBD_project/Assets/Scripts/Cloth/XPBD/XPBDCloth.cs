using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Rendering;

public class XPBDCloth : MonoBehaviour
{
    public int numSubsteps = 15;
    public int numIterations = 1;

    public float stretchingCompliance = 1e-6f;
    public float shearCompliance = 0.0001f;
    public float bendingCompliance = 0.001f;

    public bool handleSelfCollisions = false;
    public float selfCollisionFriction = 0f;
    public float velCapPerFrame = 3f;

    public float rayleighMassDamping = 0f;
    public float rayleighStiffnessDamping = 0f;

    public bool logMsPerFrame = true;
    public bool logEnergy = false;
    public bool addInitNoise = false;

    public XPBDSolver Solver { get; private set; }
    private EnergyLogger energyLogger;

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
    }

    void Update()
    {
        bool shouldLogPerformance = logMsPerFrame && Time.frameCount % logEveryNFrames == 0;
        double simStartTime = shouldLogPerformance ? Time.realtimeSinceStartupAsDouble : 0;

        float dt = 1 / 24f;
        Solver.Step(dt);
        if (logEnergy) energyLogger.Log(dt);

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
        AddConstraint(constraintsList, 0, 0, 2, 0, bendingCompliance);        // bend horizontal
        AddConstraint(constraintsList, 0, 0, 0, 2, bendingCompliance);        // bend vertical

        Solver.constraints = constraintsList.ToArray();
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

                    constraintsList.Add(new DistanceConstraint
                    {
                        p1Idx = p1,
                        p2Idx = p2,
                        restLength = Vector3.Distance(Solver.positions[p1], Solver.positions[p2]),
                        compliance = compliance
                    });
                }
            }
    }
}
