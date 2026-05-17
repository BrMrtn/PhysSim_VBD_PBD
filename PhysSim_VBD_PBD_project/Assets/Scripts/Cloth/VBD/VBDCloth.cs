using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Rendering;

public class VBDCloth : MonoBehaviour
{
    public int numSubsteps = 1;
    public int numIterations = 15;

    public float stretchingStiffness = 1e6f;
    public float shearStiffness = 1e4f;
    public float bendingStiffness = 1e3f;

    public bool handleSelfCollisions = false;
    public float selfCollisionStiffness = 1e7f;
    public float velCapPerFrame = 3f;

    public bool logMsPerFrame = true;
    public bool addInitNoise = false;

    // Chebyshev semi-iterative acceleration
    public bool useAcceleration = false;
    [Range(0f, 1f)] public float accelerationRho = 0.5f;

    public VBDSolver Solver { get; private set; }

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
            velCapPerFrame = velCapPerFrame
        };

        BuildSimulationGrid(xCoords, yCoords);
        Array.Copy(Solver.positions, Solver.restPositions, numVerts);

        BuildMeshToGrid(localVerts, xCoords, yCoords, eps);
        InitSprings();

        if (addInitNoise)
            for (int i = 0; i < numVerts; i++)
                if (Solver.invMasses[i] > 0f)
                    Solver.positions[i] += UnityEngine.Random.insideUnitSphere * 0.001f;

        if (handleSelfCollisions) Solver.CreateSpatialHash(spacing);
    }

    void Update()
    {
        bool shouldLogPerformance = logMsPerFrame && Time.frameCount % logEveryNFrames == 0;
        double simStartTime = shouldLogPerformance ? Time.realtimeSinceStartupAsDouble : 0;

        float dt = 1f / 24f;
        Solver.Step(dt);

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

                    perVertEdges[p1].Add(new VertexSpringEdge { otherIdx = p2, restLength = restLen, stiffness = stiffness });
                    perVertEdges[p2].Add(new VertexSpringEdge { otherIdx = p1, restLength = restLen, stiffness = stiffness });
                }
            }
    }
}
