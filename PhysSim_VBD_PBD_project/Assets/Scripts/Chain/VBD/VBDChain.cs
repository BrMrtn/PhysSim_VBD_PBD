using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(LineRenderer))]
public class VBDChain : MonoBehaviour
{
    public int numSubsteps = 1;
    public int numIterations = 15;
    public int numParticles = 20;
    public float restLength = 1f;
    public float stretchingStiffness = 1e6f;
    public bool hasBendingConstraints = false;
    public float bendingStiffness = 1e5f;
    public bool logMsPerFrame = true;
    public bool addInitNoise = false;
    public Material sphereMaterial;

    // Chebyshev semi-iterative acceleration
    public bool useAcceleration = false;
    [Range(0f, 1f)] public float accelerationRho = 0.5f;

    public VBDSolver Solver { get; private set; }

    private LineRenderer lineRenderer;
    private Transform tr;
    private Vector3[] renderPositions;
    private GameObject[] vertexSpheres;

    private const int logEveryNFrames = 10;
    private string performanceText = "VBD Chain: -- ms/frame";
    private GUIStyle performanceStyle;

    void Awake()
    {
        tr = transform;
        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.positionCount = numParticles;

        Solver = new VBDSolver(numParticles)
        {
            numSubsteps = numSubsteps,
            numIterations = numIterations,
            useAcceleration = useAcceleration,
            accelerationRho = accelerationRho,
            thickness = restLength
        };

        // Initialize particles in a straight line extending to the left
        Vector3 start = tr.position;
        Vector3 step = -tr.right * restLength;
        for (int i = 0; i < numParticles; i++)
            Solver.positions[i] = start + step * i;

        // Build per-incidence edge lists (each spring stored once per endpoint).
        var perVertEdges = new List<VertexSpringEdge>[numParticles];
        for (int i = 0; i < numParticles; i++) perVertEdges[i] = new List<VertexSpringEdge>();

        // Stretching springs between consecutive particles
        for (int i = 0; i < numParticles - 1; i++)
        {
            float restLen = Vector3.Distance(Solver.positions[i], Solver.positions[i + 1]);
            perVertEdges[i].Add(new VertexSpringEdge { otherIdx = i + 1, restLength = restLen, stiffness = stretchingStiffness });
            perVertEdges[i + 1].Add(new VertexSpringEdge { otherIdx = i, restLength = restLen, stiffness = stretchingStiffness });
        }

        // Bending springs connecting every second vertex
        if (hasBendingConstraints)
        {
            for (int i = 0; i < numParticles - 2; i++)
            {
                float restLen = Vector3.Distance(Solver.positions[i], Solver.positions[i + 2]);
                perVertEdges[i].Add(new VertexSpringEdge { otherIdx = i + 2, restLength = restLen, stiffness = bendingStiffness });
                perVertEdges[i + 2].Add(new VertexSpringEdge { otherIdx = i, restLength = restLen, stiffness = bendingStiffness });
            }
        }

        // Flatten into CSR (springListStart + springEdges).
        Solver.springListStart = new int[numParticles + 1];
        int total = 0;
        for (int i = 0; i < numParticles; i++)
        {
            Solver.springListStart[i] = total;
            total += perVertEdges[i].Count;
        }
        Solver.springListStart[numParticles] = total;

        Solver.springEdges = new VertexSpringEdge[total];
        for (int i = 0; i < numParticles; i++)
        {
            int s = Solver.springListStart[i];
            var list = perVertEdges[i];
            for (int j = 0; j < list.Count; j++)
                Solver.springEdges[s + j] = list[j];
        }

        renderPositions = new Vector3[numParticles];

        Solver.invMasses[0] = 0f; // Fix the first particle in place

        if (addInitNoise)
            for (int i = 0; i < numParticles; i++)
                if (Solver.invMasses[i] > 0f)
                    Solver.positions[i] += Random.insideUnitSphere * 0.001f;

        // Create small spheres at each vertex position
        float sphereRadius = restLength / 10f;
        vertexSpheres = new GameObject[numParticles];
        for (int i = 0; i < numParticles; i++)
        {
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.name = $"VertexSphere_{i}";
            sphere.transform.SetParent(tr);
            sphere.transform.position = Solver.positions[i];
            sphere.transform.localScale = Vector3.one * sphereRadius * 2f;

            if (sphereMaterial != null)
            {
                Renderer renderer = sphere.GetComponent<Renderer>();
                if (renderer != null)
                    renderer.material = sphereMaterial;
            }

            Collider collider = sphere.GetComponent<Collider>();
            if (collider != null)
                DestroyImmediate(collider);

            vertexSpheres[i] = sphere;
        }
    }

    void Update()
    {
        bool shouldLogPerformance = logMsPerFrame && Time.frameCount % logEveryNFrames == 0;
        double simStartTime = shouldLogPerformance ? Time.realtimeSinceStartupAsDouble : 0;

        Solver.numSubsteps = numSubsteps;
        Solver.numIterations = numIterations;
        Solver.useAcceleration = useAcceleration;
        Solver.accelerationRho = accelerationRho;

        float dt = 1f / 24f;
        Solver.Step(dt);

        for (int i = 0; i < numParticles; i++)
        {
            renderPositions[i] = Solver.positions[i];
            vertexSpheres[i].transform.position = Solver.positions[i];
        }
        lineRenderer.SetPositions(renderPositions);

        if (shouldLogPerformance)
        {
            double simEndTime = Time.realtimeSinceStartupAsDouble;
            double msPerFrame = (simEndTime - simStartTime) * 1000.0;
            performanceText = $"VBD Chain: {msPerFrame:F2} ms/frame";
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

        GUI.Label(new Rect(10, 30, 420, 30), performanceText, performanceStyle);
    }
}
