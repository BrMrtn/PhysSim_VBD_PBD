using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(LineRenderer))]
public class VBDChain : MonoBehaviour
{
    public int numSubsteps = 1;
    public int numIterations = 15;

    // Chebyshev semi-iterative acceleration
    public bool useAcceleration = false;
    [Range(0f, 1f)] public float accelerationRho = 0.5f;

    public int numParticles = 20;
    public float restLength = 1f;

    // World-space position of the chain's end (the bob). The fixed start is at
    // this transform's position; all particles are spaced equally between them.
    public Vector3 bobStartPosition = new Vector3(-19f, 0f, 0f);

    public float stretchingStiffness = 1e6f;
    public bool hasBendingConstraints = false;
    public float bendingStiffness = 1e5f;

    public float rayleighMassDamping = 0f;
    public float rayleighStiffnessDamping = 0f;

    public bool addInitNoise = false;
    public bool logMsPerFrame = true;
    public bool logEnergy = false;

    public Material sphereMaterial;

    public VBDSolver Solver { get; private set; }
    private EnergyLogger energyLogger;

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
            thickness = restLength,
            rayleighMassDamping = rayleighMassDamping,
            rayleighStiffnessDamping = rayleighStiffnessDamping
        };

        // Initialize particles equally spaced on the line from the fixed start
        // (this transform's position) to the bob's start position.
        Vector3 start = tr.position;
        Vector3 step = (bobStartPosition - start) / Mathf.Max(1, numParticles - 1);
        for (int i = 0; i < numParticles; i++)
            Solver.positions[i] = start + step * i;

        // Build per-incidence edge lists (each spring stored once per endpoint).
        var perVertEdges = new List<VertexSpringEdge>[numParticles];
        for (int i = 0; i < numParticles; i++) perVertEdges[i] = new List<VertexSpringEdge>();

        // Stretching springs between consecutive particles. Rest length is the
        // configured restLength, so the initial spacing may stretch/compress them.
        for (int i = 0; i < numParticles - 1; i++)
        {
            perVertEdges[i].Add(new VertexSpringEdge { otherIdx = i + 1, restLength = restLength, stiffness = stretchingStiffness });
            perVertEdges[i + 1].Add(new VertexSpringEdge { otherIdx = i, restLength = restLength, stiffness = stretchingStiffness });
        }

        // Bending springs connecting every second vertex (rest length = 2 * restLength).
        if (hasBendingConstraints)
        {
            for (int i = 0; i < numParticles - 2; i++)
            {
                perVertEdges[i].Add(new VertexSpringEdge { otherIdx = i + 2, restLength = 2f * restLength, stiffness = bendingStiffness });
                perVertEdges[i + 2].Add(new VertexSpringEdge { otherIdx = i, restLength = 2f * restLength, stiffness = bendingStiffness });
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

        if (logEnergy)
        {
            energyLogger = gameObject.AddComponent<EnergyLogger>();
            energyLogger.label = "VBDChain";
            energyLogger.overlayY = 50f;
            energyLogger.Sampler = () => EnergySampler.Sample(Solver);
        }
    }

    void Update()
    {
        bool shouldLogPerformance = logMsPerFrame && Time.frameCount % logEveryNFrames == 0;
        double simStartTime = shouldLogPerformance ? Time.realtimeSinceStartupAsDouble : 0;

        float dt = 1f / 24f;
        Solver.Step(dt);
        if (logEnergy) energyLogger.Log(dt);

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
