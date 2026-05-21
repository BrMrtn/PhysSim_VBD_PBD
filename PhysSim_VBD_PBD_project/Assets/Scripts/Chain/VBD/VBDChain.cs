using UnityEngine;

[RequireComponent(typeof(LineRenderer))]
public class VBDChain : MonoBehaviour
{
    public float frameRate = 30f;
    public int numSubsteps = 1;
    public int numIterations = 15;

    // Chebyshev semi-iterative acceleration
    public bool useAcceleration = false;
    [Range(0f, 1f)] public float accelerationRho = 0.5f;

    public int numParticles = 20;
    public float restLength = 1f;

    public Vector3 bobStartPosition = new Vector3(-19f, 0f, 0f);

    public float stretchingStiffness = 1e6f;
    public bool hasBendingConstraints = false;
    public float bendingStiffness = 1e5f;

    public float rayleighMassDamping = 0f;
    public float rayleighStiffnessDamping = 0f;

    public bool addInitNoise = false;
    public bool logMsPerFrame = true;
    public bool logEnergy = false;
    public bool logAmplitude = false;

    public Material sphereMaterial;

    public VBDSolver Solver { get; private set; }
    private EnergyLogger energyLogger;
    private AmplitudeLogger amplitudeLogger;

    private float dt;
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
        dt = 1f / Mathf.Max(1f, frameRate);
        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.positionCount = numParticles;

        var cfg = ChainConfig.Default(numParticles);
        cfg.restLength = restLength;
        cfg.stretchingStiffness = stretchingStiffness;
        cfg.hasBendingConstraints = hasBendingConstraints;
        cfg.bendingStiffness = bendingStiffness;
        cfg.start = tr.position;
        cfg.bob = bobStartPosition;

        Solver = ChainFactory.BuildVBD(cfg);
        Solver.numSubsteps = numSubsteps;
        Solver.numIterations = numIterations;
        Solver.useAcceleration = useAcceleration;
        Solver.accelerationRho = accelerationRho;
        Solver.rayleighMassDamping = rayleighMassDamping;
        Solver.rayleighStiffnessDamping = rayleighStiffnessDamping;

        renderPositions = new Vector3[numParticles];

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

        if (logAmplitude)
        {
            amplitudeLogger = gameObject.AddComponent<AmplitudeLogger>();
            amplitudeLogger.label = "VBDChain";
            amplitudeLogger.overlayY = 70f;
            amplitudeLogger.Sampler = () => AmplitudeSampler.Sample(Solver);
        }
    }

    void Update()
    {
        bool shouldLogPerformance = logMsPerFrame && Time.frameCount % logEveryNFrames == 0;
        double simStartTime = shouldLogPerformance ? Time.realtimeSinceStartupAsDouble : 0;

        Solver.Step(dt);
        if (logEnergy) energyLogger.Log(dt);
        if (logAmplitude) amplitudeLogger.Log(dt);

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
