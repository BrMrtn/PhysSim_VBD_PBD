using UnityEngine;

[RequireComponent(typeof(LineRenderer))]
public class XPBDChain : MonoBehaviour
{
    public float frameRate = 30f;
    public int numSubsteps = 15;
    public int numIterations = 1;

    public int numParticles = 20;
    public float restLength = 1f;

    public Vector3 bobStartPosition = new Vector3(-19f, 0f, 0f);

    public float stretchingCompliance = 1e-6f;
    public bool alternatingStiffness = false; // springs alternate soft, stiff, soft, ... (soft = 1/stretchingCompliance)
    public float stiffnessRatio = 10000f;      // stiff spring = soft * ratio
    public bool hasBendingConstraints = false;
    public float bendingCompliance = 1e-5f;

    public float rayleighMassDamping = 0f;
    public float rayleighStiffnessDamping = 0f;

    public bool logMsPerFrame = true;
    public bool logEnergy = false;
    public bool logSpringLength = false;

    public Material sphereMaterial;

    public XPBDSolver Solver { get; private set; }
    private EnergyLogger energyLogger;
    private SpringLengthLogger springLengthLogger;
    private bool initialLogged;

    private float dt;
    private LineRenderer lineRenderer;
    private Transform tr;
    private Vector3[] renderPositions;
    private GameObject[] vertexSpheres;

    private const int logEveryNFrames = 10;
    private string performanceText = "XPBD Chain: -- ms/frame";
    private GUIStyle performanceStyle;

    void Awake()
    {
        tr = transform;
        dt = 1f / Mathf.Max(1f, frameRate);
        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.positionCount = numParticles;

        var cfg = ChainConfig.Default(numParticles);
        cfg.restLength = restLength;
        cfg.stretchingStiffness = 1f / stretchingCompliance;
        if (alternatingStiffness) cfg.SetAlternatingStiffness(1f / stretchingCompliance, stiffnessRatio);
        cfg.hasBendingConstraints = hasBendingConstraints;
        cfg.bendingStiffness = 1f / bendingCompliance;
        cfg.start = tr.position;
        cfg.bob = bobStartPosition;

        Solver = ChainFactory.BuildXPBD(cfg);
        Solver.numSubsteps = numSubsteps;
        Solver.numIterations = numIterations;
        Solver.rayleighMassDamping = rayleighMassDamping;
        Solver.rayleighStiffnessDamping = rayleighStiffnessDamping;

        renderPositions = new Vector3[numParticles];

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

            // Apply material to the sphere if provided
            if (sphereMaterial != null)
            {
                Renderer renderer = sphere.GetComponent<Renderer>();
                if (renderer != null)
                    renderer.material = sphereMaterial;
            }

            // Remove collider from the sphere
            Collider collider = sphere.GetComponent<Collider>();
            if (collider != null)
                DestroyImmediate(collider);

            vertexSpheres[i] = sphere;
        }

        if (logEnergy)
        {
            energyLogger = gameObject.AddComponent<EnergyLogger>();
            energyLogger.label = "XPBDChain";
            energyLogger.overlayY = 50f;
            energyLogger.Sampler = () => EnergySampler.Sample(Solver);
        }

        if (logSpringLength)
        {
            springLengthLogger = gameObject.AddComponent<SpringLengthLogger>();
            springLengthLogger.label = "XPBDChain";
            springLengthLogger.overlayY = 70f;
            springLengthLogger.Sampler = () => SpringLengthSampler.Sample(Solver);
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
        if (logSpringLength) springLengthLogger.Log(dt);

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
            performanceText = $"XPBD Chain: {msPerFrame:F2} ms/frame";
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
