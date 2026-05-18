using UnityEngine;

[RequireComponent(typeof(LineRenderer))]
public class XPBDChain : MonoBehaviour
{
    public int numSubsteps = 15;
    public int numIterations = 1;
    public int numParticles = 20;
    public float restLength = 1f;
    public float stretchingCompliance = 1e-6f;
    public bool hasBendingConstraints = false;
    public float bendingCompliance = 1e-5f;
    public bool logMsPerFrame = true;
    public Material sphereMaterial;

    public XPBDSolver Solver { get; private set; }

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
        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.positionCount = numParticles;

        Solver = new XPBDSolver(numParticles)
        {
            numSubsteps = numSubsteps,
            numIterations = numIterations,
            handleSelfCollisions = false,
            thickness = restLength
        };

        // Initialize particles in a straight line extending to the left
        Vector3 start = tr.position;
        Vector3 step = -tr.right * restLength;
        for (int i = 0; i < numParticles; i++)
            Solver.positions[i] = start + step * i;

        // Build spring constraints between consecutive particles
        int constraintCount = numParticles - 1;
        if (hasBendingConstraints)
            constraintCount += numParticles - 2;

        Solver.constraints = new DistanceConstraint[constraintCount];

        // Add stretching constraints between consecutive particles
        for (int i = 0; i < numParticles - 1; i++)
        {
            Solver.constraints[i] = new DistanceConstraint(
                i,
                i + 1,
                Vector3.Distance(Solver.positions[i], Solver.positions[i + 1]),
                stretchingCompliance);
        }

        // Add bending constraints connecting every second vertex
        if (hasBendingConstraints)
        {
            int bendingStartIndex = numParticles - 1;
            for (int i = 0; i < numParticles - 2; i++)
            {
                Solver.constraints[bendingStartIndex + i] = new DistanceConstraint(
                    i,
                    i + 2,
                    Vector3.Distance(Solver.positions[i], Solver.positions[i + 2]),
                    bendingCompliance);
            }
        }

        renderPositions = new Vector3[numParticles];

        Solver.invMasses[0] = 0f; // Fix the first particle in place

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
    }

    void Update()
    {
        bool shouldLogPerformance = logMsPerFrame && Time.frameCount % logEveryNFrames == 0;
        double simStartTime = shouldLogPerformance ? Time.realtimeSinceStartupAsDouble : 0;

        Solver.numSubsteps = numSubsteps;
        Solver.numIterations = numIterations;

        float dt = 1 / 24f;
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
