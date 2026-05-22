using System;
using UnityEngine;

[RequireComponent(typeof(VBDCloth))]
public class VBDWindInteraction : MonoBehaviour
{
    public WindZone windZone;
    public float windDrag = 0.05f; // How strongly this cloth reacts to wind
    private const float TangentialFraction = 0.1f;

    private VBDCloth cloth;
    private VBDSolver solver;
    private int[] triangles;   // 3 grid-vertex indices per triangle
    private Vector3[] vertexForces; // cached wind force
    private int lastComputedFrame = -1;
    private float clothSize = 1f;

    void Start()
    {
        cloth = GetComponent<VBDCloth>();
        solver = cloth != null ? cloth.Solver : null;

        if (solver == null)
        {
            Debug.LogWarning("VBDWindInteraction: VBDCloth solver not found, disabling.", this);
            enabled = false;
            return;
        }

        if (windZone == null)
            Debug.LogWarning("VBDWindInteraction: no WindZone found in the scene.", this);

        BuildTriangles();
        vertexForces = new Vector3[solver.numVerts];
        clothSize = Mathf.Max(0.01f, MeasureClothSize());

        solver.OnPreSubstep += ApplyWind; // recompute (at most) once per frame, push into externalForces each substep
    }

    void OnDestroy()
    {
        if (solver != null)
            solver.OnPreSubstep -= ApplyWind;
    }

    /// <summary>Regular triangulation of the simulation grid.</summary>
    private void BuildTriangles()
    {
        int numX = cloth.numX;
        int numY = cloth.numY;
        int quadsX = Mathf.Max(0, numX - 1);
        int quadsY = Mathf.Max(0, numY - 1);

        triangles = new int[quadsX * quadsY * 6];
        int t = 0;

        for (int iy = 0; iy < quadsY; iy++)
        {
            for (int ix = 0; ix < quadsX; ix++)
            {
                int v00 = iy * numX + ix;
                int v10 = v00 + 1;
                int v01 = v00 + numX;
                int v11 = v01 + 1;

                triangles[t++] = v00; triangles[t++] = v10; triangles[t++] = v01;
                triangles[t++] = v10; triangles[t++] = v11; triangles[t++] = v01;
            }
        }
    }

    private float MeasureClothSize()
    {
        Vector3[] rest = solver.restPositions;
        Vector3 min = rest[0], max = rest[0];
        for (int i = 1; i < rest.Length; i++)
        {
            min = Vector3.Min(min, rest[i]);
            max = Vector3.Max(max, rest[i]);
        }
        return (max - min).magnitude;
    }

    private void ApplyWind()
    {
        if (!isActiveAndEnabled || windZone == null || triangles == null)
            return;

        if (Time.frameCount != lastComputedFrame)
        {
            ComputeWindForces();
            lastComputedFrame = Time.frameCount;
        }

        // Wind is linear in x, so it has no Hessian contribution — just an
        // additive constant on f for this substep. Accumulate per-vertex into
        // externalForces (cleared by the solver at the start of every substep).
        Vector3[] external = solver.externalForces;
        for (int i = 0; i < external.Length; i++)
            external[i] += vertexForces[i];
    }

    private void ComputeWindForces()
    {
        Array.Clear(vertexForces, 0, vertexForces.Length);

        Vector3[] positions = solver.positions;
        Vector3[] velocities = solver.velocities;

        float tangentialDrag = windDrag * TangentialFraction;

        for (int i = 0; i < triangles.Length; i += 3)
        {
            int a = triangles[i];
            int b = triangles[i + 1];
            int c = triangles[i + 2];

            Vector3 pa = positions[a];
            Vector3 pb = positions[b];
            Vector3 pc = positions[c];

            // Face normal and area, with a degeneracy guard.
            Vector3 cross = Vector3.Cross(pb - pa, pc - pa);
            float crossMag = cross.magnitude;
            if (crossMag < 1e-8f) continue;
            Vector3 normal = cross / crossMag;
            float area = 0.5f * crossMag; // |(b-a) x (c-a)| / 2

            Vector3 center = (pa + pb + pc) * (1f / 3f);
            Vector3 triVelocity = (velocities[a] + velocities[b] + velocities[c]) * (1f / 3f);

            // Air velocity relative to the moving cloth.
            Vector3 relVelocity = SampleWind(center) - triVelocity;

            float vn = Vector3.Dot(relVelocity, normal);
            Vector3 force = normal * (vn * Mathf.Abs(vn) * windDrag);

            Vector3 tangential = relVelocity - vn * normal;
            force += tangential * tangentialDrag;

            // Aerodynamic force scales with face area, so refining the mesh keeps the
            // total force constant instead of growing with vertex count. (VBD's implicit
            // solve tolerates the unscaled force, but XPBD does not, and keeping both
            // wind models identical matters for a fair solver comparison.)
            force *= area;

            Vector3 perVertex = force * (1f / 3f);
            vertexForces[a] += perVertex;
            vertexForces[b] += perVertex;
            vertexForces[c] += perVertex;
        }
    }

    /// <summary>Builds a wind velocity vector from the WindZone at a given world point.</summary>
    private Vector3 SampleWind(Vector3 worldPoint)
    {
        Vector3 direction;
        float speed = windZone.windMain;

        if (windZone.mode != WindZoneMode.Directional)
            Debug.LogWarning("VBDWindInteraction: only WindZoneMode.Directional is supported", this);

        direction = windZone.transform.forward;

        // Slow swelling / fading of the main wind.
        speed *= 1f + windZone.windPulseMagnitude *
                 Mathf.Sin(Time.time * windZone.windPulseFrequency * 2f * Mathf.PI);

        Vector3 wind = direction * speed;

        // Gusty turbulence via Perlin noise in time and space.
        float gust = windZone.windTurbulence;
        if (gust > 0f)
        {
            float spatial = 2f / clothSize;
            float temporal = (Mathf.Abs(windZone.windMain) + gust) / clothSize;
            float t = Time.time * temporal;
            float gx = Mathf.PerlinNoise(t, worldPoint.y * spatial) - 0.5f;
            float gy = Mathf.PerlinNoise(t + 37.2f, worldPoint.z * spatial) - 0.5f;
            float gz = Mathf.PerlinNoise(t + 71.9f, worldPoint.x * spatial) - 0.5f;
            wind += new Vector3(gx, gy, gz) * (2f * gust);
        }

        return wind;
    }
}
