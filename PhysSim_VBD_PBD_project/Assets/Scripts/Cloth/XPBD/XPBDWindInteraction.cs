using System;
using UnityEngine;

/// <summary>
/// Connects a Unity WindZone to the XPBDCloth simulation.
/// The aerodynamic force is computed once per frame and reused across all
/// solver substeps (the wind barely changes within a single 1/24 s step).
/// </summary>
[RequireComponent(typeof(XPBDCloth))]
public class XPBDWindInteraction : MonoBehaviour
{
    public WindZone windZone;
    public float windDrag = 0.05f; // How strongly this cloth reacts to wind
    private const float TangentialFraction = 0.1f;

    private XPBDCloth cloth;
    private XPBDSolver solver;
    private int[] triangles;   // 3 grid-vertex indices per triangle
    private Vector3[] vertexForces; // cached wind force
    private int lastComputedFrame = -1;
    private float clothSize = 1f;

    void Start()
    {
        cloth = GetComponent<XPBDCloth>();
        solver = cloth != null ? cloth.Solver : null;

        if (solver == null)
        {
            Debug.LogWarning("XPBDWindInteraction: XPBDCloth solver not found, disabling.", this);
            enabled = false;
            return;
        }

        if (windZone == null)
            Debug.LogWarning("XPBDWindInteraction: no WindZone found in the scene.", this);

        BuildTriangles();
        vertexForces = new Vector3[solver.numVerts];
        clothSize = Mathf.Max(0.01f, MeasureClothSize());

        solver.OnPreSubstep += ApplyWind;
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

    /// <summary>Computes the aerodynamic force for every triangle and accumulates it per vertex.</summary>
    private void ApplyWind()
    {
        if (!isActiveAndEnabled || windZone == null || triangles == null)
            return;

        if (Time.frameCount != lastComputedFrame)
        {
            ComputeWindForces();
            lastComputedFrame = Time.frameCount;
        }

        Vector3[] externalForces = solver.externalForces;
        for (int i = 0; i < externalForces.Length; i++)
            externalForces[i] += vertexForces[i];
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

            // Pressure force along the normal. vn * |vn| keeps it quadratic (v^2 drag)
            // and correctly signed no matter which face the air hits.
            float vn = Vector3.Dot(relVelocity, normal);
            Vector3 force = normal * (vn * Mathf.Abs(vn) * windDrag);

            // Tangential (skin friction) drag along the surface.
            Vector3 tangential = relVelocity - vn * normal;
            force += tangential * tangentialDrag;

            // Aerodynamic force scales with face area, so refining the mesh (more,
            // smaller faces) keeps the total force constant instead of growing with
            // vertex count. Without this the per-vertex wind acceleration scales as N^2
            // (mass per vertex falls as 1/N^2 while force per vertex stays fixed) and
            // the XPBD integrator explodes on finer grids.
            force *= area;

            // Spread the triangle's force evenly over its three vertices.
            // Pinned vertices (invMass 0) ignore it inside the solver.
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
            Debug.LogWarning("XPBDWindInteraction: only WindZoneMode.Directional is supported", this);

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