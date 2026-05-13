using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Rendering;

public class XPBDCloth : MonoBehaviour
{
    public int numSubsteps = 15;

    public float stretchingCompliance = 1e-6f;
    public float shearCompliance = 0.0001f;
    public float bendingCompliance = 0.001f;

    public bool handleSelfCollisions = true;
    public float selfCollisionFriction = 0.0f;
    public bool logMsPerFrame = true;
    public bool addInitNoise = false;
    public event Action OnUpdate;

    private float spacing;
    private float thickness;

    private const int logEveryNFrames = 10;
    private string performanceText = "XPBD Simulation: -- ms/frame";
    private GUIStyle performanceStyle;

    private Vector3 gravity = new Vector3(0, -9.81f, 0);

    private MeshFilter meshFilter;
    private Mesh mesh;
    private Transform tr;

    [HideInInspector] public int numVerts;
    [HideInInspector] public int numX;
    [HideInInspector] public int numY;
    [HideInInspector] public Vector3[] positions;
    [HideInInspector] public Vector3[] velocities;
    [HideInInspector] public Vector3[] previousPosition;
    [HideInInspector] public Vector3[] restPositions;
    [HideInInspector] public float[] invMasses;

    private int[] meshToGrid;
    private Vector3[] renderVertices;
    private SpatialHash spatialHash;

    [HideInInspector] public DistanceConstraint[] constraints;
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

        positions = new Vector3[numVerts];
        velocities = new Vector3[numVerts];
        previousPosition = new Vector3[numVerts];
        restPositions = new Vector3[numVerts];
        invMasses = new float[numVerts];
        Array.Fill(invMasses, 1f);

        BuildSimulationGrid(xCoords, yCoords);
        Array.Copy(positions, restPositions, numVerts);

        BuildMeshToGrid(localVerts, xCoords, yCoords, eps);
        InitConstraints();

        //Add small random noise to the positions to break perfect symmetries
        if (addInitNoise)
            for (int i = 0; i < numVerts; i++)
                if (invMasses[i] > 0f)
                    positions[i] += UnityEngine.Random.insideUnitSphere * 0.001f;

        spatialHash = new SpatialHash(spacing, numVerts);
    }

    void Update()
    {
        bool shouldLogPerformance = logMsPerFrame && Time.frameCount % logEveryNFrames == 0;
        double simStartTime = shouldLogPerformance ? Time.realtimeSinceStartupAsDouble : 0;

        float dt = 1 / 24f;
        float sdt = dt / numSubsteps;
        float invSdt2 = 1.0f / (sdt * sdt);
        float maxVelocity = 0.2f * thickness / sdt;

        if (handleSelfCollisions)
        {
            spatialHash.Create(positions);
            float maxTravelDistance = maxVelocity * dt;
            spatialHash.QueryAll(positions, maxTravelDistance);
        }

        for (int step = 0; step < numSubsteps; step++)
        {
            for (int i = 0; i < numVerts; i++)
            {
                if (invMasses[i] == 0f) continue;
                velocities[i] += gravity * sdt;

                float v = velocities[i].magnitude;
                if (v > maxVelocity)
                    velocities[i] *= maxVelocity / v;

                previousPosition[i] = positions[i];
                positions[i] += velocities[i] * sdt;
            }

            // Solve constraints
            for (int i = 0; i < constraints.Length; i++)
            {
                ref var constraint = ref constraints[i];
                int id0 = constraint.p1Idx;
                int id1 = constraint.p2Idx;
                float w0 = invMasses[id0];
                float w1 = invMasses[id1];
                float w = w0 + w1;
                if (w == 0f) continue;

                Vector3 grad = positions[id0] - positions[id1];
                float len = grad.magnitude;
                if (len == 0f) continue;

                grad /= len;
                float C = len - constraint.restLength;
                float alpha = constraint.compliance * invSdt2;
                float s = -C / (w + alpha);

                positions[id0] += grad * (s * w0);
                positions[id1] -= grad * (s * w1);
            }

            if (handleSelfCollisions) SolveSelfCollisions();

            for (int i = 0; i < numVerts; i++)
            {
                if (invMasses[i] == 0f) continue;
                velocities[i] = (positions[i] - previousPosition[i]) / sdt;
            }

            OnUpdate?.Invoke();
        }

        Matrix4x4 worldToLocal = tr.worldToLocalMatrix;
        for (int i = 0; i < numVerts; i++)
            renderVertices[i] = worldToLocal.MultiplyPoint3x4(positions[meshToGrid[i]]);

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
                positions[idx] = tr.TransformPoint(localPos);
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

        constraints = constraintsList.ToArray();
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
                        restLength = Vector3.Distance(positions[p1], positions[p2]),
                        compliance = compliance
                    });
                }
            }
    }

    private void SolveSelfCollisions()
    {
        float thickness2 = thickness * thickness;

        for (int id0 = 0; id0 < numVerts; id0++)
        {
            if (invMasses[id0] == 0f)
                continue;
            int first = spatialHash.firstAdjId[id0];
            int last = spatialHash.firstAdjId[id0 + 1];

            for (int j = first; j < last; j++)
            {
                int id1 = spatialHash.adjIds[j];
                if (invMasses[id1] == 0f)
                    continue;

                Vector3 delta = positions[id0] - positions[id1];
                float dist2 = delta.sqrMagnitude;
                if (dist2 == 0f || dist2 > thickness2)
                    continue;

                float restDist2 = (restPositions[id0] - restPositions[id1]).sqrMagnitude;
                if (dist2 > restDist2)
                    continue;

                float minDist = thickness;
                if (restDist2 < thickness2)
                    minDist = Mathf.Sqrt(restDist2); // minDist is min(thickness, restDist), so collision detection doesn't clash with constraints

                float dist = Mathf.Sqrt(dist2);
                Vector3 correction = delta * ((minDist - dist) / dist);
                positions[id0] += 0.5f * correction;
                positions[id1] -= 0.5f * correction;

                Vector3 v0 = positions[id0] - previousPosition[id0];
                Vector3 v1 = positions[id1] - previousPosition[id1];
                Vector3 vAvg = 0.5f * (v0 + v1);

                Vector3 v0Corr = vAvg - v0;
                Vector3 v1Corr = vAvg - v1;

                positions[id0] += v0Corr * selfCollisionFriction;
                positions[id1] += v1Corr * selfCollisionFriction;
            }
        }
    }
}