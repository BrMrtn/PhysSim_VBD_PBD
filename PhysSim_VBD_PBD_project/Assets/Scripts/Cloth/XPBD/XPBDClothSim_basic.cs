using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class XPBDClothSim : MonoBehaviour
{
    public int numSubsteps = 15;
    public float stretchingCompliance = 0.0f;
    public float shearCompliance = 0.0001f;
    public float bendingCompliance = 0.01f;
    public bool handleSelfCollisions = true;
    public bool logMsPerFrame = true;
    public float selfCollisionFriction = 0.0f;
    public bool addInitNoise = false;

    private float spacing = 0.1f;
    private float thickness = 0.01f;

    private const int logEveryNFrames = 10;

    private string performanceText = "XPBD Simulation: -- ms/frame";
    private GUIStyle performanceStyle;

    private Vector3 gravity = new Vector3(0, -9.81f, 0);

    private MeshFilter meshFilter;
    private Mesh mesh;
    private Transform tr;

    private int numVerts;
    private Vector3[] positions;
    private Vector3[] velocities;
    private Vector3[] predictedPositions;
    private Vector3[] restPositions;
    private float[] invMasses;

    private int[] meshToGrid;
    private Vector3[] renderVertices;
    private SpatialHash spatialHash;

    private DistanceConstraint[] constraints;

    void Start()
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
        int numX = xCoords.Count;
        int numY = yCoords.Count;

        positions = new Vector3[numVerts];
        velocities = new Vector3[numVerts];
        predictedPositions = new Vector3[numVerts];
        restPositions = new Vector3[numVerts];
        invMasses = new float[numVerts];
        Array.Fill(invMasses, 1.0f);

        BuildSimulationGrid(xCoords, yCoords, numX, numY);
        Array.Copy(positions, restPositions, numVerts);

        SetFixedVertices(numX, numY);
        BuildMeshToGrid(localVerts, xCoords, yCoords, eps, numX);
        InitConstraints(numX, numY);

        //Add small random noise to the positions to break perfect symmetries
        if (addInitNoise)
            for (int i = 0; i < numVerts; i++)
                if (invMasses[i] > 0f)
                    positions[i] += UnityEngine.Random.insideUnitSphere * 0.01f;

        //spacing = constraints[0].restLength;
        //thickness = spacing * 0.5f;
        spatialHash = new SpatialHash(spacing, numVerts);
    }

    void Update()
    {
        bool shouldLogPerformance = logMsPerFrame && Time.frameCount % logEveryNFrames == 0;
        double simStartTime = shouldLogPerformance ? Time.realtimeSinceStartupAsDouble : 0;

        float sdt = Time.deltaTime / numSubsteps;
        float invSdt2 = 1.0f / (sdt * sdt);
        float maxVelocity = 0.2f * thickness / sdt;

        if (handleSelfCollisions)
        {
            spatialHash.Create(positions);
            float maxTravelDistance = maxVelocity * Time.deltaTime;
            spatialHash.QueryAll(positions, maxTravelDistance);
        }

        for (int step = 0; step < numSubsteps; step++)
        {
            for (int i = 0; i < numVerts; i++)
            {
                if (invMasses[i] == 0f) continue;
                velocities[i] += gravity * sdt;
                predictedPositions[i] = positions[i];
                positions[i] += velocities[i] * sdt;
            }

            // Solve constraints
            for (int i = 0; i < constraints.Length; i++)
            {
                var constraint = constraints[i];
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
                velocities[i] = (positions[i] - predictedPositions[i]) / sdt;
            }
        }

        Matrix4x4 worldToLocal = tr.worldToLocalMatrix;
        for (int i = 0; i < numVerts; i++)
            renderVertices[i] = worldToLocal.MultiplyPoint3x4(positions[meshToGrid[i]]);

        mesh.SetVertices(renderVertices, 0, numVerts, UnityEngine.Rendering.MeshUpdateFlags.DontRecalculateBounds);
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

    private void SetFixedVertices(int numX, int numY)
    {
        int topLeft = (numY - 1) * numX;
        int topRight = (numY - 1) * numX + (numX - 1);

        invMasses[topLeft] = 0f;
        //invMasses[topRight] = 0f;
    }

    private void BuildSimulationGrid(List<float> xCoords, List<float> yCoords, int numX, int numY)
    {
        for (int iy = 0; iy < numY; iy++)
            for (int ix = 0; ix < numX; ix++)
            {
                int idx = iy * numX + ix;
                Vector3 localPos = new Vector3(xCoords[ix], yCoords[iy], 0f);
                positions[idx] = tr.TransformPoint(localPos);
            }
    }

    private void BuildMeshToGrid(Vector3[] localVerts, List<float> xCoords, List<float> yCoords, float eps, int numX)
    {
        meshToGrid = new int[numVerts];
        for (int i = 0; i < numVerts; i++)
        {
            int ix = xCoords.FindIndex(x => Mathf.Abs(x - localVerts[i].x) < eps);
            int iy = yCoords.FindIndex(y => Mathf.Abs(y - localVerts[i].y) < eps);
            meshToGrid[i] = iy * numX + ix;
        }
    }

    private void InitConstraints(int numX, int numY)
    {
        var constraintsList = new List<DistanceConstraint>();

        AddConstraint(constraintsList, 0, 0, 1, 0, stretchingCompliance, numX, numY);     // stretch horizontal
        AddConstraint(constraintsList, 0, 0, 0, 1, stretchingCompliance, numX, numY);     // stretch vertical
        AddConstraint(constraintsList, 0, 0, 1, 1, shearCompliance, numX, numY);          // shear down
        AddConstraint(constraintsList, 1, 0, 0, 1, shearCompliance, numX, numY);          // shear up
        AddConstraint(constraintsList, 0, 0, 2, 0, bendingCompliance, numX, numY);        // bend horizontal
        AddConstraint(constraintsList, 0, 0, 0, 2, bendingCompliance, numX, numY);        // bend vertical

        constraints = constraintsList.ToArray();
    }

    private void AddConstraint(List<DistanceConstraint> constraintsList, int offset_i0, int offset_j0, int offset_i1, int offset_j1, float compliance, int numX, int numY)
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
                    minDist = Mathf.Sqrt(restDist2);

                float dist = Mathf.Sqrt(dist2);
                Vector3 correction = delta * ((minDist - dist) / dist);
                positions[id0] += 0.5f * correction;
                positions[id1] -= 0.5f * correction;

                if (selfCollisionFriction > 0f)
                {
                    Vector3 vel0 = positions[id0] - predictedPositions[id0];
                    Vector3 vel1 = positions[id1] - predictedPositions[id1];
                    Vector3 avgVel = (vel0 + vel1) * 0.5f;

                    Vector3 vel0Correction = (vel0 - avgVel) * selfCollisionFriction;
                    Vector3 vel1Correction = (vel1 - avgVel) * selfCollisionFriction;
                    predictedPositions[id0] += vel0Correction;
                    predictedPositions[id1] += vel1Correction;
                }
            }
        }
    }
}