using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Cursor_VBD_ClothSim : MonoBehaviour
{
    public int numSubsteps = 15;
    public int numIterations = 15;
    public float stretchingCompliance = 1e-6f;
    public float shearCompliance = 0.0001f;
    public float bendingCompliance = 0.001f;
    public bool logMsPerFrame = true;
    public bool addInitNoise = false;
    public event Action OnUpdate;

    private const int LogEveryNFrames = 10;
    private const float MinCompliance = 1e-12f;

    private float spacing;
    private float thickness;

    private string performanceText = "VBD Simulation: -- ms/frame";
    private GUIStyle performanceStyle;

    private readonly Vector3 gravity = new Vector3(0f, -9.81f, 0f);

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

    private float[] masses;
    private int[] meshToGrid;
    private Vector3[] renderVertices;
    private Vector3[] inertialTargets;
    private List<int>[] vertexAdjacency;

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
        masses = new float[numVerts];
        inertialTargets = new Vector3[numVerts];

        Array.Fill(invMasses, 1f);
        Array.Fill(masses, 1f);

        int topLeftIdx = (numY - 1) * numX;
        int topRightIdx = (numY - 1) * numX + (numX - 1);

        invMasses[topLeftIdx] = 0f;
        invMasses[topRightIdx] = 0f;

        BuildSimulationGrid(xCoords, yCoords);
        Array.Copy(positions, restPositions, numVerts);

        BuildMeshToGrid(localVerts, xCoords, yCoords, eps);
        InitConstraints();
        BuildVertexAdjacency();
        SyncMassFromInverseMass();

        if (addInitNoise)
        {
            for (int i = 0; i < numVerts; i++)
            {
                if (invMasses[i] > 0f)
                    positions[i] += UnityEngine.Random.insideUnitSphere * 0.001f;
            }
        }
    }

    void Update()
    {
        bool shouldLogPerformance = logMsPerFrame && Time.frameCount % LogEveryNFrames == 0;
        double simStartTime = shouldLogPerformance ? Time.realtimeSinceStartupAsDouble : 0d;

        SyncMassFromInverseMass();

        float dt = 1f / 24f;
        float sdt = dt / numSubsteps;
        float invSdt2 = 1f / (sdt * sdt);
        float maxVelocity = 0.2f * Mathf.Max(thickness, 1e-4f) / sdt;

        for (int step = 0; step < numSubsteps; step++)
        {
            for (int i = 0; i < numVerts; i++)
            {
                if (invMasses[i] <= 0f)
                {
                    inertialTargets[i] = positions[i];
                    continue;
                }

                velocities[i] += gravity * sdt;

                float speed = velocities[i].magnitude;
                if (speed > maxVelocity)
                    velocities[i] *= maxVelocity / speed;

                previousPosition[i] = positions[i];
                inertialTargets[i] = positions[i] + velocities[i] * sdt;
                positions[i] = inertialTargets[i];
            }

            for (int iter = 0; iter < numIterations; iter++)
            {
                for (int i = 0; i < numVerts; i++)
                {
                    if (invMasses[i] <= 0f)
                        continue;

                    SolveVertexBlock(i, invSdt2);
                }
            }

            OnUpdate?.Invoke();

            for (int i = 0; i < numVerts; i++)
            {
                if (invMasses[i] <= 0f)
                    continue;

                velocities[i] = (positions[i] - previousPosition[i]) / sdt;
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
            performanceText = $"VBD Simulation: {msPerFrame:F2} ms/frame";
        }
    }

    private void SolveVertexBlock(int vertexId, float invSdt2)
    {
        Matrix4x4 hessian4 = Matrix4x4.zero;
        Vector3 rhs = Vector3.zero;

        float mass = masses[vertexId];
        hessian4.m00 = mass * invSdt2;
        hessian4.m11 = mass * invSdt2;
        hessian4.m22 = mass * invSdt2;
        rhs += mass * invSdt2 * (inertialTargets[vertexId] - positions[vertexId]);

        List<int> adjacent = vertexAdjacency[vertexId];
        for (int j = 0; j < adjacent.Count; j++)
        {
            DistanceConstraint c = constraints[adjacent[j]];

            int other = c.p1Idx == vertexId ? c.p2Idx : c.p1Idx;
            Vector3 diff = positions[vertexId] - positions[other];
            float len = diff.magnitude;
            if (len < 1e-8f)
                continue;

            float stiffness = ComplianceToStiffness(c.compliance);
            float invLen = 1f / len;
            Vector3 dir = diff * invLen;
            float l0 = c.restLength;

            rhs += stiffness * ((l0 - len) * invLen) * diff;

            float scale = stiffness * l0 * invLen;
            Matrix4x4 outer = Outer(dir);
            Matrix4x4 springHessian = Sub(
                Scale(Matrix4x4.identity, stiffness),
                Scale(Sub(Matrix4x4.identity, outer), scale)
            );
            hessian4 = Add(hessian4, springHessian);
        }

        Vector3 dx = SolveSymmetric3x3(hessian4, rhs);
        positions[vertexId] += dx;
    }

    private static Matrix4x4 Outer(Vector3 v)
    {
        Matrix4x4 m = Matrix4x4.zero;
        m.m00 = v.x * v.x;
        m.m01 = v.x * v.y;
        m.m02 = v.x * v.z;
        m.m10 = v.y * v.x;
        m.m11 = v.y * v.y;
        m.m12 = v.y * v.z;
        m.m20 = v.z * v.x;
        m.m21 = v.z * v.y;
        m.m22 = v.z * v.z;
        return m;
    }

    private static Matrix4x4 Add(Matrix4x4 a, Matrix4x4 b)
    {
        Matrix4x4 m = Matrix4x4.zero;
        m.m00 = a.m00 + b.m00; m.m01 = a.m01 + b.m01; m.m02 = a.m02 + b.m02; m.m03 = a.m03 + b.m03;
        m.m10 = a.m10 + b.m10; m.m11 = a.m11 + b.m11; m.m12 = a.m12 + b.m12; m.m13 = a.m13 + b.m13;
        m.m20 = a.m20 + b.m20; m.m21 = a.m21 + b.m21; m.m22 = a.m22 + b.m22; m.m23 = a.m23 + b.m23;
        m.m30 = a.m30 + b.m30; m.m31 = a.m31 + b.m31; m.m32 = a.m32 + b.m32; m.m33 = a.m33 + b.m33;
        return m;
    }

    private static Matrix4x4 Sub(Matrix4x4 a, Matrix4x4 b)
    {
        Matrix4x4 m = Matrix4x4.zero;
        m.m00 = a.m00 - b.m00; m.m01 = a.m01 - b.m01; m.m02 = a.m02 - b.m02; m.m03 = a.m03 - b.m03;
        m.m10 = a.m10 - b.m10; m.m11 = a.m11 - b.m11; m.m12 = a.m12 - b.m12; m.m13 = a.m13 - b.m13;
        m.m20 = a.m20 - b.m20; m.m21 = a.m21 - b.m21; m.m22 = a.m22 - b.m22; m.m23 = a.m23 - b.m23;
        m.m30 = a.m30 - b.m30; m.m31 = a.m31 - b.m31; m.m32 = a.m32 - b.m32; m.m33 = a.m33 - b.m33;
        return m;
    }

    private static Matrix4x4 Scale(Matrix4x4 a, float s)
    {
        Matrix4x4 m = Matrix4x4.zero;
        m.m00 = a.m00 * s; m.m01 = a.m01 * s; m.m02 = a.m02 * s; m.m03 = a.m03 * s;
        m.m10 = a.m10 * s; m.m11 = a.m11 * s; m.m12 = a.m12 * s; m.m13 = a.m13 * s;
        m.m20 = a.m20 * s; m.m21 = a.m21 * s; m.m22 = a.m22 * s; m.m23 = a.m23 * s;
        m.m30 = a.m30 * s; m.m31 = a.m31 * s; m.m32 = a.m32 * s; m.m33 = a.m33 * s;
        return m;
    }

    private static Vector3 SolveSymmetric3x3(Matrix4x4 a, Vector3 b)
    {
        float det =
            a.m00 * (a.m11 * a.m22 - a.m12 * a.m21) -
            a.m01 * (a.m10 * a.m22 - a.m12 * a.m20) +
            a.m02 * (a.m10 * a.m21 - a.m11 * a.m20);

        if (Mathf.Abs(det) < 1e-10f)
            return Vector3.zero;

        float invDet = 1f / det;
        Matrix4x4 inv = Matrix4x4.zero;

        inv.m00 = (a.m11 * a.m22 - a.m12 * a.m21) * invDet;
        inv.m01 = (a.m02 * a.m21 - a.m01 * a.m22) * invDet;
        inv.m02 = (a.m01 * a.m12 - a.m02 * a.m11) * invDet;
        inv.m10 = (a.m12 * a.m20 - a.m10 * a.m22) * invDet;
        inv.m11 = (a.m00 * a.m22 - a.m02 * a.m20) * invDet;
        inv.m12 = (a.m02 * a.m10 - a.m00 * a.m12) * invDet;
        inv.m20 = (a.m10 * a.m21 - a.m11 * a.m20) * invDet;
        inv.m21 = (a.m01 * a.m20 - a.m00 * a.m21) * invDet;
        inv.m22 = (a.m00 * a.m11 - a.m01 * a.m10) * invDet;

        return new Vector3(
            inv.m00 * b.x + inv.m01 * b.y + inv.m02 * b.z,
            inv.m10 * b.x + inv.m11 * b.y + inv.m12 * b.z,
            inv.m20 * b.x + inv.m21 * b.y + inv.m22 * b.z
        );
    }

    private float ComplianceToStiffness(float compliance)
    {
        float c = Mathf.Max(compliance, MinCompliance);
        return 1f / c;
    }

    private void SyncMassFromInverseMass()
    {
        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] <= 0f)
                masses[i] = 0f;
            else
                masses[i] = 1f / invMasses[i];
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
        {
            for (int ix = 0; ix < numX; ix++)
            {
                int idx = iy * numX + ix;
                Vector3 localPos = new Vector3(xCoords[ix], yCoords[iy], 0f);
                positions[idx] = tr.TransformPoint(localPos);
            }
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

        AddConstraint(constraintsList, 0, 0, 1, 0, stretchingCompliance); // stretch horizontal
        AddConstraint(constraintsList, 0, 0, 0, 1, stretchingCompliance); // stretch vertical
        AddConstraint(constraintsList, 0, 0, 1, 1, shearCompliance);      // shear down
        AddConstraint(constraintsList, 1, 0, 0, 1, shearCompliance);      // shear up
        AddConstraint(constraintsList, 0, 0, 2, 0, bendingCompliance);    // bend horizontal
        AddConstraint(constraintsList, 0, 0, 0, 2, bendingCompliance);    // bend vertical

        constraints = constraintsList.ToArray();
    }

    private void BuildVertexAdjacency()
    {
        vertexAdjacency = new List<int>[numVerts];
        for (int i = 0; i < numVerts; i++)
            vertexAdjacency[i] = new List<int>();

        for (int i = 0; i < constraints.Length; i++)
        {
            vertexAdjacency[constraints[i].p1Idx].Add(i);
            vertexAdjacency[constraints[i].p2Idx].Add(i);
        }
    }

    private void AddConstraint(List<DistanceConstraint> constraintsList, int offset_i0, int offset_j0, int offset_i1, int offset_j1, float compliance)
    {
        for (int iy = 0; iy < numY; iy++)
        {
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
    }
}