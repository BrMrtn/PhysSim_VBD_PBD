using System;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem.HID;

public class PBDClothSim : MonoBehaviour
{
    public int numSubsteps = 15;
    public float density = 0.2f; // kg/m^2
    public float stretchingCompliance = 0.0f;
    public float bendingCompliance = 1.0f;

    private int n;
    private Vector3 gravity = new Vector3(0, -9.81f, 0);

    private MeshFilter meshFilter;

    private Vector3[] positions;
    private Vector3[] velocities;
    private Vector3[] predictedPositions;
    private float[] invMasses;
    private DistanceConstraint[] stretchingConstraints;
    private BendingConstraint[] bendingConstraints;

    void Start()
    {
        meshFilter = gameObject.GetComponent<MeshFilter>();
        if (meshFilter == null || meshFilter.mesh == null)
        {
            Debug.LogWarning("No MeshFilter or Mesh found!");
            return;
        }

        // Delete MeshCollider if exists - it would clash with our own collision handling
        MeshCollider meshCollider = gameObject.GetComponent<MeshCollider>();
        if (meshCollider != null)
        {
            if (Application.isPlaying) Destroy(meshCollider);
            else DestroyImmediate(meshCollider);
        }

        positions = meshFilter.mesh.vertices;
        n = positions.Length;
        velocities = new Vector3[n];
        predictedPositions = new Vector3[n];

        invMasses = new float[n];
        
        int[] tris = meshFilter.mesh.triangles;
        for (int i = 0; i < tris.Length; i += 3)
        {
            int v1 = tris[i];
            int v2 = tris[i + 1];
            int v3 = tris[i + 2];

            Vector3 p1 = positions[v1];
            Vector3 p2 = positions[v2];
            Vector3 p3 = positions[v3];

            float area = Vector3.Cross(p2 - p1, p3 - p1).magnitude * 0.5f;
            float triMass = area * density;
            float pInvMass = triMass > 0.0f ? 1.0f / triMass / 3.0f : 0.0f;

            invMasses[v1] += pInvMass;
            invMasses[v2] += pInvMass;
            invMasses[v3] += pInvMass;
        }

        // Convert local-space particle positions to world-space
        for (int i = 0; i < n; i++)
            positions[i] = meshFilter.transform.TransformPoint(positions[i]);

        float minX = float.MaxValue;
        float maxX = float.MinValue;
        float maxY = float.MinValue;

        for (int i = 0; i < n; i++)
        {
            if (positions[i].x < minX) minX = positions[i].x;
            if (positions[i].x > maxX) maxX = positions[i].x;
            if (positions[i].y > maxY) maxY = positions[i].y;
        }

        float eps = 0.0001f;
        for (int i = 0; i < n; i++)
        {
            float x = positions[i].x;
            float y = positions[i].y;
            if ((y > maxY - eps) && (x < minX + eps || x > maxX - eps))
                invMasses[i] = 0f;
        }

        InitConstraints();

        // Add small random noise to the positions to break perfect symmetries
        for (int i = 0; i < n; i++)
            if (invMasses[i] > 0f)
                positions[i] += UnityEngine.Random.insideUnitSphere * 0.01f;

    }

    void FixedUpdate()
    {
        float sdt = Time.fixedDeltaTime / numSubsteps;

        for (int step = 0; step < numSubsteps; step++)
        {
            // Pre-solve
            for (int i = 0; i < n; i++)
            {
                velocities[i] += gravity * sdt * invMasses[i];
                predictedPositions[i] = positions[i];
                positions[i] += velocities[i] * sdt;
            }

            // Solve stretching
            float alphaStretch = stretchingCompliance / (sdt * sdt);
            foreach (DistanceConstraint constraint in stretchingConstraints)
            {
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
                float s = -C / (w + alphaStretch);

                positions[id0] += grad * (s * w0);
                positions[id1] -= grad * (s * w1);
            }

            // Solve bending
            float alphaBend = bendingCompliance / (sdt * sdt);
            foreach (BendingConstraint constraint in bendingConstraints)
            {
                int id0 = constraint.p3Idx; // Opposite vertex 1
                int id1 = constraint.p4Idx; // Opposite vertex 2
                float w0 = invMasses[id0];
                float w1 = invMasses[id1];
                float w = w0 + w1;
                if (w == 0f) continue;

                Vector3 grad = positions[id0] - positions[id1];
                float len = grad.magnitude;
                if (len == 0f) continue;

                grad /= len;
                float C = len - constraint.restLength;
                float s = -C / (w + alphaBend);

                positions[id0] += grad * (s * w0);
                positions[id1] -= grad * (s * w1);
            }

            // Post-solve
            for (int i = 0; i < n; i++)
            {
                if (invMasses[i] == 0f) continue;
                velocities[i] = (positions[i] - predictedPositions[i]) / sdt;
            }
        }

        // update mesh vertices
        Vector3[] newVertices = new Vector3[n];
        for (int i = 0; i < n; i++)
            newVertices[i] = meshFilter.transform.InverseTransformPoint(positions[i]);

        meshFilter.mesh.vertices = newVertices;
        meshFilter.mesh.RecalculateNormals();
        meshFilter.mesh.RecalculateBounds();
    }

    private void InitConstraints()
    {
        var stretchingList = new List<DistanceConstraint>();
        var bendingList = new List<BendingConstraint>();

        // Dictionary to track edges and find adjacent triangles
        // Key: edge identifier, Value: the third vertex of the triangle
        var edges = new Dictionary<long, int>();

        int[] triangles = meshFilter.mesh.triangles;

        for (int i = 0; i < triangles.Length; i += 3)
        {
            int v1 = triangles[i];
            int v2 = triangles[i + 1];
            int v3 = triangles[i + 2];

            AddEdgeAndBending(v1, v2, v3, stretchingList, bendingList, edges);
            AddEdgeAndBending(v2, v3, v1, stretchingList, bendingList, edges);
            AddEdgeAndBending(v3, v1, v2, stretchingList, bendingList, edges);
        }

        stretchingConstraints = stretchingList.ToArray();
        bendingConstraints = bendingList.ToArray();
    }

    private void AddEdgeAndBending(int v1, int v2, int oppositeVertex, List<DistanceConstraint> stretchingList, List<BendingConstraint> bendingList, Dictionary<long, int> edges)
    {
        int min = Mathf.Min(v1, v2);
        int max = Mathf.Max(v1, v2);
        long edgeKey = ((long)min << 32) | (uint)max;

        if (edges.TryGetValue(edgeKey, out int otherOppositeVertex))
        {
            // Edge already exists, so we found an adjacent triangle!
            // Create a bending constraint between the two opposite vertices (XPBD style distance constraint)
            float restLen = Vector3.Distance(positions[oppositeVertex], positions[otherOppositeVertex]);
            bendingList.Add(new BendingConstraint(v1, v2, oppositeVertex, otherOppositeVertex, restLen));
        }
        else
        {
            // First time seeing this edge, add a stretching constraint
            edges.Add(edgeKey, oppositeVertex);
            float stretchDist = Vector3.Distance(positions[min], positions[max]);
            stretchingList.Add(new DistanceConstraint(min, max, stretchDist));
        }
    }
}