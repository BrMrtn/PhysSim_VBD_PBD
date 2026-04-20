using System;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class PBDClothSim : MonoBehaviour
{
    public int longAxisResolution = 3; //determines how many dots are placed along the long edge
    public int solverIterations = 5;
    [Range(0f, 1f)]
    public float stiffness = 1f; // 1 = fully rigid, 0 = no constraint enforcement
    [Range(0f, 10f)]
    public float damping = 0.5f;

    private int shortAxisResolution;
    private float spacing;
    private int n;
    private Vector3 gravity = new Vector3(0, -9.81f, 0);

    private MeshFilter meshFilter;

    private ClothSimMeshBuilder clothSimMeshBuilder;

    private Vector3[] positions;
    private Vector3[] velocities;
    private Vector3[] predictedPositions;
    private float[] invMasses;
    private DistanceConstraint[] constraints;

    private int[] closestParticleIndices;
    private Vector3[] vertexOffsets;

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

        clothSimMeshBuilder = new ClothSimMeshBuilder();
        clothSimMeshBuilder.longAxisResolution = longAxisResolution;
        positions = clothSimMeshBuilder.Build(meshFilter);
        shortAxisResolution = clothSimMeshBuilder.shortAxisResolution;
        spacing = clothSimMeshBuilder.spacing;
        n = positions.Length;
        velocities = new Vector3[n];
        predictedPositions = new Vector3[n];
        invMasses = new float[n];
        Array.Fill(invMasses, 1f);

        for (int i = 0; i < longAxisResolution; i++)
            invMasses[i] = 0f;
        //invMasses[0] = 0f; //invMasses[longAxisResolution] = 0f; //fix two corners

        // Convert local-space particle positions to world-space
        for (int i = 0; i < n; i++)
        {
            positions[i] = meshFilter.transform.TransformPoint(positions[i]);

            // Add small random noise to the positions to break perfect symmetries
            if (invMasses[i] > 0f)
                positions[i] += UnityEngine.Random.insideUnitSphere * 0.01f;
        }

        InitConstraints();

        // Bind each mesh vertex to the closest simulated points
        Vector3[] meshVertices = meshFilter.mesh.vertices;
        closestParticleIndices = new int[meshVertices.Length];
        vertexOffsets = new Vector3[meshVertices.Length];

        for (int i = 0; i < meshVertices.Length; i++)
        {
            Vector3 worldPos = meshFilter.transform.TransformPoint(meshVertices[i]);
            float minDistance = float.MaxValue;
            int closestIdx = -1;

            for (int j = 0; j < n; j++)
            {
                float dist = (worldPos - positions[j]).sqrMagnitude;
                if (dist < minDistance)
                {
                    minDistance = dist;
                    closestIdx = j;
                }
            }

            closestParticleIndices[i] = closestIdx;
            // Store the offset in local space relative to the unsimulated closest particle so mesh retains shape
            vertexOffsets[i] = worldPos - positions[closestIdx];
        }

    }

    void FixedUpdate()
    {
        for (int i = 0; i < n; i++)
            velocities[i] += invMasses[i] * gravity * Time.deltaTime;
        for (int i = 0; i < n; i++) // damping
            velocities[i] *= 1f - damping * Time.deltaTime;
        for (int i = 0; i < n; i++)
            predictedPositions[i] = positions[i] + velocities[i] * Time.deltaTime;
        
        float k = 1f - Mathf.Pow(1f - stiffness, 1f / solverIterations);
        
        for (int i = 0; i < solverIterations; i++)
        {
            // project constraints
            foreach (DistanceConstraint constraint in constraints)
            {
                Vector3 p1 = predictedPositions[constraint.p1Idx];
                Vector3 p2 = predictedPositions[constraint.p2Idx];
                Vector3 delta = p2 - p1;

                if (delta.magnitude < 1e-6f) continue;

                float diff = (delta.magnitude - constraint.restLength) / delta.magnitude;
                float invMass1 = invMasses[constraint.p1Idx];
                float invMass2 = invMasses[constraint.p2Idx];
                float sumInvMass = invMass1 + invMass2;

                if (sumInvMass == 0f) continue; // both particles are fixed
                if (invMass1 > 0f)
                    predictedPositions[constraint.p1Idx] += invMass1 / sumInvMass * diff * delta * k;
                if (invMass2 > 0f)
                    predictedPositions[constraint.p2Idx] -= invMass2 / sumInvMass * diff * delta * k;
            }
        }
        
        for (int i = 0; i < n; i++)
        {
            velocities[i] = (predictedPositions[i] - positions[i]) / Time.deltaTime;
            positions[i] = predictedPositions[i];
        }

        //TODO *modify velocities of colliding vertices to friction and restitution coefficients*

        // MAKE NORMAL MESH FOLLOW SIMULATION MESH
        Vector3[] newVertices = new Vector3[closestParticleIndices.Length];
        for (int i = 0; i < closestParticleIndices.Length; i++)
        {
            int particleIdx = closestParticleIndices[i];
            Vector3 worldPos = positions[particleIdx] + vertexOffsets[i];
            newVertices[i] = meshFilter.transform.InverseTransformPoint(worldPos);
        }

        meshFilter.mesh.vertices = newVertices;
        meshFilter.mesh.RecalculateNormals();
        meshFilter.mesh.RecalculateBounds();

        // Update dot positions
        Vector3[] localPositions = new Vector3[n];
        for (int i = 0; i < n; i++)
        {
            localPositions[i] = meshFilter.transform.InverseTransformPoint(positions[i]);
        }
        clothSimMeshBuilder.UpdateDotPositions(localPositions);
    }

    private void InitConstraints()
    {
        // add a distance constraint for each pair of neighbouring particles - 8-szomszédság
        var constraintList = new List<DistanceConstraint>();
        float diagonalSpacing = spacing * Mathf.Sqrt(2);

        for (int i = 0; i < shortAxisResolution; i++)
        {
            for (int j = 0; j < longAxisResolution; j++)
            {
                int cur = i * longAxisResolution + j;
                if (i > 0 && j > 0) // add constraint to top left
                    constraintList.Add(new DistanceConstraint(cur, cur - longAxisResolution - 1, diagonalSpacing));
                if (i > 0) // add constraint above
                   constraintList.Add(new DistanceConstraint(cur, cur - longAxisResolution, spacing));
                if (i > 0 && j < longAxisResolution - 1) // add constraint to top right
                    constraintList.Add(new DistanceConstraint(cur, cur - longAxisResolution + 1, diagonalSpacing));
                if (j < longAxisResolution - 1) // add constraint to the right
                    constraintList.Add(new DistanceConstraint(cur, cur + 1, spacing));
            }
        }
        constraints = constraintList.ToArray();
    }

}