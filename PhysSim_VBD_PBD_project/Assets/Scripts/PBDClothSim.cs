using System;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class PBDClothSim : MonoBehaviour
{
    public int longAxisResolution = 3; //determines how many dots are placed along the long edge
    private int shortAxisResolution;
    private float spacing;
    private int n;

    private MeshFilter meshFilter;
    private Mesh mesh;

    private Vector3[] positions;
    private Vector3[] velocities;
    private Vector3[] predictedPositions;
    private float[] invMasses;
    private DistanceConstraint[] constraints;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
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

        ClothSimMeshBuilder clothSimMeshBuilder = new ClothSimMeshBuilder();
        clothSimMeshBuilder.longAxisResolution = longAxisResolution;
        positions = clothSimMeshBuilder.Build(meshFilter);
        shortAxisResolution = clothSimMeshBuilder.shortAxisResolution;
        spacing = clothSimMeshBuilder.spacing;
        n = positions.Length;

        // Convert local-space particle positions to world-space
        for (int i = 0; i < n; i++)
            positions[i] = meshFilter.transform.TransformPoint(positions[i]);

        velocities = new Vector3[n];
        predictedPositions = new Vector3[n];
        invMasses = new float[n];
        Array.Fill(invMasses, 1f);
        invMasses[0] = 0f; invMasses[longAxisResolution] = 0f; //fix two corners

        InitConstraints();
    }

    // Update is called once per frame
    void Update()
    {

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
                if (j > 0) // add constraint to top left
                    constraintList.Add(new DistanceConstraint(cur, cur - longAxisResolution - 1, diagonalSpacing));
                if (0 < i) // add constraint above
                   constraintList.Add(new DistanceConstraint(cur, cur - longAxisResolution, spacing));
                if (0 < i && j < longAxisResolution - 1) // add constraint to top right
                    constraintList.Add(new DistanceConstraint(cur, cur - longAxisResolution + 1, diagonalSpacing));
                if (j < longAxisResolution - 1) // add constraint to the right
                    constraintList.Add(new DistanceConstraint(cur, cur + 1, spacing));
            }
        }
        constraints = constraintList.ToArray();
    }
}