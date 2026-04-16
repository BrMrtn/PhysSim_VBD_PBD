using JetBrains.Annotations;
using System;
using System.Xml.Serialization;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem.Android;
using static UnityEngine.UI.GridLayoutGroup;

public class ClothSimMeshBuilder
{
    public int longAxisResolution = 16; //determines how many dots are placed along the long edge
    public int shortAxisResolution;
    public float spacing;

    private MeshFilter meshFilter;

    private Vector3[] vertices;

    public Vector3[] Build(MeshFilter _meshFilter)
    {
        if (longAxisResolution < 2)
        {
            Debug.LogWarning("longAxisResolution has to be at least 2");
            return null;
        }
        meshFilter = _meshFilter;

        Bounds bounds = meshFilter.mesh.bounds;

        Vector3 lossyScale = meshFilter.transform.lossyScale;
        Vector3 realSize = Vector3.Scale(bounds.size, lossyScale);

        // Determine the flat, the short and the long axis of the rectangle
        float[] sizeArray = { realSize.x, realSize.y, realSize.z };
        int[] dimArray = { 0, 1, 2 };
        Array.Sort(sizeArray, dimArray); //sorts both arrays by the values of sizeArray

        int flatAxis = dimArray[0];
        int shortAxis = dimArray[1];
        int longAxis = dimArray[2];

        spacing = sizeArray[2] / (longAxisResolution - 1);
        shortAxisResolution = (int)Mathf.Ceil(sizeArray[1] / spacing) + 1;

        vertices = new Vector3[longAxisResolution * shortAxisResolution];

        // Convert world-space spacing back to local-space spacing
        float longAxisScale = lossyScale[longAxis] < Mathf.Epsilon ? 1.0f : lossyScale[longAxis];
        float shortAxisScale = lossyScale[shortAxis] < Mathf.Epsilon ? 1.0f : lossyScale[shortAxis];

        float longAxisSpacing = spacing / longAxisScale;
        float shortAxisSpacing = spacing / shortAxisScale;

        // Load vertices with the points + place mass points
        for (int i = 0; i < shortAxisResolution; i++)
        {
            for (int j = 0; j < longAxisResolution; j++)
            {
                Vector3 vertex = Vector3.zero;
                vertex[shortAxis] = bounds.min[shortAxis] + i * shortAxisSpacing;
                vertex[longAxis] = bounds.min[longAxis] + j * longAxisSpacing;
                vertices[i * longAxisResolution + j] = vertex;
            }
        }

        // Scale mesh to fit vertices
        StretchMeshGeometryOnShortAxis(shortAxis, sizeArray[1], shortAxisScale);

        // Place a small spheres at each vertex
        foreach (Vector3 vertex in vertices)
            AddDot(vertex);

        return vertices;
    }

    private void StretchMeshGeometryOnShortAxis(int shortAxis, float shortWorldSize, float shortAxisScale)
    {
        float targetShortWorldSize = spacing * (shortAxisResolution - 1);
        if (targetShortWorldSize <= shortWorldSize + 1e-6f)
            return; // no elongation needed

        float currentShortLocalSize = shortWorldSize / shortAxisScale;
        float targetShortLocalSize = targetShortWorldSize / shortAxisScale;
        float scaleFactor = targetShortLocalSize / currentShortLocalSize;

        Vector3[] meshVertices = meshFilter.mesh.vertices;

        // Keep the min edge fixed, stretch toward +axis
        float min = meshFilter.mesh.bounds.min[shortAxis];
        for (int i = 0; i < meshVertices.Length; i++)
        {
            Vector3 v = meshVertices[i];
            v[shortAxis] = min + (v[shortAxis] - min) * scaleFactor;
            meshVertices[i] = v;
        }

        meshFilter.mesh.vertices = meshVertices;
        meshFilter.mesh.RecalculateBounds();
        meshFilter.mesh.RecalculateNormals();

        meshFilter.mesh.name = "PBD_ModifiedMesh";
    }

    private void AddDot(Vector3 pos)
    {
        GameObject dot = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        dot.name = "Dot";
        dot.transform.SetParent(meshFilter.transform, false);
        dot.transform.localPosition = pos;

        Vector3 parentScale = meshFilter.transform.lossyScale;
        dot.transform.localScale = new Vector3(
            0.02f / parentScale.x,
            0.02f / parentScale.y,
            0.02f / parentScale.z
        );

        // Optional: destroy the collider so it doesn't interfere with physics
        Collider col = dot.GetComponent<Collider>();
        if (col != null)
        {
            GameObject.Destroy(col);
        }
    }
}
