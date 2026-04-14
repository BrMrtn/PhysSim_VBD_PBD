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
    private float spacing;

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

        // Determine the flat, the short and the long axis of the rectangle
        float[] sizeArray = { bounds.size.x, bounds.size.y, bounds.size.z };
        int[] dimArray = { 0, 1, 2 };
        Array.Sort(sizeArray, dimArray); //sorts both arrays by the values of sizeArray

        int flatAxis = dimArray[0];
        int shortAxis = dimArray[1];
        int longAxis = dimArray[2];

        spacing = sizeArray[2] / (longAxisResolution - 1);
        int shortAxisResolution = (int)Mathf.Ceil(sizeArray[1] / spacing) + 1;

        vertices = new Vector3[longAxisResolution * shortAxisResolution];

        //load vertices with the points + place mass points
        for (int i = 0; i < longAxisResolution; i++)
        {
            for (int j = 0; j < shortAxisResolution; j++)
            {
                Vector3 vertex = Vector3.zero;
                vertex[longAxis] = bounds.min[longAxis] + i * spacing;
                vertex[shortAxis] = bounds.min[shortAxis] + j * spacing;
                vertices[i * shortAxisResolution + j] = vertex;
            }
        }

        // Place a small spheres at each vertex
        foreach (Vector3 vertex in vertices)
            AddDot(vertex);

        return vertices;
    }

    void AddDot(Vector3 pos)
    {
        GameObject dot = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        dot.name = "Dot";
        dot.transform.SetParent(meshFilter.transform, false);
        dot.transform.localPosition = pos;
        dot.transform.localScale = Vector3.one * 0.1f;

        // Optional: destroy the collider so it doesn't interfere with physics
        Collider col = dot.GetComponent<Collider>();
        if (col != null)
        {
            GameObject.Destroy(col);
        }
    }
}
