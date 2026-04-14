using System;
using System.Collections.Generic;
using UnityEngine;
using static UnityEditor.Searcher.SearcherWindow.Alignment;

//[ExecuteAlways]
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class ClothSimulation : MonoBehaviour
{
    public int gridResolution = 30;
    public float spacing = 0.2f;
    public int solverIterations = 5;
    public Vector3 gravity = new Vector3(0, -9.81f, 0);
    public bool showMassPoints = true;
    public float massPointSize = 0.05f;

    private ClothParticle[,] particles;
    private List<DistanceConstraint> constraints = new List<DistanceConstraint>();

    private Mesh mesh;
    private int width, height;
    private Transform[] massPointVisuals;
    private Transform massPointRoot;

    private Vector3[] vertices;
    private Vector2[] uv;
    private int[] triangles;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        InitializeCloth();
        InitializeMesh();
        CreateMassPointVisuals();
        UpdateMassPointVisuals();
    }

    // Update is called once per frame
    void Update()
    {
        //Simulate(Time.deltaTime);
        //UpdateMesh();
        UpdateMassPointVisuals();
    }

    void InitializeCloth()
    {
        constraints.Clear();

        MeshFilter meshFilter = GetComponent<MeshFilter>();
        Mesh sourceMesh = meshFilter != null ? meshFilter.sharedMesh : null;

        Bounds sourceBounds;
        if (sourceMesh != null && sourceMesh.vertexCount > 0)
        {
            sourceBounds = sourceMesh.bounds;
        }
        else
        {
            float fallbackSize = (Mathf.Max(2, gridResolution) - 1) * spacing;
            sourceBounds = new Bounds(Vector3.zero, new Vector3(fallbackSize, fallbackSize, 0f));
        }

        float sourceWidth = Mathf.Abs(sourceBounds.size.x);
        float sourceHeight = Mathf.Abs(sourceBounds.size.y);

        bool longSideIsWidth = sourceWidth >= sourceHeight;
        float longerSide = longSideIsWidth ? sourceWidth : sourceHeight;
        float shorterSide = longSideIsWidth ? sourceHeight : sourceWidth;

        int longCount = Mathf.Max(2, gridResolution);
        spacing = longerSide / (longCount - 1);

        int shortCount = Mathf.Max(2, Mathf.RoundToInt(shorterSide / spacing) + 1);

        width = longSideIsWidth ? longCount : shortCount;
        height = longSideIsWidth ? shortCount : longCount;

        particles = new ClothParticle[width, height];

        float fittedWidth = (width - 1) * spacing;
        float fittedHeight = (height - 1) * spacing;
        Vector3 center = sourceBounds.center;

        float startX = center.x - (fittedWidth * 0.5f);
        float startY = center.y + (fittedHeight * 0.5f);
        float z = center.z;

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                Vector3 localPos = new Vector3(startX + (x * spacing), startY - (y * spacing), z);
                Vector3 pos = transform.TransformPoint(localPos);

                float invMass = (y == 0 && (x % 5 == 0)) ? 0 : 1; // pin top row sparsely
                particles[x, y] = new ClothParticle(pos, invMass);
            }
        }

        // Structural constraints
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                if (x < width - 1)
                    constraints.Add(new DistanceConstraint(particles[x, y], particles[x + 1, y], spacing));

                if (y < height - 1)
                    constraints.Add(new DistanceConstraint(particles[x, y], particles[x, y + 1], spacing));
            }
        }
    }

    void InitializeMesh()
    {
        mesh = new Mesh();
        mesh.name = "ClothMesh";
        GetComponent<MeshFilter>().mesh = mesh;

        vertices = new Vector3[width * height];
        triangles = new int[(width - 1) * (height - 1) * 6];
        uv = new Vector2[width * height];

        // initialize verticies
        int index = 0;
        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++)
                vertices[index++] = transform.InverseTransformPoint(particles[x, y].position);

        int t = 0;
        for (int y = 0; y < height - 1; y++)
            for (int x = 0; x < width - 1; x++)
            {
                int i = y * width + x;

                triangles[t++] = i;
                triangles[t++] = i + width;
                triangles[t++] = i + 1;

                triangles[t++] = i + 1;
                triangles[t++] = i + width;
                triangles[t++] = i + width + 1;
            }

        // UVs
        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++)
                uv[y * width + x] = new Vector2((float)x / (width - 1), (float)y / (height - 1));

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.uv = uv;
        mesh.RecalculateNormals();
    }

    void CreateMassPointVisuals()
    {
        ClearMassPointVisuals();

        if (!showMassPoints || particles == null)
            return;

        massPointRoot = new GameObject("MassPoints").transform;
        massPointRoot.SetParent(transform, false);

        massPointVisuals = new Transform[width * height];
        int index = 0;

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                GameObject point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                point.name = $"MassPoint_{x}_{y}";
                point.transform.SetParent(massPointRoot, true);
                point.transform.localScale = Vector3.one * massPointSize;

                Collider pointCollider = point.GetComponent<Collider>();
                if (pointCollider != null)
                    Destroy(pointCollider);

                massPointVisuals[index++] = point.transform;
            }
        }
    }

    void UpdateMassPointVisuals()
    {
        if (!showMassPoints)
        {
            ClearMassPointVisuals();
            return;
        }

        if (particles == null || massPointVisuals == null || massPointVisuals.Length != width * height)
            return;

        int index = 0;
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                Transform visual = massPointVisuals[index++];
                if (visual != null)
                {
                    visual.position = particles[x, y].position;
                    visual.localScale = Vector3.one * massPointSize;
                }
            }
        }
    }

    void ClearMassPointVisuals()
    {
        if (massPointRoot != null)
            Destroy(massPointRoot.gameObject);

        massPointRoot = null;
        massPointVisuals = null;
    }

    void Simulate(float dt)
    {
        // Apply forces
        foreach (var p in particles)
            p.ApplyForce(gravity, dt);

        // Solve constraints
        for (int i = 0; i < solverIterations; i++)
        {
            foreach (var c in constraints)
                c.Solve();
        }
    }

    void UpdateMesh()
    {
        int index = 0;

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                vertices[index++] = transform.InverseTransformPoint(particles[x, y].position);
            }
        }

        mesh.vertices = vertices;
        mesh.RecalculateNormals();
    }
}