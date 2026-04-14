using UnityEngine;

public class PBDClothSim : MonoBehaviour
{
    public int longAxisResolution = 3; //determines how many dots are placed along the long edge
    private float spacing;

    private MeshFilter meshFilter;
    private Mesh mesh;

    Vector3[] simVertices;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        meshFilter = gameObject.GetComponent<MeshFilter>();
        if (meshFilter == null || meshFilter.mesh == null)
        {
            Debug.LogWarning("No MeshFilter or Mesh found!");
            return;
        }

        ClothSimMeshBuilder initSimulationMesh = new ClothSimMeshBuilder();
        initSimulationMesh.longAxisResolution = longAxisResolution;
        simVertices = initSimulationMesh.Build(meshFilter);

        InitConstraints();
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void InitConstraints()
    {

    }
