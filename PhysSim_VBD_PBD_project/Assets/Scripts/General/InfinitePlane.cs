using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class InfinitePlane : MonoBehaviour
{
    void Awake()
    {
        float s = 10000f;
        var mesh = new Mesh();
        mesh.vertices = new Vector3[] {
            new(-s, 0, -s), new(-s, 0, s),
            new(s,  0, s),  new(s,  0, -s)
        };
        mesh.triangles = new int[] { 0, 1, 2, 0, 2, 3 };
        mesh.RecalculateNormals();
        GetComponent<MeshFilter>().mesh = mesh;
    }
}