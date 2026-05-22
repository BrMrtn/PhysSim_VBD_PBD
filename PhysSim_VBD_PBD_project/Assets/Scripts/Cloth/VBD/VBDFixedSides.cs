using UnityEngine;

[RequireComponent(typeof(VBDCloth))]
public class VBDFixedSides : MonoBehaviour
{
    public Transform topCuboid;
    public Transform bottomCuboid;

    private VBDCloth cloth;
    private VBDSolver solver;

    private int[] topIndices;
    private int[] bottomIndices;

    // Initial local-space offset of each anchored vertex relative to its cuboid.
    private Vector3[] topLocalOffsets;
    private Vector3[] bottomLocalOffsets;

    void Start()
    {
        cloth = GetComponent<VBDCloth>();
        if (cloth == null)
        {
            Debug.LogError($"{nameof(VBDFixedSides)} requires a {nameof(VBDCloth)} component on the same GameObject.");
            enabled = false;
            return;
        }

        solver = cloth.Solver;
        int numX = cloth.numX;
        int numY = cloth.numY;

        // --- Top row: iy = numY - 1 -------------------------------------------
        // Only pin this side when a cuboid transform is assigned.
        if (topCuboid != null)
        {
            topIndices = new int[numX];
            topLocalOffsets = new Vector3[numX];
            int topRowStart = (numY - 1) * numX;
            for (int ix = 0; ix < numX; ix++)
            {
                int idx = topRowStart + ix;
                topIndices[ix] = idx;
                solver.invMasses[idx] = 0f;
                topLocalOffsets[ix] = topCuboid.InverseTransformPoint(solver.positions[idx]);
            }
        }

        // --- Bottom row: iy = 0 -----------------------------------------------
        // Only pin this side when a cuboid transform is assigned.
        if (bottomCuboid != null)
        {
            bottomIndices = new int[numX];
            bottomLocalOffsets = new Vector3[numX];
            for (int ix = 0; ix < numX; ix++)
            {
                int idx = ix; // iy * numX + ix with iy = 0
                bottomIndices[ix] = idx;
                solver.invMasses[idx] = 0f;
                bottomLocalOffsets[ix] = bottomCuboid.InverseTransformPoint(solver.positions[idx]);
            }
        }

        SetAnchorPositions();
    }

    void Update()
    {
        if (cloth == null) return;
        SetAnchorPositions();
    }

    private void SetAnchorPositions()
    {
        if (topCuboid != null)
        {
            for (int i = 0; i < topIndices.Length; i++)
                solver.positions[topIndices[i]] = topCuboid.TransformPoint(topLocalOffsets[i]);
        }

        if (bottomCuboid != null)
        {
            for (int i = 0; i < bottomIndices.Length; i++)
                solver.positions[bottomIndices[i]] = bottomCuboid.TransformPoint(bottomLocalOffsets[i]);
        }
    }
}
