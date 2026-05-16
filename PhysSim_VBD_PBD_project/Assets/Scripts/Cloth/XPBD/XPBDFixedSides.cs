using UnityEngine;

/// <summary>
/// Pins every cloth vertex on the top row and the bottom row of the simulation
/// grid to two arbitrary Transforms ("cuboids").
///
/// Each anchored vertex is stored as a local-space offset relative to its cuboid,
/// so translating *and rotating* the cuboid at runtime drags / swings the entire
/// edge of the cloth with it (the vertices keep their relative spacing along the
/// bar).
///
/// In the Unity Editor you can manipulate the cuboids live during Play mode using
/// the standard Move (W) and Rotate (E) tools in the Scene view -- no extra
/// library required. For a built game, hook any runtime-gizmo package
/// (e.g. RuntimeGizmos on GitHub) onto these same Transforms.
/// </summary>
[RequireComponent(typeof(XPBDCloth))]
public class XPBDFixedSides : MonoBehaviour
{
    public Transform topCuboid;
    public Transform bottomCuboid;

    private XPBDCloth clothSim;
    private XPBDSolver solver;

    private int[] topIndices;
    private int[] bottomIndices;

    // Initial local-space offset of each anchored vertex relative to its cuboid.
    private Vector3[] topLocalOffsets;
    private Vector3[] bottomLocalOffsets;

    void Start()
    {
        clothSim = GetComponent<XPBDCloth>();
        if (clothSim == null)
        {
            Debug.LogError($"{nameof(XPBDFixedSides)} requires an {nameof(XPBDCloth)} component on the same GameObject.");
            enabled = false;
            return;
        }

        solver = clothSim.Solver;
        int numX = clothSim.numX;
        int numY = clothSim.numY;

        // --- Top row: iy = numY - 1 -------------------------------------------
        topIndices = new int[numX];
        topLocalOffsets = new Vector3[numX];
        int topRowStart = (numY - 1) * numX;
        for (int ix = 0; ix < numX; ix++)
        {
            int idx = topRowStart + ix;
            topIndices[ix] = idx;
            solver.invMasses[idx] = 0f;

            if (topCuboid != null)
                topLocalOffsets[ix] = topCuboid.InverseTransformPoint(solver.positions[idx]);
        }

        // --- Bottom row: iy = 0 -----------------------------------------------
        bottomIndices = new int[numX];
        bottomLocalOffsets = new Vector3[numX];
        for (int ix = 0; ix < numX; ix++)
        {
            int idx = ix; // iy * numX + ix with iy = 0
            bottomIndices[ix] = idx;
            solver.invMasses[idx] = 0f;

            if (bottomCuboid != null)
                bottomLocalOffsets[ix] = bottomCuboid.InverseTransformPoint(solver.positions[idx]);
        }

        SetAnchorPositions();
    }

    void Update()
    {
        if (clothSim == null) return;
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