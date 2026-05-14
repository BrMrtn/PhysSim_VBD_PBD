using UnityEngine;

[RequireComponent(typeof(VBDCloth))]
public class VBDFixedCorners : MonoBehaviour
{
    private VBDCloth clothSim;

    void Start()
    {
        clothSim = GetComponent<VBDCloth>();
        if (clothSim == null) return;

        int numX = clothSim.numX;
        int numY = clothSim.numY;

        int topLeftIdx = (numY - 1) * numX;
        int topRightIdx = (numY - 1) * numX + (numX - 1);

        clothSim.Solver.invMasses[topLeftIdx] = 0f;
        clothSim.Solver.invMasses[topRightIdx] = 0f;
    }
}
