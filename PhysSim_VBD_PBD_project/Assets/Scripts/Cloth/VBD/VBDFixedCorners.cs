using UnityEngine;

[RequireComponent(typeof(VBDClothSim))]
public class VBDFixedCorners : MonoBehaviour
{
    private VBDClothSim clothSim;

    void Start()
    {
        clothSim = GetComponent<VBDClothSim>();
        if (clothSim == null) return;

        int numX = clothSim.numX;
        int numY = clothSim.numY;

        int topLeftIdx = (numY - 1) * numX;
        int topRightIdx = (numY - 1) * numX + (numX - 1);

        clothSim.invMasses[topLeftIdx] = 0f;
        clothSim.invMasses[topRightIdx] = 0f;
    }
}
