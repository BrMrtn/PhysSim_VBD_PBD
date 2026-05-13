using UnityEngine;

[RequireComponent(typeof(XPBDCloth))]
public class XPBDFixedCorners : MonoBehaviour
{
    private XPBDCloth clothSim;

    void Start()
    {
        clothSim = GetComponent<XPBDCloth>();

        if (clothSim == null)
            return;

        int numX = clothSim.numX;
        int numY = clothSim.numY;

        int topLeftIdx = (numY - 1) * numX;
        int topRightIdx = (numY - 1) * numX + (numX - 1);

        clothSim.invMasses[topLeftIdx] = 0f;
        clothSim.invMasses[topRightIdx] = 0f;
    }
}