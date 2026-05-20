using UnityEngine;

[RequireComponent(typeof(XPBDCloth))]
public class XPBDFixedCorners : MonoBehaviour
{
    private XPBDCloth cloth;

    void Start()
    {
        cloth = GetComponent<XPBDCloth>();

        if (cloth == null)
            return;

        int numX = cloth.numX;
        int numY = cloth.numY;

        int topLeftIdx = (numY - 1) * numX;
        int topRightIdx = (numY - 1) * numX + (numX - 1);

        cloth.Solver.invMasses[topLeftIdx] = 0f;
        cloth.Solver.invMasses[topRightIdx] = 0f;
    }
}