using UnityEngine;

[RequireComponent(typeof(VBDCloth))]
public class VBDFixedCorners : MonoBehaviour
{
    private VBDCloth cloth;

    void Start()
    {
        cloth = GetComponent<VBDCloth>();
        if (cloth == null) return;

        int numX = cloth.numX;
        int numY = cloth.numY;

        int topLeftIdx = (numY - 1) * numX;
        int topRightIdx = (numY - 1) * numX + (numX - 1);

        cloth.Solver.invMasses[topLeftIdx] = 0f;
        cloth.Solver.invMasses[topRightIdx] = 0f;
    }
}
