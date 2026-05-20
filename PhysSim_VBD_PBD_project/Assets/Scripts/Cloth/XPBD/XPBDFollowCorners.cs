using UnityEngine;

[RequireComponent(typeof(XPBDCloth))]
public class XPBDFollowCorners : MonoBehaviour
{
    public SphereCollider sphereColliderLeft;
    public SphereCollider sphereColliderRight;

    private XPBDCloth cloth;
    private XPBDSolver solver;

    private int topLeftIdx;
    private int topRightIdx;

    private Transform leftTransform;
    private Transform rightTransform;

    void Start()
    {
        cloth = GetComponent<XPBDCloth>();
        if (cloth != null)
        {
            solver = cloth.Solver;
            leftTransform = sphereColliderLeft.transform;
            rightTransform = sphereColliderRight.transform;

            int numX = cloth.numX;
            int numY = cloth.numY;
            topLeftIdx = (numY - 1) * numX;
            topRightIdx = (numY - 1) * numX + (numX - 1);

            solver.invMasses[topLeftIdx] = 0f;
            solver.invMasses[topRightIdx] = 0f;

            SetAnchorPositions();
        }
    }

    void Update()
    {
        if (cloth != null && sphereColliderLeft != null && sphereColliderRight != null)
            SetAnchorPositions();
    }

    private void SetAnchorPositions()
    {
        solver.positions[topLeftIdx] = leftTransform.position;
        solver.positions[topRightIdx] = rightTransform.position;
    }
}
