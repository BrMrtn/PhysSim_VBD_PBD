using UnityEngine;

[RequireComponent(typeof(VBDCloth))]
public class VBDFollowCorners : MonoBehaviour
{
    public SphereCollider sphereColliderLeft;
    public SphereCollider sphereColliderRight;

    private VBDCloth clothSim;
    private VBDSolver solver;

    private int topLeftIdx;
    private int topRightIdx;

    private Transform leftTransform;
    private Transform rightTransform;

    void Start()
    {
        clothSim = GetComponent<VBDCloth>();
        if (clothSim != null)
        {
            solver = clothSim.Solver;
            leftTransform = sphereColliderLeft.transform;
            rightTransform = sphereColliderRight.transform;

            int numX = clothSim.numX;
            int numY = clothSim.numY;
            topLeftIdx = (numY - 1) * numX;
            topRightIdx = (numY - 1) * numX + (numX - 1);

            solver.invMasses[topLeftIdx] = 0f;
            solver.invMasses[topRightIdx] = 0f;

            SetAnchorPositions();
        }
    }

    void Update()
    {
        if (clothSim != null && sphereColliderLeft != null && sphereColliderRight != null)
            SetAnchorPositions();
    }

    private void SetAnchorPositions()
    {
        solver.positions[topLeftIdx] = leftTransform.position;
        solver.positions[topRightIdx] = rightTransform.position;
    }
}
