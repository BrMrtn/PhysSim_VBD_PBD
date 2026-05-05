using NUnit.Framework;
using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(XPBDClothSim))]
public class XPBDFollowCorners : MonoBehaviour
{
    public SphereCollider sphereColliderTopLeft;
    public SphereCollider sphereColliderTopRight;

    private XPBDClothSim clothSim;

    private int topLeftIdx;
    private int topRightIdx;

    private Transform leftTransform;
    private Transform rightTransform;

    void Start()
    {
        clothSim = GetComponent<XPBDClothSim>();
        if (clothSim != null)
        {
            int numX = clothSim.numX;
            int numY = clothSim.numY;
            topLeftIdx = (numY - 1) * numX;
            topRightIdx = (numY - 1) * numX + (numX - 1);

            // get clothSim.constraints, convert to list, add the two constraints, and convert back to array
            var positionsList = new List<Vector3>(clothSim.positions);
            positionsList.Add(leftTransform.position);
            positionsList.Add(rightTransform.position);

            var constraintsList = new List<DistanceConstraint>(clothSim.constraints);
            constraintsList.Add(new DistanceConstraint(topLeftIdx, positionsList.Count - 2, 0f, 0f));
            constraintsList.Add(new DistanceConstraint(topRightIdx, positionsList.Count - 1, 0f, 0f));

            clothSim.positions = positionsList.ToArray(); // TODO: length stays same, how to solve?
            clothSim.constraints = constraintsList.ToArray();
        }
    }
}