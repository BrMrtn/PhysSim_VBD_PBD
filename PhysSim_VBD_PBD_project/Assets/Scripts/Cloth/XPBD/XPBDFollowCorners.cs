using NUnit.Framework;
using UnityEngine;
using System.Collections.Generic;
using System;

[RequireComponent(typeof(XPBDClothSim))]
public class XPBDFollowCorners : MonoBehaviour
{
    public SphereCollider sphereColliderLeft;
    public SphereCollider sphereColliderRight;

    private XPBDClothSim clothSim;

    private int leftAnchorIdx;
    private int rightAnchorIdx;

    private Transform leftTransform;
    private Transform rightTransform;

    void Start()
    {
        clothSim = GetComponent<XPBDClothSim>();
        if (clothSim != null)
        {
            leftTransform = sphereColliderLeft.transform;
            rightTransform = sphereColliderRight.transform;

            int numX = clothSim.numX;
            int numY = clothSim.numY;
            int topLeftIdx = (numY - 1) * numX;
            int topRightIdx = (numY - 1) * numX + (numX - 1);

            int currentLength = clothSim.positions.Length;
            leftAnchorIdx = currentLength;
            rightAnchorIdx = currentLength + 1;

            Array.Resize(ref clothSim.positions, clothSim.positions.Length + 2);
            Array.Resize(ref clothSim.invMasses, clothSim.invMasses.Length + 2);

            clothSim.invMasses[leftAnchorIdx] = 0f;
            clothSim.invMasses[rightAnchorIdx] = 0f;

            SetAnchorPositions();

            var constraintsList = new List<DistanceConstraint>(clothSim.constraints);
            constraintsList.Add(new DistanceConstraint(topLeftIdx, leftAnchorIdx, 0f, 0f));
            constraintsList.Add(new DistanceConstraint(topRightIdx, rightAnchorIdx, 0f, 0f));
            clothSim.constraints = constraintsList.ToArray();
        }
    }

    void Update()
    {
        if (clothSim != null && sphereColliderLeft != null && sphereColliderRight != null)
            SetAnchorPositions();
    }

    private void SetAnchorPositions()
    {
        clothSim.positions[leftAnchorIdx] = leftTransform.position;
        clothSim.positions[rightAnchorIdx] = rightTransform.position;
    }
}