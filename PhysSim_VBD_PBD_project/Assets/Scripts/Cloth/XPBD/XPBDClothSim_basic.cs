using System;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;

namespace XPBD_stripped_down_version
{
    public class XPBDClothSim : MonoBehaviour
    {
        public int numSubsteps = 15;
        public float stretchingCompliance = 0.0f;
        public float shearCompliance = 0.0001f;
        public float bendingCompliance = 0.01f;

        private Vector3 gravity = new Vector3(0, -9.81f, 0);

        private MeshFilter meshFilter;
        private Mesh mesh;
        private Transform tr;

        private int numVerts;
        private Vector3[] positions;
        private Vector3[] velocities;
        private Vector3[] predictedPositions;
        private float[] invMasses;

        private int[] meshToGrid;
        private Vector3[] renderVertices;

        private struct DistanceConstraint
        {
            public int p1Idx, p2Idx;
            public float restLength, compliance;
        }

        private DistanceConstraint[] constraints;

        void Start()
        {
            meshFilter = gameObject.GetComponent<MeshFilter>();
            mesh = meshFilter.mesh;
            tr = transform;
            mesh.MarkDynamic();

            MeshCollider meshCollider = gameObject.GetComponent<MeshCollider>();
            if (meshCollider != null) Destroy(meshCollider);

            Vector3[] localVerts = mesh.vertices;
            numVerts = localVerts.Length;
            renderVertices = new Vector3[numVerts];

            // Get all different x and y values for numX and numY
            float eps = 0.001f;
            var xCoords = new List<float>();
            var yCoords = new List<float>();

            foreach (var v in localVerts)
            {
                if (!xCoords.Any(x => Mathf.Abs(x - v.x) < eps)) xCoords.Add(v.x);
                if (!yCoords.Any(y => Mathf.Abs(y - v.y) < eps)) yCoords.Add(v.y);
            }

            xCoords.Sort();
            yCoords.Sort();

            int numX = xCoords.Count;
            int numY = yCoords.Count;

            positions = new Vector3[numVerts];
            velocities = new Vector3[numVerts];
            predictedPositions = new Vector3[numVerts];
            invMasses = new float[numVerts];
            Array.Fill(invMasses, 1.0f);

            for (int iy = 0; iy < numY; iy++)
                for (int ix = 0; ix < numX; ix++)
                {
                    int idx = iy * numX + ix;
                    Vector3 localPos = new Vector3(xCoords[ix], yCoords[iy], 0f);
                    positions[idx] = tr.TransformPoint(localPos);
                }

            int topLeft = (numY - 1) * numX;
            int topRight = (numY - 1) * numX + (numX - 1);

            invMasses[topLeft] = 0f;
            invMasses[topRight] = 0f;

            // Map each mesh vertex to its grid particle index
            meshToGrid = new int[numVerts];
            for (int vi = 0; vi < numVerts; vi++)
            {
                int ix = xCoords.FindIndex(x => Mathf.Abs(x - localVerts[vi].x) < eps);
                int iy = yCoords.FindIndex(y => Mathf.Abs(y - localVerts[vi].y) < eps);
                meshToGrid[vi] = iy * numX + ix;
            }

            InitConstraints(numX, numY);
        }

        void FixedUpdate()
        {
            float sdt = Time.fixedDeltaTime / numSubsteps;
            float invSdt2 = 1.0f / (sdt * sdt);

            for (int step = 0; step < numSubsteps; step++)
            {
                for (int i = 0; i < numVerts; i++)
                {
                    if (invMasses[i] == 0f) continue;
                    velocities[i] += gravity * sdt;
                    predictedPositions[i] = positions[i];
                    positions[i] += velocities[i] * sdt;
                }

                // Solve constraints
                foreach (var constraint in constraints)
                {
                    int id0 = constraint.p1Idx;
                    int id1 = constraint.p2Idx;
                    float w0 = invMasses[id0];
                    float w1 = invMasses[id1];
                    float w = w0 + w1;
                    if (w == 0f) continue;

                    Vector3 grad = positions[id0] - positions[id1];
                    float len = grad.magnitude;
                    if (len == 0f) continue;

                    grad /= len;
                    float C = len - constraint.restLength;
                    float alpha = constraint.compliance * invSdt2;
                    float s = -C / (w + alpha);

                    positions[id0] += grad * (s * w0);
                    positions[id1] -= grad * (s * w1);
                }

                for (int i = 0; i < numVerts; i++)
                {
                    if (invMasses[i] == 0f) continue;
                    velocities[i] = (positions[i] - predictedPositions[i]) / sdt;
                }
            }

            for (int i = 0; i < numVerts; i++)
                renderVertices[i] = tr.InverseTransformPoint(positions[meshToGrid[i]]);

            mesh.SetVertices(renderVertices);
            mesh.RecalculateNormals();
        }

        private void InitConstraints(int numX, int numY)
        {
            var offsets = new int[]
            {
                0, 0,  1, 0,  // stretch horizontal
                0, 0,  0, 1,  // stretch vertical
                0, 0,  1, 1,  // shear
                1, 0,  0, 1,  // shear (other)
                0, 0,  2, 0,  // bend horizontal
                0, 0,  0, 2,  // bend vertical
            };
            var compliances = new float[]
            {
                stretchingCompliance, stretchingCompliance,
                shearCompliance,      shearCompliance,
                bendingCompliance,    bendingCompliance
            };

            var constraintsList = new List<DistanceConstraint>();

            for (int type = 0; type < 6; type++)
            {
                int p = type * 4;
                int offset_i0 = offsets[p];
                int offset_j0 = offsets[p + 1];
                int offset_i1 = offsets[p + 2];
                int offset_j1 = offsets[p + 3];

                for (int iy = 0; iy < numY; iy++)
                    for (int ix = 0; ix < numX; ix++)
                    {
                        int i0 = ix + offset_i0;
                        int j0 = iy + offset_j0;
                        int i1 = ix + offset_i1;
                        int j1 = iy + offset_j1;
                        if (i0 < numX && j0 < numY && i1 < numX && j1 < numY)
                        {
                            int p1 = j0 * numX + i0;
                            int p2 = j1 * numX + i1;
                            constraintsList.Add(new DistanceConstraint
                            {
                                p1Idx = p1,
                                p2Idx = p2,
                                restLength = Vector3.Distance(positions[p1], positions[p2]),
                                compliance = compliances[type]
                            });
                        }
                    }
            }

            constraints = constraintsList.ToArray();
        }
    }
}