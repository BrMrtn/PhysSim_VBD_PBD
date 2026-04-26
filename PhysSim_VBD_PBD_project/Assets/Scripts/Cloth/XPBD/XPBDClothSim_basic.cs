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

        private int numVerts;
        private Vector3 gravity = new Vector3(0, -9.81f, 0);

        private MeshFilter meshFilter;
        private Mesh mesh;
        private Transform tr;
        private Vector3[] renderVertices;

        private Vector3[] positions;
        private Vector3[] velocities;
        private Vector3[] predictedPositions;
        private float[] invMasses;

        private int[] meshToGrid;

        private struct DistanceConstraint
        {
            public int p1Idx;
            public int p2Idx;
            public float restLength;
            public float compliance;
        }

        private DistanceConstraint[] constraints;

        void Start()
        {
            meshFilter = gameObject.GetComponent<MeshFilter>();
            mesh = meshFilter.mesh;
            tr = meshFilter.transform;
            mesh.MarkDynamic();

            if (meshFilter == null || mesh == null)
            {
                Debug.LogWarning("No MeshFilter or Mesh found!");
                return;
            }

            MeshCollider meshCollider = gameObject.GetComponent<MeshCollider>();
            if (meshCollider != null)
            {
                if (Application.isPlaying) Destroy(meshCollider);
                else DestroyImmediate(meshCollider);
            }

            positions = mesh.vertices;
            numVerts = positions.Length;
            velocities = new Vector3[numVerts];
            predictedPositions = new Vector3[numVerts];
            renderVertices = new Vector3[numVerts];
            invMasses = new float[numVerts];

            // Convert local-space particle positions to world-space
            tr.TransformPoints(positions);

            // TODO: Find the coordinates for which invMass should be set to 0.


            Array.Fill(invMasses, 1.0f);

            float minX = float.MaxValue;
            float maxX = float.MinValue;
            float maxY = float.MinValue;

            for (int i = 0; i < numVerts; i++)
            {
                if (positions[i].x < minX) minX = positions[i].x;
                if (positions[i].x > maxX) maxX = positions[i].x;
                if (positions[i].y > maxY) maxY = positions[i].y;
            }

            float eps = 0.0001f;
            for (int i = 0; i < numVerts; i++)
            {
                float x = positions[i].x;
                float y = positions[i].y;
                if ((y > maxY - eps) && (x < minX + eps || x > maxX - eps))
                    invMasses[i] = 0f;
            }

            InitConstraints();
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

            // update mesh vertices
            for (int i = 0; i < numVerts; i++)
                renderVertices[i] = tr.InverseTransformPoint(positions[i]); // TODO renderVertices might be used wrong

            mesh.SetVertices(renderVertices);
            mesh.RecalculateNormals();
        }
        private void InitConstraints()
        {
            // Determine grid dimensions (numX, numY)
            var xCoords = new List<float>();
            var yCoords = new List<float>();
            var zCoords = new List<float>();
            float epsilon = 0.001f;

            foreach (var v in positions)
            {
                if (!xCoords.Any(x => Mathf.Abs(x - v.x) < epsilon)) xCoords.Add(v.x);
                if (!yCoords.Any(y => Mathf.Abs(y - v.y) < epsilon)) yCoords.Add(v.y);
                if (!zCoords.Any(z => Mathf.Abs(z - v.z) < epsilon)) zCoords.Add(v.z);
            }

            xCoords.Sort();
            yCoords.Sort();
            zCoords.Sort();

            int numX, numY;
            bool isXZ = zCoords.Count > yCoords.Count;

            if (isXZ)
            {
                numX = xCoords.Count;
                numY = zCoords.Count;
            }
            else
            {
                numX = xCoords.Count;
                numY = yCoords.Count;
            }

            // Map particles to a grid
            var particleGridMap = new int[numX, numY];
            for (int i = 0; i < numVerts; i++)
            {
                var p = positions[i];
                int x = xCoords.FindIndex(c => Mathf.Abs(c - p.x) < epsilon);
                int y = isXZ ? zCoords.FindIndex(c => Mathf.Abs(c - p.z) < epsilon) : yCoords.FindIndex(c => Mathf.Abs(c - p.y) < epsilon);
                if (x != -1 && y != -1)
                {
                    particleGridMap[x, y] = i;
                }
            }

            var constraintsList = new List<DistanceConstraint>();
            var offsets = new int[] { 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 2, 0, 0, 2, 0 };
            var compliances = new float[] { stretchingCompliance, stretchingCompliance, shearCompliance, shearCompliance, bendingCompliance, bendingCompliance };

            for (int constType = 0; constType < 6; constType++)
            {
                for (int i = 0; i < numX; i++)
                {
                    for (int j = 0; j < numY; j++)
                    {
                        int p = 4 * constType;
                        int i0 = i + offsets[p];
                        int j0 = j + offsets[p + 1];
                        int i1 = i + offsets[p + 2];
                        int j1 = j + offsets[p + 3];

                        if (i0 < numX && j0 < numY && i1 < numX && j1 < numY)
                        {
                            int p1Idx = particleGridMap[i0, j0];
                            int p2Idx = particleGridMap[i1, j1];

                            constraintsList.Add(new DistanceConstraint
                            {
                                p1Idx = p1Idx,
                                p2Idx = p2Idx,
                                restLength = Vector3.Distance(positions[p1Idx], positions[p2Idx]),
                                compliance = compliances[constType]
                            });
                        }
                    }
                }
            }
            constraints = constraintsList.ToArray();
        }
    }
}