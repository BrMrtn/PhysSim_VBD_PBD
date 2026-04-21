using System;
using System.Collections.Generic;
using UnityEngine;

namespace GPT
{
    public class GPT_VBDClothSim : MonoBehaviour
    {
        [Header("Time / Solver")]
        public int numSubsteps = 15;
        public int vbdIterations = 8;
        public int collisionIterations = 1;

        [Header("Material")]
        public float density = 0.2f; // kg/m^2

        [Tooltip("Interpreted as an inverse-stiffness style control. 0 = very stiff.")]
        public float stretchingCompliance = 0.0f;

        [Tooltip("Interpreted as an inverse-stiffness style control. 0 = very stiff.")]
        public float bendingCompliance = 1.0f;

        [Tooltip("Simple global velocity damping used as a Unity-friendly approximation.")]
        [Range(0f, 1f)] public float velocityDamping = 0.0f;

        [Header("Self Collision")]
        public bool handleSelfCollisions = true;
        public float particleThickness = 0.01f;
        [Range(0f, 1f)] public float clothClothFriction = 0.0f;
        public float hashSpacing = 0.02f;

        [Tooltip("How often to rebuild collision pairs inside one VBD iteration. 1 = every iteration.")]
        public int collisionRebuildFrequency = 1;

        [Header("Numerics")]
        [Tooltip("Small stabilizer added to the local Hessian.")]
        public float hessianRegularization = 1e-8f;

        [Tooltip("Optional cap for a single local Newton step.")]
        public float maxLocalStep = 0.02f;

        private readonly Vector3 gravity = new Vector3(0, -9.81f, 0);

        private MeshFilter meshFilter;
        private Mesh mesh;
        private Transform tr;

        private int n;
        private Vector3[] positions;
        private Vector3[] previousPositions;
        private Vector3[] velocities;
        private Vector3[] renderVertices;
        private Vector3[] restPositions;
        private float[] invMasses;

        private StretchConstraint[] stretchingConstraints;
        private BendConstraint[] bendingConstraints;
        private List<int>[] stretchIncident;
        private List<int>[] bendIncident;
        private int[][] vertexColors;

        private SpatialHash spatialHash;
        private readonly List<ContactPair> collisionPairs = new List<ContactPair>();

        private const float Eps = 1e-8f;

        private struct StretchConstraint
        {
            public int i, j;
            public float restLength;
            public StretchConstraint(int i, int j, float restLength)
            {
                this.i = i;
                this.j = j;
                this.restLength = restLength;
            }
        }

        private struct BendConstraint
        {
            public int i, j;
            public float restLength;
            public BendConstraint(int i, int j, float restLength)
            {
                this.i = i;
                this.j = j;
                this.restLength = restLength;
            }
        }

        private struct ContactPair
        {
            public int i, j;
            public float targetDistance;
            public ContactPair(int i, int j, float targetDistance)
            {
                this.i = i;
                this.j = j;
                this.targetDistance = targetDistance;
            }
        }

        private struct Matrix3x3
        {
            public float m00, m01, m02;
            public float m10, m11, m12;
            public float m20, m21, m22;

            public static Matrix3x3 Identity
            {
                get
                {
                    return new Matrix3x3 { m00 = 1f, m11 = 1f, m22 = 1f };
                }
            }

            public static Matrix3x3 operator +(Matrix3x3 a, Matrix3x3 b)
            {
                return new Matrix3x3
                {
                    m00 = a.m00 + b.m00,
                    m01 = a.m01 + b.m01,
                    m02 = a.m02 + b.m02,
                    m10 = a.m10 + b.m10,
                    m11 = a.m11 + b.m11,
                    m12 = a.m12 + b.m12,
                    m20 = a.m20 + b.m20,
                    m21 = a.m21 + b.m21,
                    m22 = a.m22 + b.m22
                };
            }

            public static Matrix3x3 operator *(float s, Matrix3x3 a)
            {
                return new Matrix3x3
                {
                    m00 = s * a.m00,
                    m01 = s * a.m01,
                    m02 = s * a.m02,
                    m10 = s * a.m10,
                    m11 = s * a.m11,
                    m12 = s * a.m12,
                    m20 = s * a.m20,
                    m21 = s * a.m21,
                    m22 = s * a.m22
                };
            }

            public static Matrix3x3 Outer(Vector3 a, Vector3 b)
            {
                return new Matrix3x3
                {
                    m00 = a.x * b.x,
                    m01 = a.x * b.y,
                    m02 = a.x * b.z,
                    m10 = a.y * b.x,
                    m11 = a.y * b.y,
                    m12 = a.y * b.z,
                    m20 = a.z * b.x,
                    m21 = a.z * b.y,
                    m22 = a.z * b.z
                };
            }

            public Matrix3x3 AddDiagonal(float d)
            {
                m00 += d;
                m11 += d;
                m22 += d;
                return this;
            }

            public bool TrySolve(Vector3 b, out Vector3 x)
            {
                float det =
                    m00 * (m11 * m22 - m12 * m21) -
                    m01 * (m10 * m22 - m12 * m20) +
                    m02 * (m10 * m21 - m11 * m20);

                if (Mathf.Abs(det) < 1e-12f || float.IsNaN(det) || float.IsInfinity(det))
                {
                    x = Vector3.zero;
                    return false;
                }

                float invDet = 1f / det;
                float c00 = (m11 * m22 - m12 * m21) * invDet;
                float c01 = (m02 * m21 - m01 * m22) * invDet;
                float c02 = (m01 * m12 - m02 * m11) * invDet;
                float c10 = (m12 * m20 - m10 * m22) * invDet;
                float c11 = (m00 * m22 - m02 * m20) * invDet;
                float c12 = (m02 * m10 - m00 * m12) * invDet;
                float c20 = (m10 * m21 - m11 * m20) * invDet;
                float c21 = (m01 * m20 - m00 * m21) * invDet;
                float c22 = (m00 * m11 - m01 * m10) * invDet;

                x = new Vector3(
                    c00 * b.x + c01 * b.y + c02 * b.z,
                    c10 * b.x + c11 * b.y + c12 * b.z,
                    c20 * b.x + c21 * b.y + c22 * b.z
                );
                return true;
            }
        }

        private sealed class SpatialHash
        {
            private readonly float spacing;
            private readonly int tableSize;
            private readonly int[] cellStart;
            private readonly int[] cellEntries;

            public readonly List<int> pairA;
            public readonly List<int> pairB;

            public SpatialHash(float spacing, int maxObjects)
            {
                this.spacing = spacing;
                tableSize = Mathf.Max(1, 5 * maxObjects);
                cellStart = new int[tableSize + 1];
                cellEntries = new int[maxObjects];
                pairA = new List<int>(10 * maxObjects);
                pairB = new List<int>(10 * maxObjects);
            }

            private static int HashCoords(int xi, int yi, int zi, int tableSize)
            {
                int h = (xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481);
                if (h < 0) h = -h;
                return h % tableSize;
            }

            private int IntCoord(float coord) => Mathf.FloorToInt(coord / spacing);

            private int HashPos(Vector3[] pos, int id)
            {
                Vector3 p = pos[id];
                return HashCoords(IntCoord(p.x), IntCoord(p.y), IntCoord(p.z), tableSize);
            }

            public void Create(Vector3[] pos, int count)
            {
                Array.Clear(cellStart, 0, cellStart.Length);

                for (int i = 0; i < count; i++)
                    cellStart[HashPos(pos, i)]++;

                int start = 0;
                for (int i = 0; i < tableSize; i++)
                {
                    start += cellStart[i];
                    cellStart[i] = start;
                }
                cellStart[tableSize] = start;

                for (int i = 0; i < count; i++)
                {
                    int h = HashPos(pos, i);
                    cellStart[h]--;
                    cellEntries[cellStart[h]] = i;
                }
            }

            public void QueryAll(Vector3[] pos, int count, float maxDist)
            {
                pairA.Clear();
                pairB.Clear();
                float maxDist2 = maxDist * maxDist;

                for (int i = 0; i < count; i++)
                {
                    Vector3 p = pos[i];
                    int x0 = IntCoord(p.x - maxDist);
                    int y0 = IntCoord(p.y - maxDist);
                    int z0 = IntCoord(p.z - maxDist);
                    int x1 = IntCoord(p.x + maxDist);
                    int y1 = IntCoord(p.y + maxDist);
                    int z1 = IntCoord(p.z + maxDist);

                    for (int xi = x0; xi <= x1; xi++)
                        for (int yi = y0; yi <= y1; yi++)
                            for (int zi = z0; zi <= z1; zi++)
                            {
                                int h = HashCoords(xi, yi, zi, tableSize);
                                int start = cellStart[h];
                                int end = cellStart[h + 1];

                                for (int k = start; k < end; k++)
                                {
                                    int j = cellEntries[k];
                                    if (j >= i)
                                        continue;

                                    if ((pos[i] - pos[j]).sqrMagnitude > maxDist2)
                                        continue;

                                    pairA.Add(i);
                                    pairB.Add(j);
                                }
                            }
                }
            }
        }

        void Start()
        {
            meshFilter = GetComponent<MeshFilter>();
            mesh = meshFilter != null ? meshFilter.mesh : null;
            tr = meshFilter != null ? meshFilter.transform : transform;

            if (meshFilter == null || mesh == null)
            {
                Debug.LogWarning("No MeshFilter or Mesh found!");
                enabled = false;
                return;
            }

            mesh.MarkDynamic();

            MeshCollider meshCollider = GetComponent<MeshCollider>();
            if (meshCollider != null)
            {
                if (Application.isPlaying) Destroy(meshCollider);
                else DestroyImmediate(meshCollider);
            }

            positions = mesh.vertices;
            n = positions.Length;
            previousPositions = new Vector3[n];
            velocities = new Vector3[n];
            renderVertices = new Vector3[n];
            restPositions = new Vector3[n];
            invMasses = new float[n];

            // Mass from the rest mesh area.
            int[] tris = mesh.triangles;
            for (int i = 0; i < tris.Length; i += 3)
            {
                int v1 = tris[i];
                int v2 = tris[i + 1];
                int v3 = tris[i + 2];

                Vector3 p1 = positions[v1];
                Vector3 p2 = positions[v2];
                Vector3 p3 = positions[v3];

                float area = Vector3.Cross(p2 - p1, p3 - p1).magnitude * 0.5f;
                float triMass = area * density;
                float pInvMass = triMass > 0f ? (1f / triMass) / 3f : 0f;

                invMasses[v1] += pInvMass;
                invMasses[v2] += pInvMass;
                invMasses[v3] += pInvMass;
            }

            // Convert local-space particle positions to world-space.
            for (int i = 0; i < n; i++)
                positions[i] = tr.TransformPoint(positions[i]);

            // Pin the top corners, same idea as the original XPBD script.
            float minX = float.MaxValue;
            float maxX = float.MinValue;
            float maxY = float.MinValue;
            for (int i = 0; i < n; i++)
            {
                if (positions[i].x < minX) minX = positions[i].x;
                if (positions[i].x > maxX) maxX = positions[i].x;
                if (positions[i].y > maxY) maxY = positions[i].y;
            }

            float eps = 0.0001f;
            for (int i = 0; i < n; i++)
            {
                float x = positions[i].x;
                float y = positions[i].y;
                if ((y > maxY - eps) && (x < minX + eps || x > maxX - eps))
                    invMasses[i] = 0f;
            }

            InitConstraints();
            BuildVertexColors();

            // Small noise helps break symmetry.
            for (int i = 0; i < n; i++)
            {
                if (invMasses[i] > 0f)
                    positions[i] += UnityEngine.Random.insideUnitSphere * 0.01f;
            }

            Array.Copy(positions, restPositions, n);

            float spacing = Mathf.Max(hashSpacing, particleThickness, 0.0001f);
            spatialHash = new SpatialHash(spacing, n);
        }

        void FixedUpdate()
        {
            if (mesh == null)
                return;

            float sdt = Time.fixedDeltaTime / Mathf.Max(1, numSubsteps);
            float invH2 = 1f / (sdt * sdt);

            float stretchK = ComplianceToStiffness(stretchingCompliance);
            float bendK = ComplianceToStiffness(bendingCompliance);
            float collisionK = stretchK;

            for (int sub = 0; sub < numSubsteps; sub++)
            {
                for (int i = 0; i < n; i++)
                    previousPositions[i] = positions[i];

                // Initial guess y = x + h v + h^2 a.
                for (int i = 0; i < n; i++)
                {
                    if (invMasses[i] == 0f)
                        continue;

                    velocities[i] += gravity * sdt;
                    velocities[i] *= Mathf.Clamp01(1f - velocityDamping);
                    positions[i] = previousPositions[i] + velocities[i] * sdt;
                }

                if (handleSelfCollisions)
                    UpdateCollisionPairs();

                for (int iter = 0; iter < vbdIterations; iter++)
                {
                    SolveElasticVBDPass(invH2, stretchK, bendK);

                    if (handleSelfCollisions)
                    {
                        if (collisionRebuildFrequency > 0 && (iter % collisionRebuildFrequency) == 0)
                            UpdateCollisionPairs();

                        for (int c = 0; c < Mathf.Max(1, collisionIterations); c++)
                            SolveCollisionPairs(collisionK);
                    }
                }

                for (int i = 0; i < n; i++)
                {
                    if (invMasses[i] == 0f)
                        continue;

                    velocities[i] = (positions[i] - previousPositions[i]) / sdt;
                }
            }

            for (int i = 0; i < n; i++)
                renderVertices[i] = tr.InverseTransformPoint(positions[i]);

            mesh.SetVertices(renderVertices);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
        }

        private void SolveElasticVBDPass(float invH2, float stretchK, float bendK)
        {
            if (vertexColors == null || vertexColors.Length == 0)
                return;

            for (int c = 0; c < vertexColors.Length; c++)
            {
                int[] group = vertexColors[c];
                for (int idx = 0; idx < group.Length; idx++)
                {
                    int i = group[idx];
                    if (invMasses[i] == 0f)
                        continue;

                    float mass = 1f / invMasses[i];
                    Vector3 fi = -(mass * invH2) * (positions[i] - previousPositions[i]);
                    Matrix3x3 Hi = (mass * invH2) * Matrix3x3.Identity;

                    var sList = stretchIncident[i];
                    for (int k = 0; k < sList.Count; k++)
                    {
                        StretchConstraint s = stretchingConstraints[sList[k]];
                        AddLocalPairContribution(i, s.i, s.j, s.restLength, stretchK, ref fi, ref Hi);
                    }

                    var bList = bendIncident[i];
                    for (int k = 0; k < bList.Count; k++)
                    {
                        BendConstraint b = bendingConstraints[bList[k]];
                        AddLocalPairContribution(i, b.i, b.j, b.restLength, bendK, ref fi, ref Hi);
                    }

                    Hi = Hi.AddDiagonal(hessianRegularization);
                    if (Hi.TrySolve(fi, out Vector3 delta))
                    {
                        if (delta.sqrMagnitude > maxLocalStep * maxLocalStep)
                            delta = delta.normalized * maxLocalStep;

                        positions[i] += delta;
                    }
                }
            }
        }

        private void AddLocalPairContribution(int solveVertex, int i, int j, float restLength, float stiffness, ref Vector3 fi, ref Matrix3x3 Hi)
        {
            if (solveVertex != i && solveVertex != j)
                return;

            Vector3 xi = positions[i];
            Vector3 xj = positions[j];
            Vector3 d = xi - xj;
            float len = d.magnitude;
            if (len < Eps)
                return;

            Vector3 n = d / len;
            float C = len - restLength;
            Vector3 grad = stiffness * C * n;

            // Exact Hessian wrt the solved vertex.
            float L_over_r = restLength / len;
            Matrix3x3 Hpair = stiffness * (((1f - L_over_r) * Matrix3x3.Identity) + (L_over_r * Matrix3x3.Outer(n, n)));

            if (solveVertex == i)
            {
                fi -= grad;
                Hi += Hpair;
            }
            else
            {
                fi += grad;
                Hi += Hpair;
            }
        }

        private void SolveCollisionPairs(float collisionK)
        {
            if (!handleSelfCollisions || collisionPairs.Count == 0)
                return;

            for (int idx = 0; idx < collisionPairs.Count; idx++)
            {
                ContactPair c = collisionPairs[idx];
                int i = c.i;
                int j = c.j;

                if (invMasses[i] == 0f && invMasses[j] == 0f)
                    continue;

                Vector3 d = positions[i] - positions[j];
                float len = d.magnitude;
                if (len < Eps || len >= c.targetDistance)
                    continue;

                float w0 = invMasses[i];
                float w1 = invMasses[j];
                float w = w0 + w1;
                if (w <= 0f)
                    continue;

                Vector3 n = d / len;
                float C = len - c.targetDistance;
                float corrMag = -collisionK * C / w;
                Vector3 corr = corrMag * n;

                positions[i] += corr * w0;
                positions[j] -= corr * w1;

                if (clothClothFriction > 0f)
                {
                    Vector3 v0 = positions[i] - previousPositions[i];
                    Vector3 v1 = positions[j] - previousPositions[j];
                    Vector3 avg = 0.5f * (v0 + v1);
                    positions[i] += (avg - v0) * clothClothFriction;
                    positions[j] += (avg - v1) * clothClothFriction;
                }
            }
        }

        private void UpdateCollisionPairs()
        {
            collisionPairs.Clear();
            if (!handleSelfCollisions)
                return;

            spatialHash.Create(positions, n);
            float maxDist = Mathf.Max(particleThickness, 0.2f * particleThickness + particleThickness);
            spatialHash.QueryAll(positions, n, maxDist);

            float thickness2 = particleThickness * particleThickness;
            for (int k = 0; k < spatialHash.pairA.Count; k++)
            {
                int i = spatialHash.pairA[k];
                int j = spatialHash.pairB[k];

                Vector3 delta = positions[i] - positions[j];
                float dist2 = delta.sqrMagnitude;
                if (dist2 == 0f || dist2 > thickness2)
                    continue;

                float restDist2 = (restPositions[i] - restPositions[j]).sqrMagnitude;
                if (dist2 > restDist2)
                    continue;

                float targetDist = Mathf.Min(particleThickness, Mathf.Sqrt(restDist2));
                if (targetDist <= 0f)
                    continue;

                collisionPairs.Add(new ContactPair(i, j, targetDist));
            }
        }

        private void InitConstraints()
        {
            var stretchingList = new List<StretchConstraint>();
            var bendingList = new List<BendConstraint>();
            var edges = new Dictionary<long, int>();

            int[] triangles = mesh.triangles;
            for (int t = 0; t < triangles.Length; t += 3)
            {
                int v1 = triangles[t];
                int v2 = triangles[t + 1];
                int v3 = triangles[t + 2];

                AddEdgeAndBending(v1, v2, v3, stretchingList, bendingList, edges);
                AddEdgeAndBending(v2, v3, v1, stretchingList, bendingList, edges);
                AddEdgeAndBending(v3, v1, v2, stretchingList, bendingList, edges);
            }

            stretchingConstraints = stretchingList.ToArray();
            bendingConstraints = bendingList.ToArray();

            stretchIncident = new List<int>[n];
            bendIncident = new List<int>[n];
            for (int i = 0; i < n; i++)
            {
                stretchIncident[i] = new List<int>(8);
                bendIncident[i] = new List<int>(4);
            }

            for (int i = 0; i < stretchingConstraints.Length; i++)
            {
                var c = stretchingConstraints[i];
                stretchIncident[c.i].Add(i);
                stretchIncident[c.j].Add(i);
            }

            for (int i = 0; i < bendingConstraints.Length; i++)
            {
                var c = bendingConstraints[i];
                bendIncident[c.i].Add(i);
                bendIncident[c.j].Add(i);
            }
        }

        private void AddEdgeAndBending(int v1, int v2, int oppositeVertex, List<StretchConstraint> stretchingList, List<BendConstraint> bendingList, Dictionary<long, int> edges)
        {
            int min = Mathf.Min(v1, v2);
            int max = Mathf.Max(v1, v2);
            long edgeKey = ((long)min << 32) | (uint)max;

            if (edges.TryGetValue(edgeKey, out int otherOppositeVertex))
            {
                float restLen = Vector3.Distance(positions[oppositeVertex], positions[otherOppositeVertex]);
                bendingList.Add(new BendConstraint(oppositeVertex, otherOppositeVertex, restLen));
            }
            else
            {
                edges.Add(edgeKey, oppositeVertex);
                float stretchLen = Vector3.Distance(positions[min], positions[max]);
                stretchingList.Add(new StretchConstraint(min, max, stretchLen));
            }
        }

        private void BuildVertexColors()
        {
            // Greedy coloring over the interaction graph (stretch + bend).
            List<int>[] neighbors = new List<int>[n];
            for (int i = 0; i < n; i++)
                neighbors[i] = new List<int>(8);

            for (int i = 0; i < stretchingConstraints.Length; i++)
            {
                var c = stretchingConstraints[i];
                neighbors[c.i].Add(c.j);
                neighbors[c.j].Add(c.i);
            }

            for (int i = 0; i < bendingConstraints.Length; i++)
            {
                var c = bendingConstraints[i];
                neighbors[c.i].Add(c.j);
                neighbors[c.j].Add(c.i);
            }

            int[] colors = new int[n];
            Array.Fill(colors, -1);
            int maxColor = -1;

            for (int v = 0; v < n; v++)
            {
                bool[] used = new bool[n + 1];
                var nb = neighbors[v];
                for (int k = 0; k < nb.Count; k++)
                {
                    int c = colors[nb[k]];
                    if (c >= 0 && c < used.Length)
                        used[c] = true;
                }

                int chosen = 0;
                while (chosen < used.Length && used[chosen])
                    chosen++;

                colors[v] = chosen;
                if (chosen > maxColor)
                    maxColor = chosen;
            }

            var groups = new List<int>[maxColor + 1];
            for (int c = 0; c <= maxColor; c++)
                groups[c] = new List<int>();

            for (int v = 0; v < n; v++)
                groups[colors[v]].Add(v);

            vertexColors = new int[groups.Length][];
            for (int c = 0; c < groups.Length; c++)
                vertexColors[c] = groups[c].ToArray();
        }

        private float ComplianceToStiffness(float compliance)
        {
            if (compliance <= 0f)
                return 1e5f;

            return Mathf.Clamp(1f / compliance, 1e-4f, 1e5f);
        }
    }
}