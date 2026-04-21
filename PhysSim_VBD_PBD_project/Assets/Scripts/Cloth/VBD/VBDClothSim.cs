using System;
using System.Collections.Generic;
using UnityEngine;

public class VBDClothSim : MonoBehaviour
{
    [Header("Time / Solver")]
    public int numSubsteps = 15;
    public int vbdIterations = 8;

    [Header("Material")]
    public float density = 0.2f; // kg/m^2

    [Tooltip("Interpreted as an inverse-stiffness style control. 0 = very stiff.")]
    public float stretchingCompliance = 0.0f;

    [Tooltip("Interpreted as an inverse-stiffness style control. 0 = very stiff.")]
    public float bendingCompliance = 1.0f;

    [Tooltip("Simple global velocity damping used as a Unity-friendly approximation.")]
    [Range(0f, 1f)] public float velocityDamping = 0.0f;

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
    private float[] invMasses;

    private StretchConstraint[] stretchingConstraints;
    private BendConstraint[] bendingConstraints;
    private List<int>[] stretchIncident;
    private List<int>[] bendIncident;
    private int[][] vertexColors;

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
    }

    void FixedUpdate()
    {
        if (mesh == null)
            return;

        float sdt = Time.fixedDeltaTime / Mathf.Max(1, numSubsteps);
        float invH2 = 1f / (sdt * sdt);

        float stretchK = ComplianceToStiffness(stretchingCompliance);
        float bendK = ComplianceToStiffness(bendingCompliance);

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

            for (int iter = 0; iter < vbdIterations; iter++)
                SolveElasticVBDPass(invH2, stretchK, bendK);

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
