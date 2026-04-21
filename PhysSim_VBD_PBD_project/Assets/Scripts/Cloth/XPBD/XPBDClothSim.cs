using System;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class XPBDClothSim : MonoBehaviour
{
    public int numSubsteps = 15;
    public float density = 0.2f; // kg/m^2
    public float stretchingCompliance = 0.0f;
    public float bendingCompliance = 1.0f;
    public bool handleSelfCollisions = true;
    public float particleThickness = 0.01f;
    [Range(0f, 1f)] public float clothClothFriction = 0.0f;
    public float hashSpacing = 0.02f;

    private int n;
    private Vector3 gravity = new Vector3(0, -9.81f, 0);

    private MeshFilter meshFilter;
    private Mesh mesh;
    private Transform tr;
    private Vector3[] renderVertices;

    private Vector3[] positions;
    private Vector3[] velocities;
    private Vector3[] predictedPositions;
    private Vector3[] restPositions;
    private float[] invMasses;
    private DistanceConstraint[] stretchingConstraints;
    private BendingConstraint[] bendingConstraints;
    private SpatialHash spatialHash;

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

        // Delete MeshCollider if exists - it would clash with our own collision handling
        MeshCollider meshCollider = gameObject.GetComponent<MeshCollider>();
        if (meshCollider != null)
        {
            if (Application.isPlaying) Destroy(meshCollider);
            else DestroyImmediate(meshCollider);
        }

        positions = mesh.vertices;
        n = positions.Length;
        velocities = new Vector3[n];
        predictedPositions = new Vector3[n];
        renderVertices = new Vector3[n];
        restPositions = new Vector3[n];

        invMasses = new float[n];
        
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
            float pInvMass = triMass > 0.0f ? 1.0f / triMass / 3.0f : 0.0f;

            invMasses[v1] += pInvMass;
            invMasses[v2] += pInvMass;
            invMasses[v3] += pInvMass;
        }

        // Convert local-space particle positions to world-space
        //for (int i = 0; i < n; i++)
        //    positions[i] = tr.TransformPoint(positions[i]);
        tr.TransformPoints(positions);

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

        // Add small random noise to the positions to break perfect symmetries
        for (int i = 0; i < n; i++)
            if (invMasses[i] > 0f)
                positions[i] += UnityEngine.Random.insideUnitSphere * 0.01f;

        Array.Copy(positions, restPositions, n);

        float spacing = Mathf.Max(hashSpacing, particleThickness, 0.0001f);
        spatialHash = new SpatialHash(spacing, n);
    }

    void FixedUpdate()
    {
        float sdt = Time.fixedDeltaTime / numSubsteps;
        float invSdt2 = 1.0f / (sdt * sdt);
        float alphaStretch = stretchingCompliance * invSdt2;
        float alphaBend = bendingCompliance * invSdt2;

        float maxVelocity = 0.2f * particleThickness / sdt;
        if (handleSelfCollisions)
        {
            spatialHash.Create(positions, n);
            float maxTravelDistance = Mathf.Max(particleThickness, maxVelocity * Time.fixedDeltaTime + particleThickness);
            spatialHash.QueryAll(positions, n, maxTravelDistance);
        }

        for (int step = 0; step < numSubsteps; step++)
        {
            float maxAllowedVelocity = 0.2f * particleThickness / sdt;

            for (int i = 0; i < n; i++)
            {
                if (invMasses[i] == 0f)
                    continue;

                velocities[i] += gravity * sdt * invMasses[i];
                float v = velocities[i].magnitude;
                if (v > maxAllowedVelocity)
                    velocities[i] *= maxAllowedVelocity / v;

                predictedPositions[i] = positions[i];
                positions[i] += velocities[i] * sdt;
            }

            // Solve stretching
            foreach (DistanceConstraint constraint in stretchingConstraints)
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
                float s = -C / (w + alphaStretch);

                positions[id0] += grad * (s * w0);
                positions[id1] -= grad * (s * w1);
            }

            // Solve bending
            foreach (BendingConstraint constraint in bendingConstraints)
            {
                int id0 = constraint.p3Idx;
                int id1 = constraint.p4Idx;
                float w0 = invMasses[id0];
                float w1 = invMasses[id1];
                float w = w0 + w1;
                if (w == 0f) continue;

                Vector3 grad = positions[id0] - positions[id1];
                float len = grad.magnitude;
                if (len == 0f) continue;

                grad /= len;
                float C = len - constraint.restLength;
                float s = -C / (w + alphaBend);

                positions[id0] += grad * (s * w0);
                positions[id1] -= grad * (s * w1);
            }

            if (handleSelfCollisions)
                SolveSelfCollisions();

            for (int i = 0; i < n; i++)
            {
                if (invMasses[i] == 0f) continue;
                velocities[i] = (positions[i] - predictedPositions[i]) / sdt;
            }
        }

        // update mesh vertices
        for (int i = 0; i < n; i++)
            renderVertices[i] = tr.InverseTransformPoint(positions[i]); // TODO renderVertices might be used wrong

        mesh.SetVertices(renderVertices);
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }


    private void SolveSelfCollisions()
    {
        float thickness2 = particleThickness * particleThickness;

        for (int id0 = 0; id0 < n; id0++)
        {
            if (invMasses[id0] == 0f)
                continue;

            int first = spatialHash.firstAdjId[id0];
            int last = spatialHash.firstAdjId[id0 + 1];

            for (int j = first; j < last; j++)
            {
                int id1 = spatialHash.adjIds[j];
                if (invMasses[id1] == 0f)
                    continue;

                Vector3 delta = positions[id1] - positions[id0];
                float dist2 = delta.sqrMagnitude;

                if (dist2 == 0f || dist2 > thickness2)
                    continue;

                float restDist2 = (restPositions[id0] - restPositions[id1]).sqrMagnitude;
                if (dist2 > restDist2)
                    continue;

                float minDist = particleThickness;
                if (restDist2 < thickness2)
                    minDist = Mathf.Sqrt(restDist2);

                float dist = Mathf.Sqrt(dist2);
                Vector3 correction = delta * ((minDist - dist) / dist);

                positions[id0] -= 0.5f * correction;
                positions[id1] += 0.5f * correction;

                if (clothClothFriction > 0f)
                {
                    Vector3 v0 = positions[id0] - predictedPositions[id0];
                    Vector3 v1 = positions[id1] - predictedPositions[id1];
                    Vector3 avg = 0.5f * (v0 + v1);

                    positions[id0] += (avg - v0) * clothClothFriction;
                    positions[id1] += (avg - v1) * clothClothFriction;
                }
            }
        }
    }

    private void InitConstraints()
    {
        var stretchingList = new List<DistanceConstraint>();
        var bendingList = new List<BendingConstraint>();

        // Dictionary to track edges and find adjacent triangles
        // Key: edge identifier, Value: the third vertex of the triangle
        var edges = new Dictionary<long, int>();

        int[] triangles = mesh.triangles;

        for (int i = 0; i < triangles.Length; i += 3)
        {
            int v1 = triangles[i];
            int v2 = triangles[i + 1];
            int v3 = triangles[i + 2];

            AddEdgeAndBending(v1, v2, v3, stretchingList, bendingList, edges);
            AddEdgeAndBending(v2, v3, v1, stretchingList, bendingList, edges);
            AddEdgeAndBending(v3, v1, v2, stretchingList, bendingList, edges);
        }

        stretchingConstraints = stretchingList.ToArray();
        bendingConstraints = bendingList.ToArray();
    }

    private void AddEdgeAndBending(int v1, int v2, int oppositeVertex, List<DistanceConstraint> stretchingList, List<BendingConstraint> bendingList, Dictionary<long, int> edges)
    {
        int min = Mathf.Min(v1, v2);
        int max = Mathf.Max(v1, v2);
        long edgeKey = ((long)min << 32) | (uint)max;

        if (edges.TryGetValue(edgeKey, out int otherOppositeVertex))
        {
            // Edge already exists, so we found an adjacent triangle!
            // Create a bending constraint between the two opposite vertices (XPBD style distance constraint)
            float restLen = Vector3.Distance(positions[oppositeVertex], positions[otherOppositeVertex]);
            bendingList.Add(new BendingConstraint(v1, v2, oppositeVertex, otherOppositeVertex, restLen));
        }
        else
        {
            // First time seeing this edge, add a stretching constraint
            edges.Add(edgeKey, oppositeVertex);
            float stretchDist = Vector3.Distance(positions[min], positions[max]);
            stretchingList.Add(new DistanceConstraint(min, max, stretchDist));
        }
    }

    private sealed class SpatialHash
    {
        private readonly float spacing;
        private readonly int tableSize;
        private readonly int[] cellStart;
        private readonly int[] cellEntries;
        private readonly int[] queryIds;
        private int querySize;

        public readonly int[] firstAdjId;
        public readonly List<int> adjIds;

        public SpatialHash(float spacing, int maxNumObjects)
        {
            this.spacing = spacing;
            tableSize = Mathf.Max(1, 5 * maxNumObjects);
            cellStart = new int[tableSize + 1];
            cellEntries = new int[maxNumObjects];
            queryIds = new int[maxNumObjects];
            firstAdjId = new int[maxNumObjects + 1];
            adjIds = new List<int>(10 * maxNumObjects);
        }

        private int HashCoords(int xi, int yi, int zi)
        {
            int h = (xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481);
            if (h < 0)
                h = -h;
            return h % tableSize;
        }

        private int IntCoord(float coord) => Mathf.FloorToInt(coord / spacing);

        private int HashPos(Vector3[] pos, int id)
        {
            Vector3 p = pos[id];
            return HashCoords(IntCoord(p.x), IntCoord(p.y), IntCoord(p.z));
        }

        public void Create(Vector3[] pos, int count)
        {
            Array.Clear(cellStart, 0, cellStart.Length);
            Array.Clear(cellEntries, 0, cellEntries.Length);

            for (int i = 0; i < count; i++)
            {
                int h = HashPos(pos, i);
                cellStart[h]++;
            }

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

        private void Query(Vector3[] pos, int nr, float maxDist)
        {
            Vector3 p = pos[nr];
            int x0 = IntCoord(p.x - maxDist);
            int y0 = IntCoord(p.y - maxDist);
            int z0 = IntCoord(p.z - maxDist);

            int x1 = IntCoord(p.x + maxDist);
            int y1 = IntCoord(p.y + maxDist);
            int z1 = IntCoord(p.z + maxDist);

            querySize = 0;

            for (int xi = x0; xi <= x1; xi++)
            {
                for (int yi = y0; yi <= y1; yi++)
                {
                    for (int zi = z0; zi <= z1; zi++)
                    {
                        int h = HashCoords(xi, yi, zi);
                        int start = cellStart[h];
                        int end = cellStart[h + 1];

                        for (int i = start; i < end; i++)
                            queryIds[querySize++] = cellEntries[i];
                    }
                }
            }
        }

        public void QueryAll(Vector3[] pos, int count, float maxDist)
        {
            adjIds.Clear();
            float maxDist2 = maxDist * maxDist;

            for (int i = 0; i < count; i++)
            {
                firstAdjId[i] = adjIds.Count;
                Query(pos, i, maxDist);

                for (int j = 0; j < querySize; j++)
                {
                    int id1 = queryIds[j];
                    if (id1 >= i)
                        continue;

                    if ((pos[i] - pos[id1]).sqrMagnitude > maxDist2)
                        continue;

                    adjIds.Add(id1);
                }
            }

            firstAdjId[count] = adjIds.Count;
        }
    }
}