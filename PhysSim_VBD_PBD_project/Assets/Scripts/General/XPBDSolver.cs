using System;
using UnityEngine;

public class XPBDSolver
{
    public int numSubsteps = 15;
    public bool handleSelfCollisions = false;
    public float selfCollisionFriction = 0f;
    public float thickness;
    public Vector3 gravity = new Vector3(0, -9.81f, 0);

    public float velCapPerFrame = 3f; // Max per-frame travel, in multiples of `thickness`

    public int numVerts;
    public Vector3[] positions;
    public Vector3[] velocities;
    public Vector3[] previousPositions;
    public Vector3[] restPositions;
    public float[] invMasses;
    public DistanceConstraint[] constraints;
    public Vector3[] externalForces;

    private SpatialHash spatialHash;
    private float[] cachedSelfCollisionMinDist;

    public event Action OnPreSubstep;
    public event Action OnSubstep;

    public XPBDSolver(int numVerts)
    {
        this.numVerts = numVerts;
        positions = new Vector3[numVerts];
        velocities = new Vector3[numVerts];
        previousPositions = new Vector3[numVerts];
        restPositions = new Vector3[numVerts];
        invMasses = new float[numVerts];
        externalForces = new Vector3[numVerts];
        Array.Fill(invMasses, 1f);
        constraints = Array.Empty<DistanceConstraint>();
    }

    public void CreateSpatialHash(float spacing)
    {
        spatialHash = new SpatialHash(spacing, numVerts);
    }

    public void Step(float dt)
    {
        float sdt = dt / numSubsteps;
        float invSdt2 = 1.0f / (sdt * sdt);
        float maxVelocity = thickness > 0f ? velCapPerFrame * thickness / dt : float.PositiveInfinity;

        if (handleSelfCollisions)
        {
            spatialHash.Create(positions);
            float maxTravelDistance = thickness + maxVelocity * dt;
            spatialHash.QueryAll(positions, maxTravelDistance);
            CacheSelfCollisionMinDist();
        }

        for (int step = 0; step < numSubsteps; step++)
        {
            Array.Clear(externalForces, 0, numVerts);
            OnPreSubstep?.Invoke();
            Integrate(sdt, maxVelocity);
            SolveDistanceConstraints(invSdt2);
            if (handleSelfCollisions) SolveSelfCollisions();
            OnSubstep?.Invoke();
            UpdateVelocities(sdt);
        }
    }

    private void Integrate(float sdt, float maxVelocity)
    {
        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] == 0f) continue;
            velocities[i] += gravity * sdt;
            velocities[i] += externalForces[i] * sdt * invMasses[i];

            if (handleSelfCollisions)
            {
                float v = velocities[i].magnitude;
                if (v > maxVelocity)
                    velocities[i] *= maxVelocity / v;
            }

            previousPositions[i] = positions[i];
            positions[i] += velocities[i] * sdt;
        }
    }

    private void SolveDistanceConstraints(float invSdt2)
    {
        for (int i = 0; i < constraints.Length; i++)
        {
            ref var constraint = ref constraints[i];
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
    }

    private void UpdateVelocities(float sdt)
    {
        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] == 0f) continue;
            velocities[i] = (positions[i] - previousPositions[i]) / sdt;
        }
    }

    private void SolveSelfCollisions()
    {
        for (int id0 = 0; id0 < numVerts; id0++)
        {
            float w0 = invMasses[id0];
            if (w0 == 0f)
                continue;
            int first = spatialHash.firstAdjId[id0];
            int last = spatialHash.firstAdjId[id0 + 1];

            for (int j = first; j < last; j++)
            {
                int id1 = spatialHash.adjIds[j];
                float w1 = invMasses[id1];
                float w = w0 + w1;
                if (w == 0f) continue;

                float minDist = cachedSelfCollisionMinDist[j];
                Vector3 delta = positions[id0] - positions[id1];
                float dist2 = delta.sqrMagnitude;
                if (dist2 == 0f || dist2 >= minDist * minDist) continue;

                float dist = Mathf.Sqrt(dist2);
                Vector3 normal = delta / dist;
                Vector3 correction = normal * (minDist - dist);
                positions[id0] += correction * (w0 / w);
                positions[id1] -= correction * (w1 / w);

                // Friction logic: compute relative displacement of contact points over the substep
                Vector3 relDisp = (positions[id0] - previousPositions[id0]) -
                                  (positions[id1] - previousPositions[id1]);
                Vector3 dispNormal = Vector3.Dot(relDisp, normal) * normal;
                Vector3 dispTangent = relDisp - dispNormal;

                positions[id0] -= dispTangent * (selfCollisionFriction * w0 / w);
                positions[id1] += dispTangent * (selfCollisionFriction * w1 / w);
            }
        }
    }

    private void CacheSelfCollisionMinDist()
    {
        int total = spatialHash.firstAdjId[numVerts];
        if (cachedSelfCollisionMinDist == null || cachedSelfCollisionMinDist.Length < total)
            cachedSelfCollisionMinDist = new float[total];

        float thickness2 = thickness * thickness;
        for (int id0 = 0; id0 < numVerts; id0++)
        {
            int first = spatialHash.firstAdjId[id0];
            int last = spatialHash.firstAdjId[id0 + 1];
            for (int j = first; j < last; j++)
            {
                int id1 = spatialHash.adjIds[j];
                float restDist2 = (restPositions[id0] - restPositions[id1]).sqrMagnitude;
                float minDist2 = restDist2 < thickness2 ? restDist2 : thickness2;
                cachedSelfCollisionMinDist[j] = Mathf.Sqrt(minDist2);
            }
        }
    }
}
