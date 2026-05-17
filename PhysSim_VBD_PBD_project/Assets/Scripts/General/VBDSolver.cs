using System;
using UnityEngine;

public class VBDSolver
{
    public int numSubsteps = 1;
    public int numIterations = 15;
    public Vector3 gravity = new Vector3(0, -9.81f, 0);

    // Chebyshev semi-iterative acceleration (VBD paper)
    public bool useAcceleration = false;
    public float accelerationRho = 0.5f;

    public bool handleSelfCollisions = false;
    public float selfCollisionStiffness = 1e4f;
    public float thickness;

    bool hasPrevVelocities = false;

    private SpatialHash spatialHash;

    public int numVerts;
    public Vector3[] positions;
    public Vector3[] previousPosition;
    public Vector3[] velocities;
    public Vector3[] previousVelocities;
    public Vector3[] inertia;
    public Vector3[] prevprevPos;
    public Vector3[] prevIterPos;
    public Vector3[] restPositions;
    public float[] masses;
    public float[] invMasses;

    public bool[] isColliding;

    public Spring[] springs;
    public int[] springIds;
    public int[] springListStart;

    public event Action OnPreSubstep;
    public event Action OnSubstep;

    public delegate void VertexSolveDelegate(int i, Vector3 pos, ref Vector3 f, ref float h00, ref float h11, ref float h22, ref float h01, ref float h02, ref float h12);
    public event VertexSolveDelegate OnVertexSolve;

    public VBDSolver(int numVerts)
    {
        this.numVerts = numVerts;
        positions = new Vector3[numVerts];
        previousPosition = new Vector3[numVerts];
        velocities = new Vector3[numVerts];
        previousVelocities = new Vector3[numVerts];
        inertia = new Vector3[numVerts];
        prevprevPos = new Vector3[numVerts];
        prevIterPos = new Vector3[numVerts];
        restPositions = new Vector3[numVerts];
        masses = new float[numVerts];
        invMasses = new float[numVerts];
        isColliding = new bool[numVerts];
        Array.Fill(masses, 1f);
        Array.Fill(invMasses, 1f);
        springs = Array.Empty<Spring>();
        springIds = Array.Empty<int>();
        springListStart = new int[numVerts + 1];
    }

    public void CreateSpatialHash(float spacing)
    {
        spatialHash = new SpatialHash(spacing, numVerts);
    }

    public void Step(float dt)
    {
        float sdt = dt / numSubsteps;

        if (handleSelfCollisions)
        {
            spatialHash.Create(positions);
            // Search radius = collision distance + max per-frame motion. No
            // velocity clamping is applied; the contact energy itself resists
            // interpenetration during the implicit solve.
            float maxTravelDistance = thickness + MaxVelocityMagnitude() * dt;
            spatialHash.QueryAllSymmetric(positions, maxTravelDistance);
        }

        for (int step = 0; step < numSubsteps; step++)
        {
            OnPreSubstep?.Invoke(); // cache values for faster access in OnVertexSolve
            AdaptiveInitialization(sdt);

            float omega = 1f;
            for (int iter = 0; iter < numIterations; iter++)
            {
                Array.Copy(positions, prevIterPos, numVerts);
                Array.Clear(isColliding, 0, numVerts);

                Solve(sdt);

                if (useAcceleration)
                {
                    omega = GetAcceleratorOmega(iter + 1, accelerationRho, omega);
                    ApplyAccelerator(omega);
                    Array.Copy(prevIterPos, prevprevPos, numVerts);
                }
            }
            OnSubstep?.Invoke(); // TODO: for fixed stuff handling
            UpdateVelocity(sdt);
        }
    }

    private float MaxVelocityMagnitude()
    {
        float vMaxSq = 0f;
        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] == 0f) continue;
            float vSq = velocities[i].sqrMagnitude;
            if (vSq > vMaxSq) vMaxSq = vSq;
        }
        return Mathf.Sqrt(vMaxSq);
    }

    private void AdaptiveInitialization(float dt)
    {
        Vector3 gravDir = gravity.normalized;
        float gravMag = gravity.magnitude;
        float dt2 = dt * dt;
        Array.Copy(positions, previousPosition, numVerts);

        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] == 0f) continue;

            inertia[i] = previousPosition[i] + dt * velocities[i] + dt2 * gravity;
            float alpha = 1f;

            if (hasPrevVelocities && gravMag > 1e-10f)
            {
                Vector3 prevAcc = (velocities[i] - previousVelocities[i]) / dt;
                float extAcc = Vector3.Dot(prevAcc, gravDir);
                alpha = Mathf.Clamp01(extAcc / gravMag);

                positions[i] = previousPosition[i] + dt * velocities[i] + alpha * dt2 * gravity;
            }
            else
            {
                positions[i] = inertia[i];
            }
        }
    }

    // VBD solve - one Gauss-Seidel block-descent pass over all vertices
    private void Solve(float dt)
    {
        float invDt2 = 1f / (dt * dt);

        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] == 0f) continue;

            float massInvDt2 = masses[i] * invDt2;

            // f = m/dt^2 * (inertia - x)
            Vector3 f = (inertia[i] - positions[i]) * massInvDt2;

            // H = m/dt^2 * I - we only need Hessian is symmetric, because f_{xy} = f_{yx} for twice-continuously-differentiable functions
            float h00 = massInvDt2, h01 = 0f, h02 = 0f;
            float h11 = massInvDt2, h12 = 0f;
            float h22 = massInvDt2;

            // Spring contributions from all incident springs
            int start = springListStart[i];
            int end = springListStart[i + 1];
            for (int s = start; s < end; s++)
            {
                Spring spring = springs[springIds[s]];
                int v1 = spring.p1Idx;
                int v2 = spring.p2Idx;

                Vector3 diff = positions[v1] - positions[v2];
                float len = diff.magnitude;
                if (len < 1e-10f) continue;

                float l0 = spring.restLength;
                float k = spring.stiffness;
                float invL = 1f / len;
                Vector3 dir = diff * invL;
                float ratio = l0 * invL;

                // h_spring = k * ((1 - l0/l) I + (l0/l) d d^T) = coeff1 * I + coeff2 * d d^T
                float coeff1 = k * Mathf.Max(0f, 1f - ratio); // TODO:Max only needed if big dt + few substeps
                float coeff2 = k * ratio;

                h00 += coeff1 + coeff2 * dir.x * dir.x;
                h11 += coeff1 + coeff2 * dir.y * dir.y;
                h22 += coeff1 + coeff2 * dir.z * dir.z;
                h01 += coeff2 * dir.x * dir.y;
                h02 += coeff2 * dir.x * dir.z;
                h12 += coeff2 * dir.y * dir.z;

                // F_v1 = k (l0 - l)/l * diff,  F_v2 = -F_v1
                if (v1 == i) f += k * (l0 - len) * invL * diff;
                else f -= k * (l0 - len) * invL * diff;
            }

            // Self-collision as a contact energy E = 1/2 kc (minDist - d)^2.
            // Gradient/Hessian fold into the local Newton step the same way
            // springs do; no out-of-band position projection or velocity cap.
            if (handleSelfCollisions && selfCollisionStiffness > 0f)
            {
                float kc = selfCollisionStiffness;
                float thickness2 = thickness * thickness;
                int cStart = spatialHash.firstAdjId[i];
                int cEnd = spatialHash.firstAdjId[i + 1];

                for (int c = cStart; c < cEnd; c++)
                {
                    int idJ = spatialHash.adjIdsSym[c];
                    Vector3 diff = positions[i] - positions[idJ];
                    float dist2 = diff.sqrMagnitude;
                    if (dist2 < 1e-20f) continue;

                    // Effective contact distance: capped at the rest distance so
                    // adjacent grid neighbors aren't pushed apart to `thickness`.
                    float restDist2 = (restPositions[i] - restPositions[idJ]).sqrMagnitude;
                    float minDist2 = restDist2 < thickness2 ? restDist2 : thickness2;
                    if (dist2 >= minDist2) continue;

                    float dist = Mathf.Sqrt(dist2);
                    float minDist = Mathf.Sqrt(minDist2);
                    Vector3 n = diff * (1f / dist);

                    f += (kc * (minDist - dist)) * n;

                    // PSD rank-1 Hessian: kc * n n^T (full Hessian's -kc(r/d)(I-nn^T)
                    // term can go indefinite, so drop it for VBD's local solve).
                    h00 += kc * n.x * n.x;
                    h11 += kc * n.y * n.y;
                    h22 += kc * n.z * n.z;
                    h01 += kc * n.x * n.y;
                    h02 += kc * n.x * n.z;
                    h12 += kc * n.y * n.z;

                    isColliding[i] = true;
                }
            }

            OnVertexSolve?.Invoke(i, positions[i], ref f, ref h00, ref h11, ref h22, ref h01, ref h02, ref h12);

            Vector3 dx = SolveSymmetric3x3(h00, h11, h22, h01, h02, h12, f);

            positions[i] += dx;
        }
    }

    /// <summary>
    /// Solves H * x = f for symmetric 3x3 H by direct cofactor inversion.
    /// Returns the zero vector if H is (near-)singular.
    /// </summary>
    private Vector3 SolveSymmetric3x3(float h00, float h11, float h22,
                                      float h01, float h02, float h12,
                                      Vector3 f)
    {
        float det = h00 * (h11 * h22 - h12 * h12)
                  - h01 * (h01 * h22 - h12 * h02)
                  + h02 * (h01 * h12 - h11 * h02);

        if (Mathf.Abs(det) < 1e-20f) return Vector3.zero;
        float invDet = 1f / det;

        // Symmetric inverse (cofactor / det).
        float c00 = (h11 * h22 - h12 * h12) * invDet;
        float c11 = (h00 * h22 - h02 * h02) * invDet;
        float c22 = (h00 * h11 - h01 * h01) * invDet;
        float c01 = -(h01 * h22 - h12 * h02) * invDet;
        float c02 = (h01 * h12 - h11 * h02) * invDet;
        float c12 = -(h00 * h12 - h01 * h02) * invDet;

        return new Vector3(
            c00 * f.x + c01 * f.y + c02 * f.z,
            c01 * f.x + c11 * f.y + c12 * f.z,
            c02 * f.x + c12 * f.y + c22 * f.z
        );
    }

    private void UpdateVelocity(float dt)
    {
        Array.Copy(velocities, previousVelocities, numVerts);

        float invDt = 1f / dt;
        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] == 0f) { velocities[i] = Vector3.zero; continue; }
            velocities[i] = (positions[i] - previousPosition[i]) * invDt;
        }

        hasPrevVelocities = true;
    }

    // Chebyshev semi-iterative acceleration (VBD paper)
    private float GetAcceleratorOmega(int order, float rho, float prevOmega)
    {
        if (order == 1) return 1f;
        if (order == 2) return 2f / (2f - rho * rho);
        return 4f / (4f - rho * rho * prevOmega);
    }

    private void ApplyAccelerator(float omega)
    {
        if (omega <= 1f) return;
        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] == 0f || isColliding[i]) continue;
            positions[i] = omega * (positions[i] - prevprevPos[i]) + prevprevPos[i];
        }
    }

}
