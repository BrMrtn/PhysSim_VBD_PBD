using System;
using UnityEngine;

public class VBDSolver
{
    public int numSubsteps = 1;
    public int numIterations = 15;

    // Chebyshev semi-iterative acceleration (VBD paper)
    public bool useAcceleration = false;
    public float accelerationRho = 0.5f;

    public bool handleSelfCollisions = false;
    public float selfCollisionStiffness = 1e7f;
    public float thickness;
    public Vector3 gravity = new Vector3(0, -9.81f, 0);

    public float velCapPerFrame = 3f; // Max per-frame travel, in multiples of `thickness`

    // Rayleigh damping C = alpha*M + beta*K, treated implicitly in the local Newton solve.
    public float rayleighMassDamping = 0f;
    public float rayleighStiffnessDamping = 0f;

    public int numVerts;
    public Vector3[] positions;
    public Vector3[] velocities;
    public Vector3[] previousPositions;
    public Vector3[] previousVelocities;
    public Vector3[] restPositions;
    public Vector3[] inertia;
    public Vector3[] prevIterPositions;
    public Vector3[] prevPrevIterPositions;
    public float[] masses;
    public float[] invMasses;
    public bool[] isColliding;
    public VertexSpringEdge[] springEdges;
    public int[] springListStart;
    public Vector3[] externalForces;

    private SpatialHash spatialHash;
    private float[] cachedSelfCollisionMinDist;
    private bool hasPrevVelocities;

    public event Action OnPreSubstep;
    public event Action OnSubstep;

    public delegate void VertexSolveDelegate(int i, Vector3 pos, ref Vector3 f, ref float h00, ref float h11, ref float h22, ref float h01, ref float h02, ref float h12);
    public event VertexSolveDelegate OnVertexSolve;

    public VBDSolver(int numVerts)
    {
        this.numVerts = numVerts;
        positions = new Vector3[numVerts];
        previousPositions = new Vector3[numVerts];
        velocities = new Vector3[numVerts];
        previousVelocities = new Vector3[numVerts];
        inertia = new Vector3[numVerts];
        prevPrevIterPositions = new Vector3[numVerts];
        prevIterPositions = new Vector3[numVerts];
        restPositions = new Vector3[numVerts];
        masses = new float[numVerts];
        invMasses = new float[numVerts];
        isColliding = new bool[numVerts];
        Array.Fill(masses, 1f);
        Array.Fill(invMasses, 1f);
        springEdges = Array.Empty<VertexSpringEdge>();
        springListStart = new int[numVerts + 1];
        externalForces = new Vector3[numVerts];
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
            float velCap = velCapPerFrame * thickness / dt;
            float maxVel = MaxVelocityMagnitude();
            if (maxVel > velCap) maxVel = velCap;
            float maxTravelDistance = thickness + maxVel * dt;
            spatialHash.QueryAllSymmetric(positions, maxTravelDistance);
            CacheSelfCollisionMinDist();
        }

        for (int step = 0; step < numSubsteps; step++)
        {
            Array.Clear(externalForces, 0, numVerts);
            OnPreSubstep?.Invoke();
            AdaptiveInitialization(sdt);

            Array.Clear(isColliding, 0, numVerts);

            float omega = 1f;
            for (int iter = 0; iter < numIterations; iter++)
            {
                if (useAcceleration) Array.Copy(positions, prevIterPositions, numVerts);

                Solve(sdt);

                if (useAcceleration)
                {
                    omega = GetAcceleratorOmega(iter + 1, accelerationRho, omega);
                    ApplyAccelerator(omega);
                    Array.Copy(prevIterPositions, prevPrevIterPositions, numVerts);
                }
            }
            OnSubstep?.Invoke();
            UpdateVelocities(sdt);
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

    private void CacheSelfCollisionMinDist()
    {
        int total = spatialHash.firstAdjId[numVerts];
        if (cachedSelfCollisionMinDist == null || cachedSelfCollisionMinDist.Length < total)
            cachedSelfCollisionMinDist = new float[total];

        float thickness2 = thickness * thickness;
        for (int i = 0; i < numVerts; i++)
        {
            int start = spatialHash.firstAdjId[i];
            int end = spatialHash.firstAdjId[i + 1];
            for (int c = start; c < end; c++)
            {
                int j = spatialHash.adjIdsSym[c];
                float restDist2 = (restPositions[i] - restPositions[j]).sqrMagnitude;
                float minDist2 = restDist2 < thickness2 ? restDist2 : thickness2;
                cachedSelfCollisionMinDist[c] = Mathf.Sqrt(minDist2);
            }
        }
    }

    private void AdaptiveInitialization(float dt)
    {
        float dt2 = dt * dt;
        Array.Copy(positions, previousPositions, numVerts);

        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] == 0f) continue;

            Vector3 aExt = gravity + externalForces[i] * invMasses[i];
            inertia[i] = previousPositions[i] + dt * velocities[i] + dt2 * aExt;

            float alpha = 1f;
            if (hasPrevVelocities)
            {
                float aMag2 = aExt.sqrMagnitude;
                if (aMag2 > 1e-20f)
                {
                    Vector3 prevAcc = (velocities[i] - previousVelocities[i]) / dt;
                    alpha = Mathf.Clamp01(Vector3.Dot(prevAcc, aExt) / aMag2);
                }
            }

            positions[i] = previousPositions[i] + dt * velocities[i] + alpha * dt2 * aExt;
        }
    }

    // VBD solve - one Gauss-Seidel block-descent pass over all vertices
    private void Solve(float dt)
    {
        float invDt2 = 1f / (dt * dt);
        float invDt = 1f / dt;
        bool hasMassDamping = rayleighMassDamping > 0f;
        bool hasStiffDamping = rayleighStiffnessDamping > 0f;
        float alphaOverDt = rayleighMassDamping * invDt;
        float betaOverDt = rayleighStiffnessDamping * invDt;

        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] == 0f) continue;

            float massInvDt2 = masses[i] * invDt2;

            // f = m/dt^2 * (inertia - x). F_ext already included in inertia[i]
            Vector3 f = (inertia[i] - positions[i]) * massInvDt2;

            // H = m/dt^2 * I - we know Hessian is symmetric, because f_{xy} = f_{yx} for twice-continuously-differentiable functions
            float h00 = massInvDt2, h01 = 0f, h02 = 0f;
            float h11 = massInvDt2, h12 = 0f;
            float h22 = massInvDt2;

            // Mass-proportional Rayleigh damping (implicit): adds (alpha*m/dt) I to H
            // and -(alpha*m/dt)(x_i - x_old_i) to f, equivalent to -alpha*m*v_i.
            if (hasMassDamping)
            {
                float cMass = alphaOverDt * masses[i];
                Vector3 dxi = positions[i] - previousPositions[i];
                f -= cMass * dxi;
                h00 += cMass;
                h11 += cMass;
                h22 += cMass;
            }

            // Spring contributions from all incident springs.
            Vector3 pi = positions[i];
            float pix = pi.x, piy = pi.y, piz = pi.z;
            float ppix = 0f, ppiy = 0f, ppiz = 0f;
            if (hasStiffDamping)
            {
                Vector3 ppi = previousPositions[i];
                ppix = ppi.x; ppiy = ppi.y; ppiz = ppi.z;
            }

            int start = springListStart[i];
            int end = springListStart[i + 1];
            for (int s = start; s < end; s++)
            {
                VertexSpringEdge edge = springEdges[s];
                int j = edge.otherIdx;
                Vector3 pj = positions[j];

                float diffx = pix - pj.x;
                float diffy = piy - pj.y;
                float diffz = piz - pj.z;
                float len2 = diffx * diffx + diffy * diffy + diffz * diffz;
                if (len2 < 1e-20f) continue;
                float len = Mathf.Sqrt(len2);

                float l0 = edge.restLength;
                float k = edge.stiffness;
                float invL = 1f / len;
                float dirx = diffx * invL;
                float diry = diffy * invL;
                float dirz = diffz * invL;
                float ratio = l0 * invL;

                // h_spring = k * ((1 - l0/l) I + (l0/l) d d^T) = coeff1 * I + coeff2 * d d^T
                float coeff1 = k * Mathf.Max(0f, 1f - ratio); // Max only needed if big dt + few substeps
                float coeff2 = k * ratio;

                // Stiffness-proportional Rayleigh damping (implicit): for spring (i,j),
                // local K_ii = coeff1 I + coeff2 d d^T, K_ij = -K_ii. Adds
                // -(beta/dt) K_ii ((x_i - x_old_i) - (x_j - x_old_j)) to f, and
                // (beta/dt) K_ii to the (i,i) Hessian block.
                if (hasStiffDamping)
                {
                    Vector3 ppj = previousPositions[j];
                    float dvx = (pix - ppix) - (pj.x - ppj.x);
                    float dvy = (piy - ppiy) - (pj.y - ppj.y);
                    float dvz = (piz - ppiz) - (pj.z - ppj.z);
                    float dDotDv = dirx * dvx + diry * dvy + dirz * dvz;
                    float c2dot = coeff2 * dDotDv;
                    f.x -= betaOverDt * (coeff1 * dvx + c2dot * dirx);
                    f.y -= betaOverDt * (coeff1 * dvy + c2dot * diry);
                    f.z -= betaOverDt * (coeff1 * dvz + c2dot * dirz);
                    float scale = 1f + betaOverDt;
                    coeff1 *= scale;
                    coeff2 *= scale;
                }

                h00 += coeff1 + coeff2 * dirx * dirx;
                h11 += coeff1 + coeff2 * diry * diry;
                h22 += coeff1 + coeff2 * dirz * dirz;
                h01 += coeff2 * dirx * diry;
                h02 += coeff2 * dirx * dirz;
                h12 += coeff2 * diry * dirz;

                float fScale = k * (l0 - len) * invL;
                f.x += fScale * diffx;
                f.y += fScale * diffy;
                f.z += fScale * diffz;
            }

            if (handleSelfCollisions && selfCollisionStiffness > 0f)
            {
                float kc = selfCollisionStiffness;
                int cStart = spatialHash.firstAdjId[i];
                int cEnd = spatialHash.firstAdjId[i + 1];

                for (int c = cStart; c < cEnd; c++)
                {
                    int idJ = spatialHash.adjIdsSym[c];
                    Vector3 diff = positions[i] - positions[idJ];
                    float dist2 = diff.sqrMagnitude;
                    if (dist2 < 1e-20f) continue;

                    float minDist = cachedSelfCollisionMinDist[c];
                    if (dist2 >= minDist * minDist) continue;

                    float dist = Mathf.Sqrt(dist2);
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

    private void UpdateVelocities(float dt)
    {
        Array.Copy(velocities, previousVelocities, numVerts);

        float invDt = 1f / dt;

        float dtFrame = dt * numSubsteps;
        float velCap = handleSelfCollisions ? velCapPerFrame * thickness / dtFrame : float.PositiveInfinity;
        float velCap2 = velCap * velCap;

        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] == 0f) { velocities[i] = Vector3.zero; continue; }
            Vector3 v = (positions[i] - previousPositions[i]) * invDt;
            if (handleSelfCollisions)
            {
                float vSq = v.sqrMagnitude;
                if (vSq > velCap2)
                    v *= velCap / Mathf.Sqrt(vSq);
            }
            velocities[i] = v;
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
            positions[i] = omega * (positions[i] - prevPrevIterPositions[i]) + prevPrevIterPositions[i];
        }
    }

}
