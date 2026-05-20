using System;
using UnityEngine;

/// <summary>
/// Ground-truth implicit-Euler solver for a mass-spring particle system (the chain).
///
/// Both XPBD and VBD are approximate minimizers of the variational form of implicit Euler:
///     G(x) = (1 / 2h^2) ||x - y||^2_M  +  E_elastic(x)
///     y    = x_t + h v + h^2 (g + f_ext / m)
/// This solver minimizes the SAME G(x) to a tight tolerance with Newton's method, so its
/// per-frame trajectory is a valid reference that XPBD and VBD can both be measured against.
///
/// How this differs from VBDSolver, which also "uses Newton's method":
///   - VBD takes ONE Newton step on a 3-DOF LOCAL system per vertex (block coordinate descent).
///   - This solver solves the FULL global 3N x 3N system, MULTIPLE iterations, to convergence.
///
/// Accuracy choices (cf. Macklin et al. 2016 Sec. 6, Chen et al. 2024 Sec. 5.4):
///   - All solver state and arithmetic are kept in double precision (Unity is float;
///     conversion happens only at the public interface).
///   - Each per-spring Hessian block is PSD-projected so the assembled global Hessian is
///     SPD and every Newton step is a guaranteed descent direction (projectHessianToPSD).
///   - A backtracking (Armijo) line search guarantees G decreases on every iteration..
///
/// Linear solves use a dense LDL^T factorization.
/// </summary>
public class NewtonSolver
{
    public int numVerts;
    public Vector3[] positions;
    public Vector3[] velocities;
    public float[] masses;
    public float[] invMasses;
    public Vector3[] externalForces;

    // Spring graph identical to VBD layout
    public VertexSpringEdge[] springEdges;
    public int[] springListStart;

    public Vector3 gravity = new Vector3(0, -9.81f, 0);

    // ---- reference configuration ----
    // numSubsteps = 1   -> one converged implicit-Euler step per frame. Use this as the
    //                      clean reference for residual-vs-iteration / per-step comparisons.
    // numSubsteps large -> implicit Euler approaches the true continuous ODE solution as
    //                      h -> 0. Use this (e.g. 100-1000) as the "true physics" reference
    //                      for measuring each method's physical-realism error, since XPBD
    //                      and VBD substep differently and do NOT approximate the same
    //                      single implicit-Euler step.
    public int numSubsteps = 1;
    public int maxIterations = 100;        // Newton iteration cap per substep

    public double absTolerance = 1e-9;     // converged when ||grad||_inf <= absTolerance ...
    public double relTolerance = 1e-10;    // ... or <= relTolerance * ||grad_0||_inf
    public bool useLineSearch = true;
    public bool projectHessianToPSD = true;

    public event Action OnPreSubstep;
    public event Action OnSubstep;

    // ---- internal double-precision state (authoritative), flattened to 3 * numVerts ----
    private int numDofs;
    private double[] positionsFlat, velocitiesFlat, previousPositionsFlat, inertia;
    private double[] grad, deltaX;     // gradient of G and the Newton step direction
    private double[,] hessian;         // 3N x 3N system matrix / LDL^T factor
    private double[] pivots;           // LDL^T diagonal
    private bool initialized;

    public NewtonSolver(int numVerts)
    {
        this.numVerts = numVerts;
        positions = new Vector3[numVerts];
        velocities = new Vector3[numVerts];
        masses = new float[numVerts];
        invMasses = new float[numVerts];
        externalForces = new Vector3[numVerts];
        Array.Fill(masses, 1f);
        Array.Fill(invMasses, 1f);
        springEdges = Array.Empty<VertexSpringEdge>();
        springListStart = new int[numVerts + 1];

        numDofs = numVerts * 3;
        positionsFlat = new double[numDofs];
        velocitiesFlat = new double[numDofs];
        previousPositionsFlat = new double[numDofs];
        inertia = new double[numDofs];
        grad = new double[numDofs];
        deltaX = new double[numDofs];
        hessian = new double[numDofs, numDofs];
        pivots = new double[numDofs];
    }

    private void ConvertToDouble()
    {
        for (int i = 0; i < numVerts; i++)
        {
            positionsFlat[3 * i + 0] = positions[i].x;
            positionsFlat[3 * i + 1] = positions[i].y;
            positionsFlat[3 * i + 2] = positions[i].z;
            velocitiesFlat[3 * i + 0] = velocities[i].x;
            velocitiesFlat[3 * i + 1] = velocities[i].y;
            velocitiesFlat[3 * i + 2] = velocities[i].z;
        }
    }

    private void ConvertToFloat()
    {
        for (int i = 0; i < numVerts; i++)
        {
            positions[i] = new Vector3((float)positionsFlat[3 * i], (float)positionsFlat[3 * i + 1], (float)positionsFlat[3 * i + 2]);
            velocities[i] = new Vector3((float)velocitiesFlat[3 * i], (float)velocitiesFlat[3 * i + 1], (float)velocitiesFlat[3 * i + 2]);
        }
    }

    private void ImportExternalChanges()
    {
        for (int i = 0; i < numVerts; i++)
        {
            for (int c = 0; c < 3; c++)
            {
                int r = 3 * i + c;
                if (positions[i][c] != (float)positionsFlat[r]) positionsFlat[r] = positions[i][c];
                if (velocities[i][c] != (float)velocitiesFlat[r]) velocitiesFlat[r] = velocities[i][c];
            }
        }
    }

    public void Step(float dt)
    {
        if (!initialized) { ConvertToDouble(); initialized = true; }

        double h = dt / Math.Max(1, numSubsteps);

        for (int s = 0; s < numSubsteps; s++)
        {
            Array.Clear(externalForces, 0, numVerts);
            OnPreSubstep?.Invoke();
            ImportExternalChanges();

            // Inertial prediction (inertia), and warm-start the Newton solve there.
            for (int i = 0; i < numVerts; i++)
            {
                previousPositionsFlat[3 * i + 0] = positionsFlat[3 * i + 0];
                previousPositionsFlat[3 * i + 1] = positionsFlat[3 * i + 1];
                previousPositionsFlat[3 * i + 2] = positionsFlat[3 * i + 2];

                if (invMasses[i] == 0f)   // pinned: inertia = x, stays put
                {
                    inertia[3 * i + 0] = positionsFlat[3 * i + 0];
                    inertia[3 * i + 1] = positionsFlat[3 * i + 1];
                    inertia[3 * i + 2] = positionsFlat[3 * i + 2];
                    continue;
                }

                Vector3 aExt = gravity + externalForces[i] * invMasses[i];
                inertia[3 * i + 0] = positionsFlat[3 * i + 0] + h * velocitiesFlat[3 * i + 0] + h * h * aExt.x;
                inertia[3 * i + 1] = positionsFlat[3 * i + 1] + h * velocitiesFlat[3 * i + 1] + h * h * aExt.y;
                inertia[3 * i + 2] = positionsFlat[3 * i + 2] + h * velocitiesFlat[3 * i + 2] + h * h * aExt.z;

                positionsFlat[3 * i + 0] = inertia[3 * i + 0];
                positionsFlat[3 * i + 1] = inertia[3 * i + 1];
                positionsFlat[3 * i + 2] = inertia[3 * i + 2];
            }

            SolveImplicitEulerStep(h);

            // Expose the solved positions so post-substep callbacks (e.g. collision response)
            // can read and correct them, then import their corrections so the velocity update
            // below reflects the final, corrected positions -- as in VBD / XPBD.
            ConvertToFloat();
            OnSubstep?.Invoke();
            ImportExternalChanges();

            // Backward-Euler velocity update: v = (x - x_prev) / h
            for (int i = 0; i < numVerts; i++)
            {
                if (invMasses[i] == 0f)
                {
                    velocitiesFlat[3 * i] = velocitiesFlat[3 * i + 1] = velocitiesFlat[3 * i + 2] = 0.0;
                    continue;
                }
                velocitiesFlat[3 * i + 0] = (positionsFlat[3 * i + 0] - previousPositionsFlat[3 * i + 0]) / h;
                velocitiesFlat[3 * i + 1] = (positionsFlat[3 * i + 1] - previousPositionsFlat[3 * i + 1]) / h;
                velocitiesFlat[3 * i + 2] = (positionsFlat[3 * i + 2] - previousPositionsFlat[3 * i + 2]) / h;
            }

            ConvertToFloat();
        }
    }

    // Solve argmin_x G(x) to a tight tolerance with Newton's method.
    private void SolveImplicitEulerStep(double h)
    {
        double invH2 = 1.0 / (h * h);
        double grad0Inf = 0.0;

        for (int iter = 0; iter < maxIterations; iter++)
        {
            BuildGradientAndHessian(invH2);

            double gInf = 0.0;
            for (int k = 0; k < numDofs; k++)
            {
                double a = Math.Abs(grad[k]);
                if (a > gInf) gInf = a;
            }
            if (iter == 0) grad0Inf = gInf;

            // Converged when ||grad||_inf is below the absolute floor or has shrunk by
            // the relative factor versus the initial residual.
            if (gInf <= absTolerance || gInf <= relTolerance * grad0Inf)
            {
                Debug.Log($"NewtonSolver: converged in {iter} iterations (|grad|_inf = {gInf:E2})");
                break;
            }

            // Solve hessian * deltaX = -grad. hessian is SPD when projectHessianToPSD is true.
            FactorizeLDLt();
            SolveLDLt();

            double t = useLineSearch ? LineSearch(invH2) : 1.0;
            for (int k = 0; k < numDofs; k++) positionsFlat[k] += t * deltaX[k];

            if (t == 0.0) break;   // line search could not make progress
        }
    }

    // grad = dG/dx ; hessian = d^2G/dx^2  (assembled global 3N x 3N system).
    private void BuildGradientAndHessian(double invH2)
    {
        Array.Clear(grad, 0, numDofs);
        Array.Clear(hessian, 0, hessian.Length);

        // Inertia term: (m / h^2)(x - y) for the gradient, (m / h^2) I for the Hessian.
        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] == 0f) continue;
            double mih2 = masses[i] * invH2;
            for (int c = 0; c < 3; c++)
            {
                int r = 3 * i + c;
                grad[r] += mih2 * (positionsFlat[r] - inertia[r]);
                hessian[r, r] += mih2;
            }
        }

        // Spring term. Each undirected spring is processed once (when i < otherIdx),
        // even though the CSR list stores it once per endpoint.
        for (int i = 0; i < numVerts; i++)
        {
            int start = springListStart[i];
            int end = springListStart[i + 1];
            for (int e = start; e < end; e++)
            {
                int j = springEdges[e].otherIdx;
                if (j <= i) continue;

                double k = springEdges[e].stiffness;
                double l0 = springEdges[e].restLength;

                double dxv = positionsFlat[3 * i + 0] - positionsFlat[3 * j + 0];
                double dyv = positionsFlat[3 * i + 1] - positionsFlat[3 * j + 1];
                double dzv = positionsFlat[3 * i + 2] - positionsFlat[3 * j + 2];
                double len2 = dxv * dxv + dyv * dyv + dzv * dzv;
                if (len2 < 1e-24) continue;

                double len = Math.Sqrt(len2);
                double invLen = 1.0 / len;
                double ux = dxv * invLen, uy = dyv * invLen, uz = dzv * invLen;

                // Spring elastic energy E = 0.5 k (len - l0)^2.
                // Gradient on vertex i: dE/dx_i = k (len - l0) u ; on j: the negative.
                double gscale = k * (len - l0);
                double gx = gscale * ux, gy = gscale * uy, gz = gscale * uz;
                grad[3 * i + 0] += gx; grad[3 * i + 1] += gy; grad[3 * i + 2] += gz;
                grad[3 * j + 0] -= gx; grad[3 * j + 1] -= gy; grad[3 * j + 2] -= gz;

                // Spring Hessian block S = c1 I + c2 u u^T.
                // c1 = k (1 - l0/len) is the geometric-stiffness term; under compression
                // (len < l0) it is negative, making S indefinite. PSD projection clamps
                // it to >= 0, guaranteeing the assembled hessian is SPD. The off-diagonal
                // block is -S, and [S, -S; -S, S] is PSD whenever S is.
                double ratio = l0 * invLen;
                double c1 = projectHessianToPSD ? k * Math.Max(0.0, 1.0 - ratio)
                                                : k * (1.0 - ratio);
                double c2 = k * ratio;

                double s00 = c1 + c2 * ux * ux;
                double s11 = c1 + c2 * uy * uy;
                double s22 = c1 + c2 * uz * uz;
                double s01 = c2 * ux * uy;
                double s02 = c2 * ux * uz;
                double s12 = c2 * uy * uz;

                AddBlock(3 * i, 3 * i, s00, s11, s22, s01, s02, s12, 1.0);
                AddBlock(3 * j, 3 * j, s00, s11, s22, s01, s02, s12, 1.0);
                AddBlock(3 * i, 3 * j, s00, s11, s22, s01, s02, s12, -1.0);
                AddBlock(3 * j, 3 * i, s00, s11, s22, s01, s02, s12, -1.0);
            }
        }

        // Pin constraint: force deltaX = 0 for pinned vertices by replacing their rows and
        // columns with the identity. The free-vertex equations are unaffected -- spring
        // forces from pinned neighbours are already in grad, evaluated at their fixed
        // positions, and the zeroed columns just drop a term that multiplies deltaX = 0.
        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] != 0f) continue;
            for (int c = 0; c < 3; c++)
            {
                int r = 3 * i + c;
                for (int q = 0; q < numDofs; q++) { hessian[r, q] = 0.0; hessian[q, r] = 0.0; }
                hessian[r, r] = 1.0;
                grad[r] = 0.0;
            }
        }
    }

    private void AddBlock(int r, int col,
                          double s00, double s11, double s22,
                          double s01, double s02, double s12, double sign)
    {
        hessian[r + 0, col + 0] += sign * s00; hessian[r + 0, col + 1] += sign * s01; hessian[r + 0, col + 2] += sign * s02;
        hessian[r + 1, col + 0] += sign * s01; hessian[r + 1, col + 1] += sign * s11; hessian[r + 1, col + 2] += sign * s12;
        hessian[r + 2, col + 0] += sign * s02; hessian[r + 2, col + 1] += sign * s12; hessian[r + 2, col + 2] += sign * s22;
    }

    // Dense LDL^T factorization, in place: L (unit lower-triangular) overwrites the lower
    // triangle of hessian, D is stored in pivots. Stable without pivoting when hessian is SPD,
    // which is guaranteed by the always-positive inertia term plus PSD-projected spring blocks.
    private void FactorizeLDLt()
    {
        const double pivotFloor = 1e-12;
        for (int j = 0; j < numDofs; j++)
        {
            double d = hessian[j, j];
            for (int k = 0; k < j; k++) d -= hessian[j, k] * hessian[j, k] * pivots[k];

            // Indefinite/singular guard. With projectHessianToPSD (default) this never
            // triggers. Without it, clamping the pivot keeps the solve usable; the line
            // search then ensures the (possibly imperfect) direction still reduces G.
            if (d < pivotFloor) d = pivotFloor;
            pivots[j] = d;

            for (int i = j + 1; i < numDofs; i++)
            {
                double sum = hessian[i, j];
                for (int k = 0; k < j; k++) sum -= hessian[i, k] * hessian[j, k] * pivots[k];
                hessian[i, j] = sum / d;
            }
        }
    }

    // Solve (L D L^T) deltaX = -grad using the factorization left in hessian / pivots.
    private void SolveLDLt()
    {
        // Forward: L y = -grad   (y reuses the deltaX buffer)
        for (int i = 0; i < numDofs; i++)
        {
            double sum = -grad[i];
            for (int k = 0; k < i; k++) sum -= hessian[i, k] * deltaX[k];
            deltaX[i] = sum;
        }
        // Diagonal: D z = y
        for (int i = 0; i < numDofs; i++) deltaX[i] /= pivots[i];
        // Back: L^T deltaX = z
        for (int i = numDofs - 1; i >= 0; i--)
        {
            double sum = deltaX[i];
            for (int k = i + 1; k < numDofs; k++) sum -= hessian[k, i] * deltaX[k];
            deltaX[i] = sum;
        }
    }

    // G(x + t * deltaX), evaluated without mutating positionsFlat.
    private double ComputeEnergy(double invH2, double t)
    {
        double G = 0.0;

        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] == 0f) continue;
            double mih2 = masses[i] * invH2;
            for (int c = 0; c < 3; c++)
            {
                int r = 3 * i + c;
                double d = (positionsFlat[r] + t * deltaX[r]) - inertia[r];
                G += 0.5 * mih2 * d * d;
            }
        }

        for (int i = 0; i < numVerts; i++)
        {
            int start = springListStart[i];
            int end = springListStart[i + 1];
            for (int e = start; e < end; e++)
            {
                int j = springEdges[e].otherIdx;
                if (j <= i) continue;
                double dxv = (positionsFlat[3 * i + 0] + t * deltaX[3 * i + 0]) - (positionsFlat[3 * j + 0] + t * deltaX[3 * j + 0]);
                double dyv = (positionsFlat[3 * i + 1] + t * deltaX[3 * i + 1]) - (positionsFlat[3 * j + 1] + t * deltaX[3 * j + 1]);
                double dzv = (positionsFlat[3 * i + 2] + t * deltaX[3 * i + 2]) - (positionsFlat[3 * j + 2] + t * deltaX[3 * j + 2]);
                double ext = Math.Sqrt(dxv * dxv + dyv * dyv + dzv * dzv) - springEdges[e].restLength;
                G += 0.5 * springEdges[e].stiffness * ext * ext;
            }
        }
        return G;
    }

    // Armijo backtracking line search. Returns the step length t in (0, 1], or 0 if no
    // decrease could be found.
    private double LineSearch(double invH2)
    {
        double g0 = ComputeEnergy(invH2, 0.0);

        double slope = 0.0;
        for (int k = 0; k < numDofs; k++) slope += grad[k] * deltaX[k];
        // With a PSD Hessian, deltaX is always a descent direction (slope < 0). If it is not
        // (only possible when projectHessianToPSD is false), fall back to plain monotone
        // backtracking by treating the slope as zero.
        if (slope >= 0.0) slope = 0.0;

        const double c = 1e-4;
        double t = 1.0;
        for (int b = 0; b < 40; b++)
        {
            if (ComputeEnergy(invH2, t) <= g0 + c * t * slope) return t;
            t *= 0.5;
        }
        return 0.0;
    }
}
