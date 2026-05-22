using UnityEngine;

// Convergence diagnostic for the VBD local solve.
//   maxForce  - largest residual force magnitude over free vertices (N).
//   avgForce  - mean residual force magnitude over free vertices (N).
//   maxAccel  - largest residual acceleration = maxForce-vertex |f|/m (m/s^2).
//               Compare against g (9.81): >> g means badly under-converged,
//               << g means the solve has essentially reached force balance.
// The residual is the gradient of the incremental potential at the final positions;
// at the solve's fixed point it is zero. See VBDSolver.ComputeMaxResidual.
public struct ResidualSample
{
    public float maxForce;
    public float avgForce;
    public float maxAccel;
}

public static class ResidualSampler
{
    // The solver computes the residual during Step (when computeResidual is set), since
    // the evaluation needs the substep dt and the internal force assembly. This just
    // surfaces the cached result through the same Sampler API as the other loggers.
    public static ResidualSample Sample(VBDSolver s) => s.LastResidual;
}
