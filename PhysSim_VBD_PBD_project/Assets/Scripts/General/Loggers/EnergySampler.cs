using UnityEngine;

public struct EnergySample
{
    public float kinetic;
    public float gravitational; // potential
    public float elastic; // potential
    public float Total => kinetic + gravitational + elastic;
}

// Snapshots the mechanical energy of a solver's current state.
// KE = sum 0.5 m_i |v_i|^2; PE_grav = -sum m_i (g . x_i); PE_elastic = sum 0.5 k (l - l0)^2.
// Pinned vertices (invMass == 0) are skipped so they don't add a constant offset.
//
// These solvers are variational implicit (backward) Euler: x^{n+1} is solved for, then
// v^{n+1} = (x^{n+1} - x^n)/dt. Velocity and position are co-located at time n+1 (this is
// not a staggered/leapfrog scheme), so KE and PE must both be sampled from the n+1 state:
// KE from velocities and PE from positions. Averaging PE onto the n+1/2 midpoint while KE
// stays at n+1 puts the two halves half a step out of phase and injects a spurious
// oscillation into the total, so we sample positions directly.
public static class EnergySampler
{
    public static EnergySample Sample(XPBDSolver s)
    {
        EnergySample r = default;
        for (int i = 0; i < s.numVerts; i++)
        {
            if (s.invMasses[i] == 0f) continue;
            float m = 1f / s.invMasses[i];
            r.kinetic += 0.5f * m * s.velocities[i].sqrMagnitude;
            r.gravitational -= m * Vector3.Dot(s.gravity, s.positions[i]);
        }

        var cons = s.constraints;
        if (cons != null)
        {
            for (int i = 0; i < cons.Length; i++)
            {
                ref var con = ref cons[i];
                // compliance == 0 is a hard constraint (k -> inf); no elastic PE when satisfied
                if (con.compliance <= 0f) continue;
                float k = 1f / con.compliance;
                float ext = (s.positions[con.p1Idx] - s.positions[con.p2Idx]).magnitude - con.restLength;
                r.elastic += 0.5f * k * ext * ext;
            }
        }

        return r;
    }

    public static EnergySample Sample(VBDSolver s)
    {
        EnergySample r = default;
        for (int i = 0; i < s.numVerts; i++)
        {
            if (s.invMasses[i] == 0f) continue;
            r.kinetic += 0.5f * s.masses[i] * s.velocities[i].sqrMagnitude;
            r.gravitational -= s.masses[i] * Vector3.Dot(s.gravity, s.positions[i]);
        }

        // VBD's CSR stores each spring twice -> half value at the end
        float twiceElastic = 0f;
        for (int i = 0; i < s.numVerts; i++)
        {
            int start = s.springListStart[i];
            int end = s.springListStart[i + 1];
            for (int e = start; e < end; e++)
            {
                var edge = s.springEdges[e];
                int j = edge.otherIdx;
                float ext = (s.positions[i] - s.positions[j]).magnitude - edge.restLength;
                twiceElastic += 0.5f * edge.stiffness * ext * ext;
            }
        }
        r.elastic = 0.5f * twiceElastic;

        return r;
    }

    public static EnergySample Sample(NewtonSolver s)
    {
        EnergySample r = default;
        for (int i = 0; i < s.numVerts; i++)
        {
            if (s.invMasses[i] == 0f) continue;
            r.kinetic += 0.5f * s.masses[i] * s.velocities[i].sqrMagnitude;
            r.gravitational -= s.masses[i] * Vector3.Dot(s.gravity, s.positions[i]);
        }

        float twiceElastic = 0f;
        for (int i = 0; i < s.numVerts; i++)
        {
            int start = s.springListStart[i];
            int end = s.springListStart[i + 1];
            for (int e = start; e < end; e++)
            {
                var edge = s.springEdges[e];
                Vector3 d = s.positions[i] - s.positions[edge.otherIdx];
                float ext = d.magnitude - edge.restLength;
                twiceElastic += 0.5f * edge.stiffness * ext * ext;
            }
        }
        r.elastic = 0.5f * twiceElastic;

        return r;
    }
}
