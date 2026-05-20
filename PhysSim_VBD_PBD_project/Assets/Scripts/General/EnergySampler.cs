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
                Vector3 d = s.positions[con.p1Idx] - s.positions[con.p2Idx];
                float ext = d.magnitude - con.restLength;
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
                Vector3 d = s.positions[i] - s.positions[edge.otherIdx];
                float ext = d.magnitude - edge.restLength;
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
