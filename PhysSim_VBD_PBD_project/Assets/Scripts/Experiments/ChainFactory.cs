using System.Collections.Generic;
using UnityEngine;

public struct ChainConfig
{
    public int numParticles;
    public float restLength;

    public float stretchingStiffness;

    // Optional per-spring stretching stiffness (length numParticles-1, spring i connects i and i+1).
    // When null, every spring uses the uniform stretchingStiffness above.
    public float[] stretchingStiffnessPerSpring;

    public bool hasBendingConstraints;
    public float bendingStiffness;

    public float particleMass; // mass of every interior particle
    public float endMass;      // mass of the last particle (the bob)

    public Vector3 start; // fixed first particle (pinned)
    public Vector3 bob;   // initial position of the last particle
    public Vector3 gravity;

    public float StretchingCompliance => 1f / stretchingStiffness;
    public float BendingCompliance => 1f / bendingStiffness;

    // Stiffness/compliance of a single stretching spring, honoring the per-spring override if set.
    public float StretchingStiffnessOf(int spring) =>
        (stretchingStiffnessPerSpring != null && spring < stretchingStiffnessPerSpring.Length)
            ? stretchingStiffnessPerSpring[spring]
            : stretchingStiffness;

    public float StretchingComplianceOf(int spring) => 1f / StretchingStiffnessOf(spring);

    public static ChainConfig Default(int numParticles)
    {
        float rest = 1f;
        return new ChainConfig
        {
            numParticles = numParticles,
            restLength = rest,
            stretchingStiffness = 1e6f,
            hasBendingConstraints = false,
            bendingStiffness = 1e5f,
            particleMass = 1f,
            endMass = 1f,
            start = Vector3.zero,
            bob = new Vector3(-(numParticles - 1) * rest, 0f, 0f),
            gravity = new Vector3(0f, -9.81f, 0f),
        };
    }

    // Chain whose stretching springs alternate soft, stiff, soft, stiff, ...
    // Spring 0 (and every even spring) uses softStiffness; every odd spring uses softStiffness * ratio.
    public static ChainConfig AlternatingStiffness(int numParticles, float softStiffness, float ratio = 10000f)
    {
        var cfg = Default(numParticles);
        cfg.stretchingStiffness = softStiffness;
        cfg.SetAlternatingStiffness(softStiffness, ratio);
        return cfg;
    }

    // Fill stretchingStiffnessPerSpring with the alternating soft/stiff pattern for this chain's size.
    public void SetAlternatingStiffness(float softStiffness, float ratio = 10000f)
    {
        int springs = Mathf.Max(0, numParticles - 1);
        stretchingStiffnessPerSpring = new float[springs];
        for (int i = 0; i < springs; i++)
            stretchingStiffnessPerSpring[i] = (i % 2 == 0) ? softStiffness : softStiffness * ratio;
    }
}

public static class ChainFactory
{
    private static Vector3[] InitialPositions(in ChainConfig cfg)
    {
        var pos = new Vector3[cfg.numParticles];
        Vector3 step = (cfg.bob - cfg.start) / Mathf.Max(1, cfg.numParticles - 1);
        for (int i = 0; i < cfg.numParticles; i++)
            pos[i] = cfg.start + step * i;
        return pos;
    }

    public static XPBDSolver BuildXPBD(in ChainConfig cfg)
    {
        int n = cfg.numParticles;
        var solver = new XPBDSolver(n)
        {
            handleSelfCollisions = false,
            thickness = cfg.restLength,
            gravity = cfg.gravity,
        };

        var pos = InitialPositions(cfg);
        for (int i = 0; i < n; i++) solver.positions[i] = pos[i];

        int constraintCount = n - 1 + (cfg.hasBendingConstraints ? n - 2 : 0);
        solver.constraints = new DistanceConstraint[constraintCount];
        for (int i = 0; i < n - 1; i++)
            solver.constraints[i] = new DistanceConstraint(i, i + 1, cfg.restLength, cfg.StretchingComplianceOf(i));
        if (cfg.hasBendingConstraints)
        {
            int b = n - 1;
            for (int i = 0; i < n - 2; i++)
                solver.constraints[b + i] = new DistanceConstraint(i, i + 2, 2f * cfg.restLength, cfg.BendingCompliance);
        }

        float invParticle = 1f / cfg.particleMass;
        for (int i = 0; i < n; i++) solver.invMasses[i] = invParticle;
        solver.invMasses[0] = 0f;
        solver.invMasses[n - 1] = cfg.endMass <= 0f ? 0f : 1f / cfg.endMass;

        return solver;
    }

    public static VBDSolver BuildVBD(in ChainConfig cfg)
    {
        int n = cfg.numParticles;
        var solver = new VBDSolver(n)
        {
            handleSelfCollisions = false,
            thickness = cfg.restLength,
            gravity = cfg.gravity,
        };

        var pos = InitialPositions(cfg);
        for (int i = 0; i < n; i++) solver.positions[i] = pos[i];

        BuildSpringCsr(cfg, out solver.springListStart, out solver.springEdges);
        SetMasses(cfg, solver.masses, solver.invMasses);
        return solver;
    }

    public static NewtonSolver BuildNewton(in ChainConfig cfg)
    {
        int n = cfg.numParticles;
        var solver = new NewtonSolver(n) { gravity = cfg.gravity };

        var pos = InitialPositions(cfg);
        for (int i = 0; i < n; i++) solver.positions[i] = pos[i];

        BuildSpringCsr(cfg, out solver.springListStart, out solver.springEdges);
        SetMasses(cfg, solver.masses, solver.invMasses);
        return solver;
    }

    private static void BuildSpringCsr(in ChainConfig cfg, out int[] springListStart, out VertexSpringEdge[] springEdges)
    {
        int n = cfg.numParticles;
        var perVert = new List<VertexSpringEdge>[n];
        for (int i = 0; i < n; i++) perVert[i] = new List<VertexSpringEdge>();

        for (int i = 0; i < n - 1; i++)
        {
            float k = cfg.StretchingStiffnessOf(i);
            perVert[i].Add(new VertexSpringEdge { otherIdx = i + 1, restLength = cfg.restLength, stiffness = k });
            perVert[i + 1].Add(new VertexSpringEdge { otherIdx = i, restLength = cfg.restLength, stiffness = k });
        }
        if (cfg.hasBendingConstraints)
        {
            for (int i = 0; i < n - 2; i++)
            {
                perVert[i].Add(new VertexSpringEdge { otherIdx = i + 2, restLength = 2f * cfg.restLength, stiffness = cfg.bendingStiffness });
                perVert[i + 2].Add(new VertexSpringEdge { otherIdx = i, restLength = 2f * cfg.restLength, stiffness = cfg.bendingStiffness });
            }
        }

        springListStart = new int[n + 1];
        int total = 0;
        for (int i = 0; i < n; i++) { springListStart[i] = total; total += perVert[i].Count; }
        springListStart[n] = total;

        springEdges = new VertexSpringEdge[total];
        for (int i = 0; i < n; i++)
        {
            int s = springListStart[i];
            var list = perVert[i];
            for (int j = 0; j < list.Count; j++) springEdges[s + j] = list[j];
        }
    }

    private static void SetMasses(in ChainConfig cfg, float[] masses, float[] invMasses)
    {
        int n = cfg.numParticles;
        for (int i = 0; i < n; i++)
        {
            masses[i] = cfg.particleMass;
            invMasses[i] = 1f / cfg.particleMass;
        }
        invMasses[0] = 0f;

        int last = n - 1;
        if (cfg.endMass <= 0f) { invMasses[last] = 0f; }
        else { masses[last] = cfg.endMass; invMasses[last] = 1f / cfg.endMass; }
    }
}
