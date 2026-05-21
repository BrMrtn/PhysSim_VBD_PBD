using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using UnityEngine;
using Debug = UnityEngine.Debug;

/// <summary>

/// For each method (XPBD, VBD) it sweeps a 2D grid of (numSubsteps S, numIterations n),
/// runs a fixed number of Step(dt) calls, and records:
///   - x-axis: median ms per step (System.Diagnostics.Stopwatch),
///   - y-axis: RMS position error against a single cached, converged Newton reference.
/// </summary>
public class WallClockExperiment : MonoBehaviour
{
    public bool testXPBD = true;
    public bool testVBD = true;

    public int numSteps = 120;
    public int repeats = 3;
    public int warmupSteps = 0;
    public float frameRate = 30f;

    public int[] substepsGrid = { 1, 2, 4, 8, 16, 32 };
    public int[] iterationsGrid = { 1, 2, 4, 8, 16, 32 };

    private float dt;

    [Header("Chain")]
    public int numParticles = 10;
    public float restLength = 1f;
    public float stretchingStiffness = 1e4f;
    public bool hasBendingConstraints = true;
    public float bendingStiffness = 1e3f;
    public float particleMass = 1f;
    public float endMass = 100f;

    [Header("Newton reference (true-physics ground truth)")]
    public int referenceNumSubsteps = 60; // many substeps approach continuous Newton
    public int referenceMaxIterations = 100;
    public double referenceAbsTolerance = 1e-11;
    public double referenceRelTolerance = 1e-12;

    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    void Start()
    {
        var totalTimer = Stopwatch.StartNew();

        dt = 1f / Mathf.Max(1f, frameRate);
        var cfg = BuildConfig();

        Debug.Log($"[WallClock] Computing Newton reference: {numSteps} steps x " +
                  $"{referenceNumSubsteps} substeps, {numParticles} particles...");
        var swRef = Stopwatch.StartNew();
        Vector3[][] reference = RunNewtonReference(cfg);
        swRef.Stop();
        Debug.Log($"[WallClock] Newton reference done in {swRef.Elapsed.TotalSeconds:F1}s.");

        string dir = Path.GetFullPath(Path.Combine(Application.dataPath, "../../Data/WallClock"));
        Directory.CreateDirectory(dir);
        string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");

        using var summary = new StreamWriter(Path.Combine(dir, $"WallClock_summary_{stamp}.csv"));
        // totalIterations = substeps * iterations = total inner solver sweeps per Step(); the
        // x-axis for the iterations-to-accuracy (work-normalized) plot, decoupled from CPU ms.
        summary.WriteLine("method;substeps;iterations;medianMsPerStep;rmsError;repeats;totalIterations");

        if (testXPBD)
            Sweep("XPBD", cfg, reference, summary,
                  (S, n) =>
                  {
                      var s = ChainFactory.BuildXPBD(cfg);
                      s.numSubsteps = S; s.numIterations = n;
                      return (() => s.Step(dt), s.positions);
                  });

        if (testVBD)
            Sweep("VBD", cfg, reference, summary,
                  (S, n) =>
                  {
                      var s = ChainFactory.BuildVBD(cfg);
                      s.numSubsteps = S; s.numIterations = n;
                      s.useAcceleration = false;
                      s.accelerationRho = 0.5f;
                      return (() => s.Step(dt), s.positions);
                  });

        summary.Flush();
        totalTimer.Stop();
        Debug.Log($"[WallClock] Done in {totalTimer.Elapsed.TotalSeconds:F2} seconds. CSVs written to {dir}");
    }

    private ChainConfig BuildConfig()
    {
        var cfg = ChainConfig.Default(numParticles);
        cfg.restLength = restLength;
        cfg.stretchingStiffness = stretchingStiffness;
        cfg.hasBendingConstraints = hasBendingConstraints;
        cfg.bendingStiffness = bendingStiffness;
        cfg.particleMass = particleMass;
        cfg.endMass = endMass;
        cfg.bob = new Vector3(-(numParticles - 1) * restLength -1f, 0f, 0f);
        return cfg;
    }

    // One converged Newton run; caches the full per-step position trajectory.
    private Vector3[][] RunNewtonReference(in ChainConfig cfg)
    {
        var solver = ChainFactory.BuildNewton(cfg);
        solver.numSubsteps = referenceNumSubsteps;
        solver.maxIterations = referenceMaxIterations;
        solver.absTolerance = referenceAbsTolerance;
        solver.relTolerance = referenceRelTolerance;
        solver.projectHessianToPSD = true;
        solver.logConvergence = false;

        var traj = new Vector3[numSteps][];
        for (int i = 0; i < numSteps; i++)
        {
            solver.Step(dt);
            traj[i] = (Vector3[])solver.positions.Clone();
        }
        return traj;
    }

    // builder(S, n) returns a fresh, fully-initialised solver as a (step closure, live
    // positions array) pair. We rebuild for every warmup and every timed rep so each run
    // starts from the identical initial state the reference was generated from.
    private void Sweep(string method, ChainConfig cfg, Vector3[][] reference,
                       StreamWriter summary,
                       Func<int, int, (Action step, Vector3[] positions)> builder)
    {
        var traj = new Vector3[numSteps][];

        foreach (int S in substepsGrid)
        foreach (int n in iterationsGrid)
        {
            var times = new List<double>(repeats);
            double err = double.NaN;

            for (int rep = 0; rep < repeats; rep++)
            {
                GC.Collect();
                GC.WaitForPendingFinalizers();
                GC.Collect();

                // Warmup on a throwaway solver (JIT + caches), then time a fresh one.
                var warm = builder(S, n);
                for (int i = 0; i < warmupSteps; i++) warm.step();

                var run = builder(S, n);
                var sw = Stopwatch.StartNew();
                for (int i = 0; i < numSteps; i++)
                {
                    run.step();
                    traj[i] = (Vector3[])run.positions.Clone();
                }
                sw.Stop();

                double msPerStep = sw.Elapsed.TotalMilliseconds / numSteps;
                times.Add(msPerStep);

                // Deterministic given identical initial state -> compute once.
                if (rep == 0) err = ComputeRmsError(traj, reference);
            }

            double median = Median(times);
            summary.WriteLine(string.Format(Inv, "{0};{1};{2};{3:F6};{4:E6};{5};{6}",
                method, S, n, median, err, repeats, S * n));
            Debug.Log(string.Format(Inv, "[WallClock] {0} S={1,2} n={2,2}: {3:F4} ms/step, RMS err {4:E3}",
                method, S, n, median, err));
        }
    }

    private double ComputeRmsError(Vector3[][] traj, Vector3[][] reference)
    {
        double sumSq = 0.0;
        long count = 0;
        for (int t = 0; t < numSteps; t++)
        {
            Vector3[] a = traj[t];
            Vector3[] b = reference[t];
            for (int i = 0; i < a.Length; i++)
            {
                double dx = a[i].x - b[i].x;
                double dy = a[i].y - b[i].y;
                double dz = a[i].z - b[i].z;
                sumSq += dx * dx + dy * dy + dz * dz;
                count++;
            }
        }
        return Math.Sqrt(sumSq / count);
    }

    private static double Median(List<double> values)
    {
        var sorted = new List<double>(values);
        sorted.Sort();
        int m = sorted.Count / 2;
        return sorted.Count % 2 == 1 ? sorted[m] : 0.5 * (sorted[m - 1] + sorted[m]);
    }
}
