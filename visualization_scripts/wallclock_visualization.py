import csv
from collections import defaultdict

import matplotlib.pyplot as plt

# Path to the summary CSV written by WallClockExperiment (per-(S,n) medians).
csv_path = r"..\Data\WallClock\WallClock_summary_20260521_120118.csv"

# Columns: method;numParticles;substeps;iterations;medianMsPerStep;rmsError;repeats
rows = defaultdict(list)
with open(csv_path, newline="") as f:
    reader = csv.DictReader(f, delimiter=";")
    for r in reader:
        rows[r["method"]].append(
            (
                int(r["substeps"]),
                int(r["iterations"]),
                float(r["medianMsPerStep"]),
                float(r["rmsError"]),
            )
        )

# Error (vs converged Newton) on y, ms/step on x. Both axes log: cost and error span
# orders of magnitude across the sweep.
plt.figure(figsize=(9, 6))
for method, pts in rows.items():
    pts.sort(key=lambda p: p[2])  # by cost, so the connecting line reads left-to-right
    xs = [p[2] for p in pts]
    ys = [p[3] for p in pts]
    plt.plot(xs, ys, "-o", label=method, markersize=5, alpha=0.8)
    for S, n, ms, err in pts:
        plt.annotate(f"{S}x{n}", (ms, err), fontsize=7,
                     textcoords="offset points", xytext=(4, 4))

plt.xscale("log")
plt.yscale("log")
plt.xlabel("ms / step (median)")
plt.ylabel("RMS position error vs converged Newton")
plt.title("Chain solvers: accuracy vs wall-clock cost")
plt.grid(True, which="both", ls=":", alpha=0.5)
plt.legend()
plt.tight_layout()
plt.show()
