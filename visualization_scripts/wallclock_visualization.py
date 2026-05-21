import csv
from collections import defaultdict

import matplotlib.pyplot as plt

# Path to the summary CSV written by WallClockExperiment (per-(S,n) medians).
csv_path = r"..\Data\WallClock\WallClock_summary_20260521_164820.csv"

rows = defaultdict(list)
with open(csv_path, newline="") as f:
    reader = csv.DictReader(f, delimiter=";")
    for r in reader:
        key = (r["method"], int(r["substeps"]))
        rows[key].append(
            (
                int(r["substeps"]),
                int(r["iterations"]),
                float(r["medianMsPerStep"]),
                float(r["rmsError"]),
            )
        )

plt.figure(figsize=(9, 6))
for (method, substeps), pts in sorted(rows.items()):
    pts.sort(key=lambda p: p[2])  # by time
    xs = [p[2] for p in pts]
    ys = [p[3] for p in pts]
    c = "red" if "VBD" in method else "green" if "XPBD" in method else None
    plt.plot(xs, ys, "-o", label=f"{method} S{substeps}", markersize=5, alpha=0.8, color=c)
    for S, n, ms, err in pts:
        plt.annotate(f"S{S}xI{n}", (ms, err), fontsize=7,
                     textcoords="offset points", xytext=(4, 4))

plt.xscale("log")
plt.yscale("log")
plt.xlabel("median wallclock time per step (ms)")
plt.ylabel("RMS position error vs converged Newton")
plt.title("Chain solvers: accuracy vs wallclock time")
plt.grid(True, which="both", ls=":", alpha=0.5)
plt.legend()
plt.tight_layout()
plt.show()
