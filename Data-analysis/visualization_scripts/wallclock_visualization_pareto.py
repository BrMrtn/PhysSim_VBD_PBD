import csv
import os
from collections import defaultdict

import matplotlib.pyplot as plt

# Path to the summary CSV written by WallClockExperiment (per-(S,n) medians).
csv_path = r"..\Logs\WallClock\WallClock_fair_base_newton128ss.csv"

rows = defaultdict(list)
with open(csv_path, newline="") as f:
    reader = csv.DictReader(f, delimiter=";")
    for r in reader:
        key = r["method"]
        rows[key].append(
            (
                int(r["substeps"]),
                int(r["iterations"]),
                float(r["medianMsPerStep"]),
                float(r["rmsError"]),
            )
        )

plt.figure(figsize=(9, 6))
for method, pts in sorted(rows.items()):
    pts.sort(key=lambda p: (p[2], p[3]))  # by time, then error
    
    pareto_pts = []
    min_err = float('inf')
    for p in pts:
        if p[3] < min_err:
            pareto_pts.append(p)
            min_err = p[3]

    c = "red" if "VBD" in method else "green" if "XPBD" in method else None
    
    # all_xs = [p[2] for p in pts]
    # all_ys = [p[3] for p in pts]
    # plt.plot(all_xs, all_ys, "o", markersize=4, alpha=0.4, color=c)

    xs = [p[2] for p in pareto_pts]
    ys = [p[3] for p in pareto_pts]
    plt.plot(xs, ys, "-o", label=f"{method} Pareto", markersize=5, alpha=0.8, color=c)
    
    for S, n, ms, err in pareto_pts:
        plt.annotate(f"S{S}xI{n}", (ms, err), fontsize=7,
                     textcoords="offset points", xytext=(4, 4))

plt.xscale("log")
plt.yscale("log")
plt.xlabel("ms/frame")
plt.ylabel("RMS position error vs converged Newton")
plt.title("Chain solvers: accuracy vs wallclock time")
plt.grid(True, which="both", ls=":", alpha=0.5)
plt.legend()
plt.tight_layout()

out_dir = r"..\Data\Visuals\WallClock"
os.makedirs(out_dir, exist_ok=True)
out_name = os.path.basename(csv_path).replace(".csv", ".png")
out_name = out_name.replace("WallClock_fair_base_newton", "WallClock_fair_base_clean_newton")
out_path = os.path.join(out_dir, out_name)
plt.savefig(out_path, dpi=300, bbox_inches="tight")
print(f"Saved plot to {out_path}")
