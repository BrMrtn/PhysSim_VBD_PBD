import csv
import glob
import math
import os

import matplotlib.pyplot as plt
import numpy as np

# Physical parameters of the single-spring oscillator (from the Unity scene).
STIFFNESS = 1000.0  # N/m
MASS = 1.0          # kg
# Undamped angular frequency. The continuous EOM m*u'' = -k*u has no damping;
# any decay in the solver curves comes from the implicit time discretization.
OMEGA = math.sqrt(STIFFNESS / MASS)


def solver_color(name):
    """Color a series by its solver name: XPBD->green, VBD->red, Newton->blue."""
    if name.startswith("XPBD"):
        return "green"
    if name.startswith("VBD"):
        return "red"
    if name.startswith("Newton"):
        return "blue"
    return None


# Directory of per-run CSVs written by SpringLengthLogger.cs.
# Each file is named "<label>_<timestamp>.csv" and holds a single-spring run.
log_dir = r"..\Logs\SpringLengthLogs"

paths = sorted(glob.glob(os.path.join(log_dir, "*.csv")))
if not paths:
    raise SystemExit(f"No CSV files found in {os.path.abspath(log_dir)}")

MAX_FRAMES = 200

plt.figure(figsize=(9, 6))
dt = None
offset0 = None
# Solver curves overlap almost exactly, so draw each successive one thinner
# than the last: the earlier (thicker) lines stay visible underneath.
base_lw = 8.0
lw_step = 3.0
for idx, path in enumerate(paths):
    frames = []
    offsets = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f, delimiter=";")
        prev_time = None
        for i, r in enumerate(reader):
            if i >= MAX_FRAMES:
                break
            # Single spring: the end hangs at y = -currentLength, and rests at
            # y = -restLength = -1. Offset from y = -1 is restLength - currentLength.
            offset = float(r["totalRest"]) - float(r["totalCurrent"])
            frames.append(i)
            offsets.append(offset)
            # Capture the per-frame timestep from the logged time column so the
            # analytical curve lines up with the frame-number x-axis.
            t = float(r["time"])
            if i == 1 and prev_time is not None and dt is None:
                dt = t - prev_time
            prev_time = t

    # Initial displacement = amplitude of the undamped oscillation. Taken from
    # the data so the analytical curve matches the run's actual start (sign
    # included), regardless of stretched vs. compressed setup.
    if offset0 is None and offsets:
        offset0 = offsets[0]

    label = os.path.basename(path).replace(".csv", "")
    lw = max(base_lw - idx * lw_step, 0.8)
    plt.plot(frames, offsets, "-", label=label, alpha=0.8,
             color=solver_color(label), lw=lw)

# Analytical undamped solution: offset(t) = offset0 * cos(omega * t).
if dt is not None and offset0 is not None:
    n = np.arange(MAX_FRAMES)            # frame index (x-axis)
    t = n * dt                           # corresponding simulation time
    analytical = offset0 * np.cos(OMEGA * t)
    plt.plot(n, analytical, "k--", lw=1.5, alpha=0.9,
             label="analytical (undamped)")
    plt.annotate(
        r"$x(t) = x_0\,\cos(\omega t),\quad \omega=\sqrt{k/m}="
        + f"{OMEGA:.2f}" + r"\,\mathrm{s^{-1}}$",
        xy=(0.02, 0.02), xycoords="axes fraction",
        fontsize=11, va="bottom", ha="left",
        bbox=dict(boxstyle="round", fc="white", ec="0.7", alpha=0.85))

plt.axhline(0.0, color="black", ls=":", alpha=0.5)
plt.xlabel("frame number")
plt.ylabel("offset [m]")
plt.title("Harmonic oscillator: displacement over time")
plt.grid(True, which="both", ls=":", alpha=0.5)
plt.legend()
plt.tight_layout()

out_dir = r"..\Visuals\SpringLength"
os.makedirs(out_dir, exist_ok=True)
out_path = os.path.join(out_dir, "oscillator_amplitude.png")
plt.savefig(out_path, dpi=300, bbox_inches="tight")
print(f"Saved plot to {out_path}")
