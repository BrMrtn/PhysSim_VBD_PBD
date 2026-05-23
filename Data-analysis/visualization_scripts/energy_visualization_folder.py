import glob
import os
import re

import matplotlib.pyplot as plt
import numpy as np

# Folder containing the energy CSV logs
logs_dir = r"..\Logs\EnergyLogs"
save_dir = r"..\Visuals\EnergyDissipation"

os.makedirs(save_dir, exist_ok=True)


def solver_color(name):
    """Color a series by its solver name: XPBD->green, VBD->red, Newton->blue."""
    if name.startswith("XPBD"):
        return "green"
    if name.startswith("VBD"):
        return "red"
    if name.startswith("Newton"):
        return "blue"
    return None

def format_label(name):
    """Format filename to a readable label like 'VBD 4 iterations'."""
    solver = "VBD" if name.startswith("VBD") else "XPBD" if name.startswith("XPBD") else "Newton" if name.startswith("Newton") else ""
    match = re.search(r'_i(\d+)', name)
    if solver and match:
        return f"{solver} {match.group(1)} iterations"
    return os.path.splitext(name)[0]


# Columns:
# 0 = frame
# 1 = time
# 2 = kinetic
# 3 = gravitational
# 4 = elastic
# 5 = total

# Load every CSV in the folder, keeping only VBD and XPBD logs.
series = []
for csv_path in sorted(glob.glob(os.path.join(logs_dir, "*.csv"))):
    name = os.path.basename(csv_path)
    if not (name.startswith("VBD") or name.startswith("XPBD")):
        continue

    data = np.genfromtxt(csv_path, delimiter=';', skip_header=1)
    time_series = data[:, 1]
    total = data[:, 5]
    series.append((name, time_series, total))

if not series:
    raise SystemExit(f"No VBD/XPBD CSV files found in {logs_dir}")

# Clip every series to the smallest common end time so they share an x-range.
common_start = 0
common_end = 80

# Draw graph
plt.figure(figsize=(10, 5))
for name, time, total in series:
    mask = (time > common_start) & (time <= common_end)
    label = format_label(name)
    
    plt.plot(time[mask], total[mask], linewidth=2, label=label)

plt.xlabel("Time", fontsize=14)
plt.ylabel("Total Energy", fontsize=14)

plt.legend()
plt.grid(True)

plt.tight_layout()
filename = "energy_dissipation_s" + str(common_start) + "_e" + str(common_end) + ".png"
save_path = os.path.join(save_dir, filename)
plt.savefig(save_path, dpi=300)
print(f"Plot saved to: {os.path.abspath(save_path)}")