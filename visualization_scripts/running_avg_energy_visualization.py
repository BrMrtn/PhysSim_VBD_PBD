import matplotlib.pyplot as plt
import numpy as np

# Path to your CSV file
csv_path = r"..\Logs\EnergyLogs\XPBDCloth_20260520_202954.csv"

# Load CSV data
data = np.genfromtxt(csv_path, delimiter=';', skip_header=1)

# Columns:
# 0 = frame
# 1 = time
# 2 = kinetic
# 3 = gravitational
# 4 = elastic
# 5 = total

time = data[:, 1]
total_energy = data[:, 5]
running_avg_total = np.cumsum(total_energy) / np.arange(1, len(total_energy) + 1)

# Draw graph
plt.figure(figsize=(10, 5))
plt.plot(time, running_avg_total, linewidth=2)

plt.xlabel("Time")
plt.ylabel("Running Average of Total Energy")
plt.title("Running Average of Total Energy Over Time")
plt.grid(True)

plt.tight_layout()
plt.show()