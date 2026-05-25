import os

import matplotlib.pyplot as plt

# Steady-state average FPS on the wind-driven hanging cloth (Table tab:NodeCountFPS).
resolutions = ["8x8", "16x16", "32x32", "64x64", "128x128"]

xpbd = [133.8, 84.5, 36.0, 10.6, 2.8]  # baseline: XPBD S4xI4
vbd = {
    "VBD S4xImax4": [138.1,91.2, 31.8, 8.3, 2.1 ],
    "VBD S4xI2": [136.0, 92.4, 42.8, 13.6, 3.7],
    "VBD S4xI4": [124.7, 73.4, 29.2,  8.1, 2.1],
}

# Relative FPS difference vs XPBD, in %. Positive = VBD faster than XPBD.
plt.figure(figsize=(9, 6))
x = list(range(len(resolutions)))
n = len(vbd)
width = 0.8 / n

for i, (method, values) in enumerate(vbd.items()):
    diff = [100.0 * (v / b - 1.0) for v, b in zip(values, xpbd)]
    offset = (i - (n - 1) / 2) * width
    bars = plt.bar([xi + offset for xi in x], diff, width, label=method,
                   alpha=0.85, edgecolor="black", linewidth=0.5)
    for rect, d in zip(bars, diff):
        va = "bottom" if d >= 0 else "top"
        plt.text(rect.get_x() + rect.get_width() / 2, d, f"{d:+.0f}%",
                 ha="center", va=va, fontsize=9)

plt.axhline(0, color="black", lw=1)
plt.xticks(x, resolutions)
plt.xlabel("Mesh resolution", fontsize=14)
plt.ylabel("FPS difference vs XPBD S4xI4 (%)", fontsize=14)
plt.grid(True, axis="y", ls=":", alpha=0.5)
plt.legend(fontsize=12)
plt.tight_layout()

out_dir = r"..\Visuals"
os.makedirs(out_dir, exist_ok=True)
out_path = os.path.join(out_dir, "NodeCountFPS_vbd_vs_xpbd_pct.png")
plt.savefig(out_path, dpi=300, bbox_inches="tight")
print(f"Saved plot to {out_path}")
