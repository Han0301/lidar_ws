#!/usr/bin/env python3
"""Plot the odometry replay output for all three recorded cases."""

import csv
import json
import os
from pathlib import Path

ROOT = Path(os.environ.get("LIDAR_EVAL_RESULTS", Path(__file__).resolve().parent / "results")).resolve()
ROOT.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / "mplconfig"))
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load(case):
    with (ROOT / case / "odometry.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    result = json.loads((ROOT / case / "odometry_summary.json").read_text(encoding="utf-8"))
    return rows, result


fig, axes = plt.subplots(1, 3, figsize=(15, 4.6), constrained_layout=True)

rows, summary = load("indoor_static")
time = [float(row["stamp_s"]) - float(rows[0]["stamp_s"]) for row in rows]
for axis in ("x", "y", "z"):
    first = float(rows[0][axis])
    axes[0].plot(time, [(float(row[axis]) - first) * 100 for row in rows], label=axis.upper())
axes[0].axhline(0, color="0.7", lw=0.7)
axes[0].set(title=f"Indoor static | end gap {summary['closure_3d_m']*100:.1f} cm",
            xlabel="Sensor time (s)", ylabel="Displacement from first pose (cm)")
axes[0].legend()
axes[0].grid(alpha=0.25)

for ax, case, title in zip(axes[1:], ("indoor_loop", "outdoor_loop"),
                           ("Indoor closed path", "Outdoor closed path")):
    rows, summary = load(case)
    x = [float(row["x"]) for row in rows]
    y = [float(row["y"]) for row in rows]
    ax.plot(x, y, lw=1.0, color="#2362a7")
    ax.scatter([x[0]], [y[0]], color="green", s=45, marker="o", label="Start", zorder=4)
    ax.scatter([x[-1]], [y[-1]], color="red", s=45, marker="x", label="End", zorder=4)
    ax.set(title=f"{title} | end gap {summary['closure_3d_m']*100:.1f} cm",
           xlabel="X in camera_init (m)", ylabel="Y in camera_init (m)")
    ax.axis("equal")
    ax.grid(alpha=0.25)
    ax.legend()

ROOT.mkdir(parents=True, exist_ok=True)
output = ROOT / "trajectories.png"
fig.savefig(output, dpi=180)
print(output)
