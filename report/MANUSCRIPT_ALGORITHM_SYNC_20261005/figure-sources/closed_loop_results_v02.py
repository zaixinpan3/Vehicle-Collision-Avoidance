#!/usr/bin/env python3
"""Closed-loop results figure for manuscript.tex (Section VI).

Plots recorded data only; no simulation is run. Inputs are the per-frame
traces of the campaign of report/ESTIMATOR_DIRECT_TARGET_20261005.tex
(controller d1c31d6): 14 exact-state runs and 84 runs with the estimator and
noisy sensors.

Usage: python3 closed_loop_results_v02.py <repository root> <campaign root> [output stem]
"""
import hashlib
import json
import math
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

repo = Path(sys.argv[1]).resolve()
campaign = Path(sys.argv[2]).expanduser()
stem = Path(sys.argv[3]) if len(sys.argv) > 3 else Path(__file__).with_suffix("")
sys.path.insert(0, str(repo / "scripts"))
from auditJointPredictiveSafety import distance, rectangle, target_state  # noqa: E402

BLUE, ORANGE, GREY, RED = "#0072B2", "#D55E00", "#7F7F7F", "#CC0000"
LABEL = {"headOn": "Head-on", "acceleratingHeadOn": "Accelerating head-on",
         "brakingLead": "Braking lead", "crossing": "Crossing",
         "turningCrossing": "Turning crossing", "curvedHeadOn": "Curved head-on",
         "curvedCrossing": "Curved crossing"}

plt.rcParams.update({"font.family": "serif", "font.size": 7, "axes.linewidth": 0.6,
                     "axes.labelsize": 7, "xtick.labelsize": 6.5, "ytick.labelsize": 6.5,
                     "legend.fontsize": 6, "pdf.fonttype": 42, "lines.linewidth": 0.9})


def load(group):
    runs = {}
    for source in sorted((campaign / group).glob("speed*.json")):
        match = re.fullmatch(r"speed(\d+)-(\w+)", source.stem)
        result = json.loads(source.read_text())["results"]
        runs[(int(match.group(1)), match.group(2))] = result[0] if isinstance(result, list) else result
    return runs


def path_coordinates(x, y, curved):
    """Station and left-positive lateral offset on the straight or R = 200 m path."""
    if not curved:
        return x, y
    return 200 * math.atan2(x, 200 - y), 200 - math.hypot(x, y - 200)


def inside(point, pose, shape):
    """True when a point lies in a rectangle given by its pose and half extents."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    return (abs(c * dx + s * dy - shape[2]) <= shape[0]
            and abs(-s * dx + c * dy - shape[3]) <= shape[1])


def samples(result):
    """Time, ego state and body distance at every recorded replay instant."""
    initial = result["targetInitialState"]
    vehicle = result["configuration"]["vehicle"]
    shape = (vehicle["length"] / 2, vehicle["width"] / 2, *vehicle["rectangleOffset"])
    rows = []
    for hold in result["trace"]:
        for dt, state in zip(hold["auditTimes"][:-1], hold["auditStates"][:-1]):
            t = hold["time"] + dt
            target = target_state(initial, t)
            gap = distance(rectangle(*state[:3], shape=shape),
                           rectangle(*target[:3], shape=initial[7:11]))
            rows.append((t, state, target, gap))
    return rows, shape, initial[7:11]


def trajectory_panel(axis, result, title, stamps, limits):
    curved = result["baselineCruise"]["referenceCurve"]["curvature"] != 0
    rows, ego_shape, target_shape = samples(result)

    def transform(points):
        return [path_coordinates(px, py, curved) for px, py in points]

    window = [row for row in rows if row[0] <= stamps[-1] + 1.5]
    axis.axhline(0, color=GREY, lw=0.5, ls=(0, (4, 3)), zorder=1)
    for edge in (-8.5344, 12.192):
        axis.axhline(edge, color="k", lw=0.8, zorder=1)
    axis.plot(*zip(*transform([(r[1][0], r[1][1]) for r in window])), color=BLUE, zorder=3)
    axis.plot(*zip(*transform([(r[2][0], r[2][1]) for r in window])), color=ORANGE, zorder=3)
    for index, stamp in enumerate(stamps):
        row = min(rows, key=lambda r: abs(r[0] - stamp))
        shade = 0.22 + 0.6 * index / max(1, len(stamps) - 1)
        for pose, shape, color in ((row[1], ego_shape, BLUE), (row[2], target_shape, ORANGE)):
            corners = transform(rectangle(*pose[:3], shape=shape))
            axis.add_patch(Polygon(corners, closed=True, facecolor=color, edgecolor=color,
                                   alpha=shade, lw=0.5, zorder=4))
        s, d = path_coordinates(row[1][0], row[1][1], curved)
        axis.annotate(f"{stamp:g} s", (s, d), xytext=(0, -8), textcoords="offset points",
                      ha="center", va="top", fontsize=5.5, color=BLUE, zorder=6,
                      bbox=dict(boxstyle="round,pad=0.08", fc="white", ec="none", alpha=0.75))
    axis.set_aspect("equal")
    axis.set_xlim(*limits[0]); axis.set_ylim(*limits[1])
    axis.set_title(title, fontsize=7, pad=2.5, loc="left")
    axis.set_ylabel("lateral (m)", labelpad=1)
    axis.tick_params(length=2)


def main():
    exact = load("exact")
    noisy = {seed: load(f"noisy-seed{seed}") for seed in (20261003, 1, 2, 3, 4, 5)}
    figure = plt.figure(figsize=(7.16, 4.3))
    grid = figure.add_gridspec(3, 2, width_ratios=[1.95, 1], hspace=0.62, wspace=0.2,
                               left=0.055, right=0.995, top=0.96, bottom=0.095)
    stamps = [0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
    panels = [(exact[(15, "headOn")], "(a) Head-on, 15 m/s, exact states"),
              (noisy[20261003][(15, "headOn")], "(b) Head-on, 15 m/s, estimated states"),
              (noisy[20261003][(15, "turningCrossing")], "(c) Turning crossing, 15 m/s, estimated states")]
    for row, (result, title) in enumerate(panels):
        axis = figure.add_subplot(grid[row, 0])
        gap = result["minimumReplayClearanceMeters"]
        trajectory_panel(axis, result, f"{title} (min. distance {gap:.2f} m)", stamps, ((-4, 62), (-9.5, 13.2)))
        if row == len(panels) - 1:
            axis.set_xlabel("station along the path (m)", labelpad=1)
    axis_gap = figure.add_subplot(grid[0, 1])
    axis_lateral = figure.add_subplot(grid[1, 1])
    axis_time = figure.add_subplot(grid[2, 1])
    groups = [(runs, GREY, 0.35, 1) for runs in noisy.values()] + [(exact, BLUE, 0.7, 3)]
    calls = {"exact": [], "noisy": []}
    for runs, color, width, order in groups:
        for (speed, scenario), result in sorted(runs.items()):
            rows, _, _ = samples(result)
            early = [r for r in rows if r[0] <= 6.05]
            axis_gap.semilogy([r[0] for r in early], [max(r[3], 1e-3) for r in early],
                              color=color, lw=width, zorder=order)
            holds = [h for h in result["trace"] if h["time"] <= 20.5]
            axis_lateral.plot([h["time"] for h in holds], [abs(h["transverseError"][0]) for h in holds],
                              color=color, lw=width, zorder=order)
            calls["exact" if runs is exact else "noisy"] += [h["controllerSeconds"] for h in result["trace"]]
    axis_gap.set_xlim(0, 6); axis_gap.set_ylim(5e-2, 60)
    axis_gap.axhline(0.20, color="k", lw=0.5, ls=":")
    axis_gap.text(5.9, 0.17, "node margin 0.20 m", ha="right", va="top", fontsize=5.5)
    axis_gap.set_yticks([1e-1, 1, 10])
    axis_gap.set_xlabel("time (s)", labelpad=1); axis_gap.set_ylabel("body distance (m)", labelpad=1)
    axis_gap.set_title("(d) Distance to the obstacle, 98 runs", fontsize=7, pad=2.5, loc="left")
    axis_lateral.set_xlim(0, 20.4); axis_lateral.set_ylim(0, 11.5)
    axis_lateral.set_xticks([0, 5, 10, 15, 20])
    axis_lateral.set_xlabel("time (s)", labelpad=1); axis_lateral.set_ylabel(r"$|e_y|$ (m)", labelpad=1)
    axis_lateral.set_title("(e) Lateral deviation from the path", fontsize=7, pad=2.5, loc="left")
    for name, color in (("noisy", GREY), ("exact", BLUE)):
        ordered = sorted(calls[name])
        axis_time.semilogx(ordered, [(k + 1) / len(ordered) for k in range(len(ordered))], color=color)
    axis_time.axvline(0.05, color=RED, lw=0.7, ls="--")
    axis_time.text(0.056, 0.45, "hold period 50 ms", color=RED, fontsize=5.5, rotation=90, va="center")
    axis_time.set_xlim(0.002, 1.2); axis_time.set_ylim(0, 1)
    axis_time.set_xticks([0.01, 0.05, 0.2, 1.0])
    axis_time.set_xticklabels(["0.01", "0.05", "0.2", "1"])
    axis_time.minorticks_off()
    axis_time.set_xlabel("controller call time (s)", labelpad=1); axis_time.set_ylabel("fraction of calls", labelpad=1)
    count = len(calls["exact"]) + len(calls["noisy"])
    axis_time.set_title(f"(f) Call time, {count} calls", fontsize=7, pad=2.5, loc="left")
    for axis in (axis_gap, axis_lateral, axis_time):
        axis.tick_params(length=2)
        axis.grid(True, which="major", lw=0.3, color="0.85")
    handles = [plt.Line2D([], [], color=BLUE, lw=0.8, label="exact states (14)"),
               plt.Line2D([], [], color=GREY, lw=0.8, label="estimated states (84)")]
    axis_lateral.legend(handles=handles, loc="upper right", frameon=False, handlelength=1.4,
                        borderaxespad=0.2, labelspacing=0.25)
    figure.savefig(stem.with_suffix(".pdf"))
    figure.savefig(stem.with_suffix(".png"), dpi=220)
    print(count, len(calls["exact"]), len(calls["noisy"]))


if __name__ == "__main__":
    main()
