#!/usr/bin/env python3
"""Closed-loop results figure for manuscript.tex (Section VI).

Plots recorded data only; no simulation is run. Inputs are the per-frame
traces of the timed-flow campaign named in
report/SHARMA_TIMED_FLOW_20261002/comparison.json. Each raw file is checked
against the SHA-256 stored in that committed record before it is used.

Usage: python3 closed_loop_results_v01.py <repository root> [output stem]
"""
import hashlib
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

repo = Path(sys.argv[1]).resolve()
stem = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).with_suffix("")
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


def load():
    comparison = json.loads((repo / "report/SHARMA_TIMED_FLOW_20261002/comparison.json").read_text())
    runs = {}
    for record in comparison["results"]:
        if record["version"] != "current":
            continue
        source = Path(record["source"])
        if hashlib.sha256(source.read_bytes()).hexdigest() != record["sourceSha256"]:
            raise SystemExit(f"hash mismatch: {source}")
        runs[(record["speed"], record["scenario"])] = json.loads(source.read_text())["results"]
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


def trajectory_panel(axis, result, title, stamps, limits, contact=None):
    curved = result["baselineCruise"]["referenceCurve"]["curvature"] != 0
    rows, ego_shape, target_shape = samples(result)

    def transform(points):
        return [path_coordinates(px, py, curved) for px, py in points]

    window = [row for row in rows if row[0] <= stamps[-1] + 1.5]
    axis.axhline(0, color=GREY, lw=0.5, ls=(0, (4, 3)), zorder=1)
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
        above = d < 5
        axis.annotate(f"{stamp:g} s", (s, d), xytext=(0, 8 if above else -8),
                      textcoords="offset points", ha="center", va="bottom" if above else "top",
                      fontsize=5.5, color=BLUE, zorder=6,
                      bbox=dict(boxstyle="round,pad=0.08", fc="white", ec="none", alpha=0.75))
    if contact:
        # Mean of the body corners lying inside the other body at the deepest sample.
        row = min((r for r in rows if r[3] <= 0), key=lambda r: abs(r[0] - contact))
        ego_corners = rectangle(*row[1][:3], shape=ego_shape)
        target_corners = rectangle(*row[2][:3], shape=target_shape)
        hits = ([c for c in ego_corners if inside(c, row[2], target_shape)]
                + [c for c in target_corners if inside(c, row[1], ego_shape)])
        px = sum(c[0] for c in hits) / len(hits); py = sum(c[1] for c in hits) / len(hits)
        s, d = path_coordinates(px, py, curved)
        axis.plot([s], [d], marker="x", color=RED, ms=5, mew=1.2, zorder=7)
        axis.annotate("contact", (s, d), xytext=(-52, -24), textcoords="offset points",
                      fontsize=5.5, color=RED, zorder=7, va="center",
                      arrowprops=dict(arrowstyle="-", color=RED, lw=0.5, shrinkA=1, shrinkB=3))
    axis.set_aspect("equal")
    axis.set_xlim(*limits[0]); axis.set_ylim(*limits[1])
    axis.set_title(title, fontsize=7, pad=2.5, loc="left")
    axis.set_ylabel("lateral (m)", labelpad=1)
    axis.tick_params(length=2)


def main():
    runs = load()
    figure = plt.figure(figsize=(7.16, 3.75))
    grid = figure.add_gridspec(3, 2, width_ratios=[1.95, 1], hspace=0.62, wspace=0.2,
                               left=0.055, right=0.995, top=0.955, bottom=0.105)
    stamps = [0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
    panels = [((15, "headOn"), "(a) Head-on, 15 m/s (minimum distance 0.097 m)", None),
              ((15, "crossing"), "(b) Crossing, 15 m/s (minimum distance 0.054 m)", None),
              ((15, "turningCrossing"), "(c) Turning crossing, 15 m/s (contact from 1.51 to 1.54 s)", 1.525)]
    for row, (key, title, contact) in enumerate(panels):
        axis = figure.add_subplot(grid[row, 0])
        trajectory_panel(axis, runs[key], title, stamps, ((-4, 62), (-6, 11.5)), contact)
        if row == len(panels) - 1:
            axis.set_xlabel("station along the path (m)", labelpad=1)
    axis_gap = figure.add_subplot(grid[0, 1])
    axis_lateral = figure.add_subplot(grid[1, 1])
    axis_time = figure.add_subplot(grid[2, 1])
    call_seconds = []
    for (speed, scenario), result in sorted(runs.items()):
        rows, _, _ = samples(result)
        times = [r[0] for r in rows]
        gaps = [max(r[3], 1e-3) for r in rows]
        failed = result["minimumReplayClearanceMeters"] <= 0
        color = RED if failed else (BLUE if speed == 15 else GREY)
        width = 1.1 if failed else 0.6
        axis_gap.semilogy(times, gaps, color=color, lw=width, zorder=3 if failed else 2)
        hold_times = [h["time"] for h in result["trace"]]
        axis_lateral.plot(hold_times, [abs(h["transverseError"][0]) for h in result["trace"]],
                          color=color, lw=width, zorder=3 if failed else 2)
        call_seconds += [h["controllerSeconds"] for h in result["trace"]]
    axis_gap.set_xlim(0, 4); axis_gap.set_ylim(8e-3, 40)
    axis_gap.axhline(0.10, color="k", lw=0.5, ls=":")
    axis_gap.text(3.95, 0.082, "node margin 0.10 m", ha="right", va="top", fontsize=5.5)
    axis_gap.text(1.62, 1.0e-2, "contact", color=RED, fontsize=5.5, va="bottom")
    axis_gap.set_yticks([1e-2, 1e-1, 1, 10])
    axis_gap.set_xlabel("time (s)", labelpad=1); axis_gap.set_ylabel("body distance (m)", labelpad=1)
    axis_gap.set_title("(d) Distance to the obstacle, 14 runs", fontsize=7, pad=2.5, loc="left")
    axis_lateral.set_xlim(0, 20.4); axis_lateral.set_ylim(0, 10.5)
    axis_lateral.set_xticks([0, 5, 10, 15, 20])
    axis_lateral.set_xlabel("time (s)", labelpad=1); axis_lateral.set_ylabel(r"$|e_y|$ (m)", labelpad=1)
    axis_lateral.set_title("(e) Lateral deviation from the path", fontsize=7, pad=2.5, loc="left")
    ordered = sorted(call_seconds)
    axis_time.semilogx(ordered, [(k + 1) / len(ordered) for k in range(len(ordered))], color="k")
    axis_time.axvline(0.05, color=RED, lw=0.7, ls="--")
    axis_time.text(0.054, 0.5, "hold period 50 ms", color=RED, fontsize=5.5, rotation=90, va="center")
    axis_time.set_xlim(0.03, 1.2); axis_time.set_ylim(0, 1)
    axis_time.set_xticks([0.05, 0.1, 0.2, 0.5, 1.0])
    axis_time.set_xticklabels(["0.05", "0.1", "0.2", "0.5", "1"])
    axis_time.minorticks_off()
    axis_time.set_xlabel("controller call time (s)", labelpad=1); axis_time.set_ylabel("fraction of calls", labelpad=1)
    axis_time.set_title(f"(f) Call time, {len(ordered)} calls", fontsize=7, pad=2.5, loc="left")
    for axis in (axis_gap, axis_lateral, axis_time):
        axis.tick_params(length=2)
        axis.grid(True, which="major", lw=0.3, color="0.85")
    handles = [plt.Line2D([], [], color=GREY, lw=0.8, label="8 m/s"),
               plt.Line2D([], [], color=BLUE, lw=0.8, label="15 m/s"),
               plt.Line2D([], [], color=RED, lw=1.1, label="15 m/s turning crossing")]
    axis_lateral.legend(handles=handles, loc="upper right", frameon=False, handlelength=1.4,
                        borderaxespad=0.2, labelspacing=0.25)
    figure.savefig(stem.with_suffix(".pdf"))
    figure.savefig(stem.with_suffix(".png"), dpi=220)


if __name__ == "__main__":
    main()
