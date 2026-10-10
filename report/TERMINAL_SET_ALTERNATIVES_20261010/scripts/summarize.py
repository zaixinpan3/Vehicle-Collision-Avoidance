#!/usr/bin/env python3
"""Tabulate alternativeCertificate results (JSON lines) as text or LaTeX rows."""
import json, sys

def rows(paths):
    for path in paths:
        with open(path) as handle:
            for line in handle:
                line = line.strip()
                if line:
                    yield json.loads(line)

def fmt(r):
    e = r["extents"]
    speeds = r["speeds"] if isinstance(r["speeds"], list) else [r["speeds"]]
    curv = r["curvatures"] if isinstance(r["curvatures"], list) else [r["curvatures"]]
    point = f"{speeds[0]:g}/{curv[0]:g}" if len(speeds) == 1 else f"{min(speeds):g}-{max(speeds):g}/{min(curv):g}-{max(curv):g}"
    union = ""
    if r["unionExtents"] != r["extents"]:
        u = r["unionExtents"]
        union = f" (union {u[0]:.2f} m, {u[1]:.3f} rad)"
    slips = r["maximumSlipRadians"]
    return (f"{r['label']:<26} {point:<14} lat {e[0]:.3f} m  head {e[1]:.4f} rad  speed {e[2]:.3f}  vy {e[3]:.3f}  r {e[4]:.4f}"
            f"  fill {r['regionFill']:.3f}  T {r['certifiedTimeConstantSeconds']:.2f}/{r['requiredTimeConstantSeconds']:g} s"
            f"  mu {r['holdFactor']:.4f}  slips {slips[0]:.3f}/{slips[1]:.3f}  worst {r['sampledWorstContraction']:.4f}"
            f"  rows {int(bool(r['sampledRowsSatisfied']))}  {r['elapsedSeconds']:.0f} s{union}")

def latex(r):
    e = r["extents"]
    speeds = r["speeds"] if isinstance(r["speeds"], list) else [r["speeds"]]
    curv = r["curvatures"] if isinstance(r["curvatures"], list) else [r["curvatures"]]
    point = f"{speeds[0]:g} & {curv[0]:g}" if len(speeds) == 1 else f"{min(speeds):g}--{max(speeds):g} & {min(curv):g}--{max(curv):g}"
    label = r["label"].replace("+", "{+}")
    return (f"\\texttt{{{label}}} & {point} & {e[0]:.2f} & {e[1]:.3f} & {e[2]:.2f} & {e[3]:.3f} & {e[4]:.3f} & "
            f"{r['certifiedTimeConstantSeconds']:.2f} & {r['holdFactor']:.3f} & {r['sampledWorstContraction']:.3f} \\\\")

if __name__ == "__main__":
    args = sys.argv[1:]
    mode = "text"
    if args and args[0] == "--latex":
        mode = "latex"; args = args[1:]
    for r in rows(args):
        print(latex(r) if mode == "latex" else fmt(r))
