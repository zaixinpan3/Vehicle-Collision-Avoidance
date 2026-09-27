"""Audit road-free plant trials against road fits reconstructed after execution.

Uses the independent vehicle recovery auditor for physical state, rectangle
collision and recovery checks. The additional curb audit never treats absent
controller road rows as evidence of road containment.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.io import loadmat

from auditNonlinearRecoveryStudy import serializable, vector
from auditNonlinearVehicleRecovery import audit, normalize_matlab, quadratic_margin, structs


def audit_road_free(path):
    result = normalize_matlab(loadmat(path, simplify_cells=True))["result"]
    summary = audit(path)
    frames = structs(result["ablation"]["boundaryFrames"])
    trace = result["plantTrace"]
    time = vector(trace["time"])
    position = np.column_stack([trace["positionX"], trace["positionY"]])
    yaw = vector(trace["yaw"])
    cfg = result["controllerConfiguration"]
    size = np.array([cfg["vehicle"]["length"], cfg["vehicle"]["width"]])
    period = float(result["scenario"]["sampleTime"])
    tolerance = 100 * np.spacing(max(1., period, float(time[-1])))
    steps = np.clip(np.ceil((time - tolerance) / period).astype(int), 1, len(frames))
    margins = np.full((len(time), 2), np.nan)
    covered = np.zeros_like(margins, dtype=bool)
    for sample, step in enumerate(steps):
        for side, boundary in enumerate(structs(frames[step - 1])):
            margins[sample, side], covered[sample, side] = quadratic_margin(
                position[sample], yaw[sample], size, boundary)
    metadata = structs(result["attempts"]["metadata"])
    road_rows = [int(item["roadRowCount"]) for item in metadata if isinstance(item, dict)]
    solve = vector(result["attempts"]["solveTime"])
    posthoc_safe = bool(np.all(covered) and np.all(np.isfinite(margins)) and np.all(margins >= 0))
    summary.update(
        controllerRoadRowsAbsent=bool(road_rows and all(count == 0 for count in road_rows)),
        posthocRoadAudit=dict(
            sampleCount=len(time), boundaryFrameCount=len(frames),
            everyFootprintCovered=bool(np.all(covered)),
            minimumConservativeMargin=float(np.nanmin(margins)),
            negativeMarginSampleCount=int(np.count_nonzero(np.any(margins < 0, axis=1))),
            sampledContainment=posthoc_safe,
            scope="Post-execution local quadratic curb fits, 6/8 m lane offsets plus 2.6 m shoulders; sampled footprint check"),
        controllerTiming=dict(count=len(solve), medianSeconds=float(np.median(solve)),
                              p95Seconds=float(np.percentile(solve, 95)),
                              maximumSeconds=float(np.max(solve)),
                              over50msCount=int(np.count_nonzero(solve > .05))),
        recoveredWithPosthocRoadCheck=bool(summary["recovered"] and posthoc_safe))
    # In the imported audit, roadEnabled=false makes sampledRoadSafe vacuous.
    # Expose that absence explicitly; the separate posthoc result is authoritative.
    summary["sampledRoadSafe"] = None
    summary["minimumConservativeRoadFunctionMargin"] = None
    if not summary["controllerRoadRowsAbsent"]:
        summary["problems"].append("The controller retained road constraint rows.")
        summary["allChecksPassed"] = False
    return serializable(summary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    rows = [audit_road_free(path) for path in args.inputs]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(results=rows), indent=2, allow_nan=False) + "\n")
    for row in rows:
        print(f"{row['name']}: completed={row['completed']}, collisionFree={row['collisionFree']}, "
              f"posthocRoad={row['posthocRoadAudit']['sampledContainment']}, "
              f"recovered={row['recoveredWithPosthocRoadCheck']}, checks={row['allChecksPassed']}")
    if not all(row["allChecksPassed"] for row in rows):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
