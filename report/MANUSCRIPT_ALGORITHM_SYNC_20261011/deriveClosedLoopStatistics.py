#!/usr/bin/env python3
"""Closed-loop statistics quoted in paper/manuscript.tex (October 11, 2026 sync).

Reads the recorded per-frame traces of the 98-encounter campaign of the
controller at commit ea88059 (report/CONTROLLER_SPEED_20261011.tex) and runs
no simulation. The campaign root holds one output file per encounter,
seed<S>-speed<V>-<scenario>.json, with seed 0 the exact-state runs and the
other six seeds the runs with the estimator in the loop.

Usage: deriveClosedLoopStatistics.py <campaign root> <output json>
"""
import json
import math
import re
import statistics
import sys
from pathlib import Path

root = Path(sys.argv[1]).expanduser()
ZERO_PRIMARY = 1e-4 + 1e-5  # lexicographicTieTolerance + feasibilityTolerance


def quantile(values, q):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


def target_state(q, time):
    """predictiveSafetyGeometry.predictTarget: constant A and beta, braking ends at rest."""
    x, y, yaw, speed, acceleration, beta, rear = q[:7]
    arc = speed * time + .5 * acceleration * time * time
    now = speed + acceleration * time
    if speed * acceleration < 0 and time >= -speed / acceleration:
        arc, now, acceleration = -speed * speed / (2 * acceleration), 0.0, 0.0
    elif speed == 0 and acceleration <= 0:
        arc, now, acceleration = 0.0, 0.0, 0.0
    curvature = math.sin(beta) / rear
    half = curvature * arc / 2
    scale = math.sin(half) / half if half else 1.0
    course = yaw + beta + half
    return (x + arc * scale * math.cos(course), y + arc * scale * math.sin(course),
            yaw + curvature * arc, now, acceleration, beta)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def run_statistics(path):
    result = json.loads(path.read_text())["results"]
    if isinstance(result, list):
        result = result[0]
    trace = result["trace"]
    configuration = result["configuration"]
    h = configuration["controller"]["sampleTime"]
    rho = math.exp(-2 * h / configuration["clf"]["convergenceTimeConstantSeconds"])
    inputs = [f["input"] for f in trace]
    audit = [s for f in trace for s in f["auditStates"]]
    shifted = [f for f in trace if f["initialization"] == "shiftedInputRollout"]
    recovery = result["recovery"] if isinstance(result["recovery"], dict) else {}
    pairs = list(zip(trace[:-1], trace[1:]))
    zero = [(a, b) for a, b in pairs
            if a["clfInitialValue"] > 1e-3 and a["clfSlack"] <= 1e-5 * max(1.0, a["clfInitialValue"])]
    # Pairs whose issued first input is the CLF stage's own (full step).
    full = [(a, b) for a, b in zero if a["acceptedStep"] == 1]
    stats = {
        "outcome": result["outcome"], "failure": result.get("failure", ""),
        "frames": len(trace), "executedFrames": result["executedFrames"],
        "failureTime": result.get("failureTime"),
        "collision": bool(result["collisionDetected"]),
        "roadDeparture": bool(result["roadDepartureDetected"]),
        "minimumGap": result["minimumReplayClearanceMeters"],
        "minimumRoadMargin": result["minimumReplayRoadMarginMeters"],
        "maximumLateralDeviation": result["maximumReplayLateralDeviationMeters"],
        "recoverySeconds": recovery.get("confirmationTimeSeconds") if recovery.get("recovered") else None,
        "maximumSteering": max(abs(u[0]) for u in inputs) if inputs else None,
        "brakingRatioRange": [min(u[1] for u in inputs), max(u[1] for u in inputs)] if inputs else None,
        "maximumYawRate": max(abs(s[5]) for s in audit) if audit else None,
        "maximumSideslip": max(abs(math.atan2(s[4], s[3])) for s in audit) if audit else None,
        "minimumSpeed": min(s[3] for s in audit) if audit else None,
        "framesBelow4": sum(f["state"][3] < 4 for f in trace),
        "framesBelow2p5": sum(f["state"][3] < 2.5 for f in trace),
        "positivePrimaryFrames": sum(f["primaryOptimum"] > ZERO_PRIMARY for f in trace),
        "maximumPrimaryOptimum": max((f["primaryOptimum"] for f in trace), default=None),
        "freshRestarts": sum(bool(f["potentialFieldRestarted"]) for f in trace),
        "initializations": {k: sum(f["initialization"] == k for f in trace)
                            for k in sorted({f["initialization"] for f in trace})},
        "horizons": [f["horizonSteps"] for f in trace],
        "firstHorizon": trace[0]["horizonSteps"] if trace else None,
        "acceptedSteps": [f["acceptedStep"] for f in trace],
        "relinearizedFrames": sum((f.get("acceptanceIterations") or 0) > 0 for f in trace),
        "shiftedFrames": len(shifted),
        "shiftedCandidateFeasible": sum(bool(f["candidateFeasible"]) for f in shifted),
        "shiftedCandidateReasons": [f["candidateReason"] for f in shifted if not f["candidateFeasible"]],
        "anchorCertified": sum(bool(f["anchorCertified"]) for f in shifted),
        "appendedTerminal": sum(f["appendedTerminalSteps"] or 0 for f in trace),
        "framesWithBrakingHold": sum((f["appendedBrakingSteps"] or 0) > 0 for f in trace),
        "framesWithGuidedHolds": sum((f["appendedGuidedSteps"] or 0) > 0 for f in trace),
        "extensionStopped": [f["extensionStopped"] for f in trace if f["extensionStopped"]],
        "terminalEnds": {k: sum(f["terminalEnd"] == k for f in trace)
                         for k in sorted({f["terminalEnd"] for f in trace})},
        "terminalMargins": [f["terminalMarginMeters"] for f in trace if f["terminalMarginMeters"] is not None],
        "linearizationGaps": [f["linearizationGapMeters"] for f in trace if f["linearizationGapMeters"] is not None],
        "modelInnovations": [f["trust"]["modelInnovation"] for f in trace
                             if isinstance(f.get("trust"), dict) and f["trust"].get("modelInnovation") is not None],
        "trustScales": [f["inputTrustScale"] for f in trace],
        "clfPairs": len(zero),
        "clfPairsMissingDecrease": sum(b["clfInitialValue"] > rho * a["clfInitialValue"] + 1e-9 for a, b in zero),
        "clfPairsIncreasing": sum(b["clfInitialValue"] > a["clfInitialValue"] + 1e-9 for a, b in zero),
        "clfPairsAllLarge": sum(a["clfInitialValue"] > 1e-3 for a, b in pairs),
        "clfFullStepPairs": len(full),
        "clfFullStepPairsMissingDecrease": sum(b["clfInitialValue"] > rho * a["clfInitialValue"] + 1e-9 for a, b in full),
        "clfFullStepPairsIncreasing": sum(b["clfInitialValue"] > a["clfInitialValue"] + 1e-9 for a, b in full),
        "controllerSeconds": [f["controllerSeconds"] for f in trace],
        "observerSeconds": [f.get("observerSeconds") or 0 for f in trace],
    }
    # Estimation errors with the estimator in the loop (true state against the
    # controller's input), and the published target parameters against truth.
    q0 = result["targetInitialState"]
    heading, lateral, position, accel, beta, course = [], [], [], [], [], []
    target_set = {"positionRadius": [], "courseRadius": [], "speedHalfWidth": [],
                  "accelerationHalfWidth": [], "curvatureHalfWidth": []}
    for f in trace:
        x, xh = f["state"], f["estimatedState"]
        heading.append(abs(wrap(xh[2] - x[2])))
        lateral.append(abs(xh[4] - x[4]))
        position.append(math.hypot(xh[0] - x[0], xh[1] - x[1]))
        qh = f.get("estimatedTarget")
        if f.get("observedTarget") and qh and 0.5 <= f["time"] <= 3.0:
            truth = target_state(q0, f["time"])
            accel.append((qh[4] - truth[4]) ** 2)
            beta.append((qh[5] - truth[5]) ** 2)
        ts = f.get("targetSet")
        if isinstance(ts, dict) and ts.get("available"):
            for k in target_set:
                if ts.get(k) is not None:
                    target_set[k].append(ts[k])
    stats["egoErrorP95"] = {"heading": quantile(heading, .95) if heading else None,
                            "lateralVelocity": quantile(lateral, .95) if lateral else None,
                            "position": quantile(position, .95) if position else None}
    stats["targetParameterRms"] = {"acceleration": math.sqrt(sum(accel) / len(accel)) if accel else None,
                                   "sideslip": math.sqrt(sum(beta) / len(beta)) if beta else None}
    stats["targetSetMedian"] = {k: statistics.median(v) if v else None for k, v in target_set.items()}
    stats["targetSetFrames"] = len(target_set["positionRadius"])
    return stats


def summarize(runs):
    seconds = [s for r in runs.values() for s in r["controllerSeconds"]]
    horizons = [h for r in runs.values() for h in r["horizons"]]
    steps = [a for r in runs.values() for a in r["acceptedSteps"] if a is not None]
    gaps = [g for r in runs.values() for g in r["linearizationGaps"]]
    innovations = [g for r in runs.values() for g in r["modelInnovations"]]
    ends = {}
    for r in runs.values():
        for k, v in r["terminalEnds"].items():
            ends[k] = ends.get(k, 0) + v
    stopped = {}
    for r in runs.values():
        for k in r["extensionStopped"]:
            stopped[k] = stopped.get(k, 0) + 1
    reasons = {}
    for r in runs.values():
        for k in r["shiftedCandidateReasons"]:
            reasons[k] = reasons.get(k, 0) + 1
    recovered = [r for r in runs.values() if r["outcome"] == "recovered"]
    return {
        "runs": len(runs), "recovered": len(recovered),
        "outcomes": {o: sum(r["outcome"] == o and (r["failure"] or "") == "" for r in runs.values())
                     for o in sorted({r["outcome"] for r in runs.values()})},
        "failures": {k: r["failure"] for k, r in runs.items() if r["outcome"] != "recovered"},
        "collisions": sum(r["collision"] for r in runs.values()),
        "roadDepartures": sum(r["roadDeparture"] for r in runs.values()),
        "frames": len(seconds),
        "controllerMedian": statistics.median(seconds), "controllerP95": quantile(seconds, .95),
        "controllerP99": quantile(seconds, .99), "controllerMaximum": max(seconds),
        "over50ms": sum(s > .05 for s in seconds), "over100ms": sum(s > .1 for s in seconds),
        "observerMedian": statistics.median(s for r in runs.values() for s in r["observerSeconds"]),
        "horizonMedian": statistics.median(horizons), "horizonP95": quantile(horizons, .95),
        "horizonP99": quantile(horizons, .99), "horizonMaximum": max(horizons),
        "firstHorizonMedian": statistics.median(r["firstHorizon"] for r in runs.values() if r["firstHorizon"]),
        "acceptedStepCounts": {str(a): steps.count(a) for a in sorted(set(steps), reverse=True)},
        "relinearizedFrames": sum(r["relinearizedFrames"] for r in runs.values()),
        "shiftedFrames": sum(r["shiftedFrames"] for r in runs.values()),
        "shiftedCandidateFeasible": sum(r["shiftedCandidateFeasible"] for r in runs.values()),
        "shiftedCandidateReasons": reasons,
        "anchorCertified": sum(r["anchorCertified"] for r in runs.values()),
        "framesWithBrakingHold": sum(r["framesWithBrakingHold"] for r in runs.values()),
        "framesWithGuidedHolds": sum(r["framesWithGuidedHolds"] for r in runs.values()),
        "extensionStopped": stopped,
        "terminalEnds": ends,
        "freshRestarts": sum(r["freshRestarts"] for r in runs.values()),
        "positivePrimaryFrames": sum(r["positivePrimaryFrames"] for r in runs.values()),
        "runsWithPositivePrimary": sum(r["positivePrimaryFrames"] > 0 for r in runs.values()),
        "maximumPrimaryOptimum": max(r["maximumPrimaryOptimum"] or 0 for r in runs.values()),
        "linearizationGapMedian": statistics.median(gaps) if gaps else None,
        "linearizationGapP95": quantile(gaps, .95) if gaps else None,
        "linearizationGapMaximum": max(gaps) if gaps else None,
        "modelInnovationMedian": statistics.median(innovations) if innovations else None,
        "modelInnovationP95": quantile(innovations, .95) if innovations else None,
        "trustScaleRange": [min(min(r["trustScales"]) for r in runs.values() if r["trustScales"]),
                            max(max(r["trustScales"]) for r in runs.values() if r["trustScales"])],
        "maximumSteering": max(r["maximumSteering"] or 0 for r in runs.values()),
        "brakingRatioRange": [min(r["brakingRatioRange"][0] for r in runs.values() if r["brakingRatioRange"]),
                              max(r["brakingRatioRange"][1] for r in runs.values() if r["brakingRatioRange"])],
        "maximumYawRate": max(r["maximumYawRate"] or 0 for r in runs.values()),
        "maximumSideslip": max(r["maximumSideslip"] or 0 for r in runs.values()),
        "minimumSpeed": min(r["minimumSpeed"] for r in runs.values() if r["minimumSpeed"] is not None),
        "runsBelow4": sum(r["framesBelow4"] > 0 for r in runs.values()),
        "runsBelow2p5": sum(r["framesBelow2p5"] > 0 for r in runs.values()),
        "lateralDeviationRange": [min(r["maximumLateralDeviation"] for r in runs.values()),
                                  max(r["maximumLateralDeviation"] for r in runs.values())],
        "minimumGap": min(r["minimumGap"] for r in runs.values()),
        "medianRunGap": statistics.median(r["minimumGap"] for r in runs.values()),
        "runsBelowMargin": sum(r["minimumGap"] < .2 for r in runs.values()),
        "minimumRoadMargin": min(r["minimumRoadMargin"] for r in runs.values()),
        "recoveryMedian": statistics.median(r["recoverySeconds"] for r in recovered) if recovered else None,
        "clfPairs": sum(r["clfPairs"] for r in runs.values()),
        "clfPairsAllLarge": sum(r["clfPairsAllLarge"] for r in runs.values()),
        "clfPairsMissingDecrease": sum(r["clfPairsMissingDecrease"] for r in runs.values()),
        "clfPairsIncreasing": sum(r["clfPairsIncreasing"] for r in runs.values()),
        "clfFullStepPairs": sum(r["clfFullStepPairs"] for r in runs.values()),
        "clfFullStepPairsMissingDecrease": sum(r["clfFullStepPairsMissingDecrease"] for r in runs.values()),
        "clfFullStepPairsIncreasing": sum(r["clfFullStepPairsIncreasing"] for r in runs.values()),
    }


runs = {}
for path in sorted(root.glob("seed*-speed*-*.json")):
    if path.name.endswith("-summary.json"):
        continue
    match = re.fullmatch(r"seed(\d+)-speed(\d+)-(\w+)", path.stem)
    runs[path.stem] = dict(run_statistics(path), seed=int(match.group(1)),
                           speed=int(match.group(2)), scenario=match.group(3))
groups = {"exact": {k: r for k, r in runs.items() if r["seed"] == 0},
          "noisy": {k: r for k, r in runs.items() if r["seed"] != 0}}
summary = {name: summarize(group) for name, group in groups.items()}
for r in runs.values():
    for key in ("controllerSeconds", "observerSeconds", "horizons", "acceptedSteps", "terminalMargins",
                "linearizationGaps", "modelInnovations", "trustScales", "shiftedCandidateReasons"):
        r.pop(key)
Path(sys.argv[2]).write_text(json.dumps({"summary": summary, "runs": runs}, indent=1))
for name, group in summary.items():
    print(name, json.dumps(group, indent=None)[:4000])
