#!/usr/bin/env python3
"""Derive the closed-loop statistics quoted in paper/manuscript.tex.

No simulation is run. The input is the recorded timed-flow campaign of
controller commit a29ccb6: report/SHARMA_TIMED_FLOW_20261002/comparison.json
(committed) names one raw per-frame trace per scenario in the external
experiment directory and stores its SHA-256. Every raw file is verified
against that hash before use. Body distance is recomputed with the
independent Python geometry of scripts/auditJointPredictiveSafety.py.

Usage: python3 deriveClosedLoopStatistics.py <repository root> <output.json>
"""
import hashlib
import json
import math
import statistics
import sys
from pathlib import Path

repo = Path(sys.argv[1]).resolve()
output = Path(sys.argv[2])
sys.path.insert(0, str(repo / "scripts"))
from auditJointPredictiveSafety import distance, rectangle, target_state  # noqa: E402


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


comparison = json.loads((repo / "report/SHARMA_TIMED_FLOW_20261002/comparison.json").read_text())
rows = []
calls = []
parts = {"initialization": [], "formulation": [], "solver": []}
clf = {"pairs": 0, "zeroSlackPairs": 0, "decreaseNotAchieved": 0, "valueIncreased": 0, "largestIncrease": 0.0}
for record in comparison["results"]:
    if record["version"] != "current":
        continue
    source = Path(record["source"])
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    if digest != record["sourceSha256"]:
        raise SystemExit(f"hash mismatch: {source}")
    result = json.loads(source.read_text())["results"]
    trace = result["trace"]
    configuration = result["configuration"]
    initial = result["targetInitialState"]
    vehicle = configuration["vehicle"]
    shape = (vehicle["length"] / 2, vehicle["width"] / 2, *vehicle["rectangleOffset"])
    tolerance = configuration["solver"]["feasibilityTolerance"]

    minimum, minimum_time, contact = math.inf, None, []
    for hold in trace:
        for dt, state in zip(hold["auditTimes"], hold["auditStates"]):
            t = hold["time"] + dt
            gap = distance(rectangle(*state[:3], shape=shape),
                           rectangle(*target_state(initial, t)[:3], shape=initial[7:11]))
            if gap < minimum:
                minimum, minimum_time = gap, t
            if gap <= 0:
                contact.append(t)
    assert abs(minimum - result["minimumReplayClearanceMeters"]) < 1e-9

    seconds = [hold["controllerSeconds"] for hold in trace]
    resume = next((k for k, hold in enumerate(trace) if abs(hold["time"] - 8.0) < 1e-9), None)
    positive = [hold for hold in trace if (hold["primaryOptimum"] or 0) > tolerance]
    pairs = zero = missed = increased = 0
    largest = 0.0
    for now, nxt in zip(trace[:-1], trace[1:]):
        v0, v1 = now["clfInitialValue"], nxt["clfInitialValue"]
        pairs += 1
        scale = max(v0, 1.0)
        if now["clfSlack"] <= tolerance * scale:
            zero += 1
            if v1 - v0 > -now["clfRequiredDecrease"] + tolerance * scale:
                missed += 1
            if v1 - v0 > tolerance * scale:
                increased += 1
                largest = max(largest, v1 - v0)
    for key, value in (("pairs", pairs), ("zeroSlackPairs", zero), ("decreaseNotAchieved", missed),
                       ("valueIncreased", increased)):
        clf[key] += value
    clf["largestIncrease"] = max(clf["largestIncrease"], largest)
    for hold in trace:
        attempts = hold["attempts"] if isinstance(hold["attempts"], list) else [hold["attempts"]]
        parts["initialization"].append(sum(a["initializationSeconds"] for a in attempts))
        parts["formulation"].append(sum(a["formulationSeconds"] for a in attempts))
        parts["solver"].append(sum(stage["seconds"] for stage in hold["solverStages"]))
    calls += seconds
    recovery = result["recovery"]
    rows.append({
        "speed": record["speed"], "scenario": record["scenario"], "sourceSha256": digest,
        "frames": len(trace), "failure": result["failure"], "failureTime": result["failureTime"],
        "horizonSteps": sorted({hold["horizonSteps"] for hold in trace}),
        "minimumBodyDistanceMeters": minimum, "minimumBodyDistanceTime": minimum_time,
        "contactInterval": [min(contact), max(contact)] if contact else None,
        "recovered": recovery["recovered"], "recoveryConfirmationSeconds": recovery["confirmationTimeSeconds"],
        "maximumAbsLateralErrorMeters": max(abs(hold["transverseError"][0]) for hold in trace),
        "minimumRoadMarginMeters": result["minimumReplayRoadMarginMeters"],
        "maximumAbsSteeringRadians": max(abs(hold["input"][0]) for hold in trace),
        "minimumForceRatio": min(hold["input"][1] for hold in trace),
        "maximumForceRatio": max(hold["input"][1] for hold in trace),
        "freshAnchorSamples": sum(1 for hold in trace if hold["flowRestarted"]),
        "freshAnchorRestoredZero": sum(1 for hold in trace if hold["flowRestarted"]
                                       and (hold["primaryOptimum"] or 0) <= tolerance),
        "maximumAbsYawRate": max(abs(state[5]) for hold in trace for state in hold["auditStates"]),
        "threeSolveSamples": sum(1 for hold in trace if hold["solverCalls"] == 3),
        "positivePrimarySamples": len(positive),
        "positiveWithoutFreshAnchor": sum(1 for hold in positive if not hold["flowRestarted"]),
        "positiveAtLowerBound": sum(1 for hold in positive
                                    if abs(hold["primaryOptimum"] - hold["primaryLowerBound"]) <= tolerance),
        "positivePrimaryTimes": [positive[0]["time"], positive[-1]["time"]] if positive else None,
        "maximumPrimaryOptimum": max((hold["primaryOptimum"] or 0) for hold in trace),
        "nonconvergedSamples": sum(1 for hold in trace if not hold["optimizationConverged"]),
        "solverCalls": sum(hold["solverCalls"] for hold in trace),
        "maximumSolverCallsPerSample": max(hold["solverCalls"] for hold in trace),
        "medianCallSeconds": statistics.median(seconds),
        "maximumCallSeconds": max(seconds),
        "maximumCallSecondsExcludingFirstAndResume": max(s for k, s in enumerate(seconds) if k not in (0, resume)),
        "callsOver50ms": sum(1 for s in seconds if s > 0.05),
        "callsOver100ms": sum(1 for s in seconds if s > 0.10),
        "baselineFirstCollisionSeconds": result["baselineCruise"]["firstCollisionSeconds"],
        "baselineInitialClearanceMeters": result["baselineCruise"]["initialClearanceMeters"],
    })

successful = [r for r in rows if r["recovered"] and r["minimumBodyDistanceMeters"] > 0]
baseline = [r for r in comparison["results"] if r["version"] == "baseline"]
summary = {
    "runs": len(rows),
    "collisionFreeAndRecovered": len(successful),
    "collisions": sum(1 for r in rows if r["minimumBodyDistanceMeters"] <= 0),
    "interrupted": sum(1 for r in rows if r["failure"]),
    "successfulRunsBelowNodeMargin": sum(1 for r in successful if r["minimumBodyDistanceMeters"] < 0.10),
    "subMarginSuccessesWithExecutedRelaxation": sum(1 for r in successful if r["minimumBodyDistanceMeters"] < 0.10
                                                   and r["positivePrimarySamples"] > 0),
    "freshAnchorSamples": sum(r["freshAnchorSamples"] for r in rows),
    "freshAnchorRestoredZero": sum(r["freshAnchorRestoredZero"] for r in rows),
    "maximumAbsYawRate": max(r["maximumAbsYawRate"] for r in rows),
    "runsExceedingEstimatorYawRateDomain": sum(1 for r in rows if r["maximumAbsYawRate"] > 0.30),
    "threeSolveSamples": sum(r["threeSolveSamples"] for r in rows),
    "maximumSolverCallsPerSample": max(r["maximumSolverCallsPerSample"] for r in rows),
    "smallestSuccessfulDistanceMeters": min(r["minimumBodyDistanceMeters"] for r in successful),
    "recoveryRangeSeconds": [min(r["recoveryConfirmationSeconds"] for r in successful),
                             max(r["recoveryConfirmationSeconds"] for r in successful)],
    "lateralDeviationRangeMeters": [min(r["maximumAbsLateralErrorMeters"] for r in rows),
                                    max(r["maximumAbsLateralErrorMeters"] for r in rows)],
    "runsLeavingFourMeterCorridor": sum(1 for r in rows if r["minimumRoadMarginMeters"] < 0),
    "maximumAbsSteeringRadians": max(r["maximumAbsSteeringRadians"] for r in rows),
    "forceRatioRange": [min(r["minimumForceRatio"] for r in rows), max(r["maximumForceRatio"] for r in rows)],
    "positivePrimarySamples": sum(r["positivePrimarySamples"] for r in rows),
    "positiveWithoutFreshAnchor": sum(r["positiveWithoutFreshAnchor"] for r in rows),
    "positiveAtLowerBound": sum(r["positiveAtLowerBound"] for r in rows),
    "runsWithPositivePrimary": sum(1 for r in rows if r["positivePrimarySamples"]),
    "positivePrimaryTimeRange": [min(r["positivePrimaryTimes"][0] for r in rows if r["positivePrimaryTimes"]),
                                 max(r["positivePrimaryTimes"][1] for r in rows if r["positivePrimaryTimes"])],
    "controllerCalls": len(calls),
    "medianCallSeconds": statistics.median(calls),
    "p95CallSeconds": percentile(calls, 0.95),
    "maximumCallSeconds": max(calls),
    "maximumCallSecondsExcludingFirstAndResume": max(r["maximumCallSecondsExcludingFirstAndResume"] for r in rows),
    "callsOver50ms": sum(r["callsOver50ms"] for r in rows),
    "callsOver100ms": sum(r["callsOver100ms"] for r in rows),
    "solverShareOfCallTime": sum(parts["solver"]) / sum(calls),
    "formulationShareOfCallTime": sum(parts["formulation"]) / sum(calls),
    "initializationShareOfCallTime": sum(parts["initialization"]) / sum(calls),
    "nonconvergedSamples": sum(r["nonconvergedSamples"] for r in rows),
    "clf": clf,
    "baselineFirstCollisionRangeSeconds": [min(r["baselineFirstCollisionSeconds"] for r in rows),
                                           max(r["baselineFirstCollisionSeconds"] for r in rows)],
    "precedingSeed": {
        "completedEightSeconds": sum(1 for r in baseline if r["eightSeconds"]["completed"]),
        "stoppedWithoutPrimarySolution": sum(1 for r in baseline if "pcbfNoNumericalResult" in r["eightSeconds"]["failure"]),
        "stoppedOnDistanceDualCheck": sum(1 for r in baseline if "distanceDualFailed" in r["eightSeconds"]["failure"]),
        "interruptedAfterCollision": sum(1 for r in baseline if r["eightSeconds"]["failure"]
                                         and not r["eightSeconds"]["strictlyCollisionFree"]),
    },
}
output.write_text(json.dumps({"scope": "Derived from recorded traces; no simulation run", "summary": summary,
                              "runs": rows}, indent=1) + "\n")
print(json.dumps(summary, indent=1))
