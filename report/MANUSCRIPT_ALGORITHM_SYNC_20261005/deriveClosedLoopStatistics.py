#!/usr/bin/env python3
"""Closed-loop statistics quoted in paper/manuscript.tex (October 5, 2026 sync).

Reads the recorded per-frame traces of the campaign of
report/ESTIMATOR_DIRECT_TARGET_20261005.tex (controller d1c31d6); runs no
simulation. Usage: deriveClosedLoopStatistics.py <campaign root> <output json>
"""
import json
import statistics
import sys
from pathlib import Path

root = Path(sys.argv[1]).expanduser()
TOL = 1e-4 + 1e-5  # lexicographicTieTolerance + feasibilityTolerance (zeroSlack)


def quantile(values, q):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


def run_statistics(path):
    result = json.loads(path.read_text())["results"]
    if isinstance(result, list):
        result = result[0]
    trace = result["trace"]
    inputs = [h["input"] for h in trace]
    states = [s for h in trace for s in h["auditStates"]]
    slack = [h["clfSlack"] <= 1e-5 * max(1.0, h["clfInitialValue"]) for h in trace]
    pairs = list(zip(trace[:-1], trace[1:]))
    rho = result["configuration"]["controller"]["sampleTime"]
    rho = pow(2.718281828459045, -2 * rho / result["configuration"]["clf"]["convergenceTimeConstantSeconds"])
    zero = [(a, b) for a, b in pairs if a["clfSlack"] <= 1e-5 * max(1.0, a["clfInitialValue"])]
    return {
        "outcome": result["outcome"], "holds": len(trace),
        "minimumGap": result["minimumReplayClearanceMeters"],
        "minimumRoadMargin": result["minimumReplayRoadMarginMeters"],
        "maximumLateralDeviation": result["maximumReplayLateralDeviationMeters"],
        "recoverySeconds": result["recovery"].get("confirmedTimeSeconds") if isinstance(result["recovery"], dict) else None,
        "maximumSteering": max(abs(u[0]) for u in inputs),
        "brakingRatioRange": [min(u[1] for u in inputs), max(u[1] for u in inputs)],
        "maximumYawRate": max(abs(s[5]) for s in states),
        "maximumSideslip": max(abs(s[4] / s[3]) for s in states),
        "positivePrimaryFrames": sum(h["primaryOptimum"] > TOL for h in trace),
        "maximumPrimaryOptimum": max(h["primaryOptimum"] for h in trace),
        "freshRestarts": sum(bool(h["potentialFieldRestarted"]) for h in trace),
        "horizonMedian": statistics.median(h["horizonSteps"] for h in trace),
        "horizonMaximum": max(h["horizonSteps"] for h in trace),
        "zeroClfSlackFrames": sum(slack),
        "zeroSlackPairs": len(zero),
        "zeroSlackPairsMissingDecrease": sum(b["clfInitialValue"] > rho * a["clfInitialValue"] + 1e-9 for a, b in zero),
        "zeroSlackPairsIncreasing": sum(b["clfInitialValue"] > a["clfInitialValue"] + 1e-9 for a, b in zero),
        "trustScaleRange": [min(h["inputTrustScale"] for h in trace), max(h["inputTrustScale"] for h in trace)],
        "controllerSeconds": [h["controllerSeconds"] for h in trace],
        "observerSeconds": [h.get("observerSeconds") or 0 for h in trace],
    }


summary = {}
for group in sorted(p for p in root.iterdir() if p.is_dir() and (p.name == "exact" or p.name.startswith("noisy"))):
    runs = {p.stem: run_statistics(p) for p in sorted(group.glob("speed*.json")) if "summary" not in p.name}
    seconds = [s for r in runs.values() for s in r.pop("controllerSeconds")]
    observer = [s for r in runs.values() for s in r.pop("observerSeconds")]
    summary[group.name] = {
        "runs": runs, "frames": len(seconds),
        "controllerMedian": statistics.median(seconds), "controllerP95": quantile(seconds, .95),
        "controllerMaximum": max(seconds), "shareOver50ms": sum(s > .05 for s in seconds) / len(seconds),
        "observerMedian": statistics.median(observer),
    }
Path(sys.argv[2]).write_text(json.dumps(summary, indent=1))
for name, group in summary.items():
    r = group["runs"].values()
    print(name, group["frames"], "ctrl med %.4f p95 %.4f max %.3f over50 %.3f obs %.4f" % (
        group["controllerMedian"], group["controllerP95"], group["controllerMaximum"], group["shareOver50ms"], group["observerMedian"]),
        "posPrimary", sum(x["positivePrimaryFrames"] for x in r), "restarts", sum(x["freshRestarts"] for x in r),
        "steer %.3f yawrate %.3f slip %.3f" % (max(x["maximumSteering"] for x in r), max(x["maximumYawRate"] for x in r), max(x["maximumSideslip"] for x in r)),
        "b [%.2f,%.2f]" % (min(x["brakingRatioRange"][0] for x in r), max(x["brakingRatioRange"][1] for x in r)),
        "ey %.2f-%.2f" % (min(x["maximumLateralDeviation"] for x in r), max(x["maximumLateralDeviation"] for x in r)),
        "zeroPairs", sum(x["zeroSlackPairs"] for x in r), sum(x["zeroSlackPairsMissingDecrease"] for x in r), sum(x["zeroSlackPairsIncreasing"] for x in r),
        "H", max(x["horizonMaximum"] for x in r))
