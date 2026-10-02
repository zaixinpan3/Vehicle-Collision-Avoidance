#!/usr/bin/env python3
"""Check the numbers and structure of paper/manuscript.tex against their records.

Every quoted number is recomputed from a committed record (or from a file of
this bundle that was derived from one) and must appear in the manuscript in
the stated format. Structural checks cover labels, references, citations,
environment balance and the spans that this revision leaves untouched.

Usage: python3 auditManuscript.py <repository root> <output.json> [<preceding revision>]
The optional third argument names the Git revision that holds the preceding
manuscript; with it, the spans that this revision leaves untouched are compared.
For this revision it is 5692e1551241b568bcfb300093814373c674a412.
Exit status is nonzero if any check fails.
"""
import csv
import json
import re
import subprocess
import sys
from pathlib import Path

repo = Path(sys.argv[1]).resolve()
output = Path(sys.argv[2])
bundle = Path(__file__).resolve().parent
manuscript = (repo / "paper/manuscript.tex").read_text()
flat = re.sub(r"\s+", " ", manuscript)
checks = []


def record(name, passed, detail=""):
    checks.append({"check": name, "passed": bool(passed), "detail": detail})


def present(name, text):
    """The literal text (whitespace-normalized) must occur in the manuscript."""
    record(name, re.sub(r"\s+", " ", text) in flat, text)


def rows(path):
    with open(path, newline="") as handle:
        return list(csv.DictReader(handle))


# ----------------------------------------------------------------- estimator design
design = json.loads((bundle / "estimator-design.json").read_text())
ultimate = design["ultimateBounds"]
present("scalar gains", f"{design['velocityGain']:.4f}")
assert design["velocityGain"] == design["yawGain"] == design["positionGain"]
present("Lipschitz L_q", f"L_q={design['lipschitzVelocity']:.4f}")
present("Lipschitz L_s", f"L_s={design['lipschitzAcceleration']:.4f}")
present("shape slowest rate", f"\\omega_0={design['shapeSlowestRate']:.4f}")
present("shape", "[" + ",".join(f"{v:.4f}" for v in design["shape"]) + "]")
present("bandwidth", f"\\omega={design['bandwidth']:.4f}")
present("innovation gains", "[" + ",".join(f"{v:.4f}" for v in design["innovationGains"]) + "]")
H = design["comparisonMatrix"]
present("comparison matrix", f"{H[0][0]:.4f}&0\\\\{H[1][0]:.4f}&{H[1][1]:.4f}")
d = design["comparisonInput"]
present("comparison input", f"{d[0]:.5f}\\\\{d[1]:.5f}")
present("body-velocity radius", f"{ultimate['bodyVelocity']:.5f}\\,m/s")
present("position radius", f"{ultimate['position']:.5f}\\,m")
present("target radii", f"{ultimate['relativePosition']:.4f}\\,m, {ultimate['targetVelocity']:.3f}\\,m/s and "
        f"{ultimate['targetAcceleration']:.2f}\\,m/s$^2$")
present("ego rear-axle distance", f"l_{{r,E}}={design['egoRearAxleDistance']:.3f}")
present("domain constants", f"\\Omega_{{\\max}}={design['targetDomain']['yawRateMaximum']:.4f}")
present("acceleration norm bound", f"S={design['targetDomain']['accelerationNormBound']:.4f}")
record("certificate residual margin positive", design["lipschitzResidualMargin"] > 0,
       str(design["lipschitzResidualMargin"]))

table = rows(repo / "report/TARGET_NOISE_SHAPE_GAINS_20261001/design_table.csv")
previous = next(r for r in table if r["Scenario"] == "lowRelativeVelocity" and "previous" in r["Synthesis"])
current = next(r for r in table if r["Scenario"] == "lowRelativeVelocity" and "this change" in r["Synthesis"])
record("design table reproduces today's synthesis",
       abs(float(current["bandwidth"]) - design["bandwidth"]) < 1e-4
       and abs(float(current["positionUltimateM"]) - ultimate["relativePosition"]) < 1e-4)
present("preceding gains", "[" + ",".join(f"{float(previous[k]):.2f}" for k in ("gain1", "gain2", "gain3")) + "]")
present("preceding radii", f"{float(previous['positionUltimateM']):.3f}\\,m, "
        f"{float(previous['velocityUltimateMps']):.2f}\\,m/s and "
        f"{float(previous['accelerationUltimateMps2']):.2f}\\,m/s$^2$")
present("predicted noise, bounded-real shape",
        f"${float(previous['predictedVelocityRmsMps']):.3f}$\\,m/s for the bounded-real shape")
present("predicted noise, noise-variance shape",
        f"${float(current['predictedVelocityRmsMps']):.3f}$\\,m/s for the noise-variance shape")
circular = next(r for r in table if r["Scenario"] == "circularFollowing" and "this change" in r["Synthesis"])
present("L_q in the 0.03 rad sideslip domain", f"from {float(current['phiVelocity']):.2f} to "
        f"{float(circular['phiVelocity']):.2f}")

# ----------------------------------------------------------- estimator comparison
summary = rows(repo / "report/TARGET_NOISE_SHAPE_GAINS_20261001/audit/summary.csv")
names = {"straightOncoming": "Straight oncoming", "circularCrossing": "Arc crossing",
         "circularFollowing": "Arc following", "avoidanceSwerve": "Swerve and braking",
         "lowRelativeVelocity": "Low relative speed", "aggressiveEgo": "Ego weaving"}
wins = {"relativePositionRmse": 0, "targetSpeedRmse": 0, "courseRmseDeg": 0}
global_wins = 0
for key, label in names.items():
    def pick(estimator):
        return next(r for r in summary if r["Scenario"] == key and r["Variant"] == "nominal"
                    and r["Estimator"] == estimator)
    ours, tuned, single = pick("structured"), pick("sharmaScenarioTunedPredictor"), pick("sharmaTunedPredictor")
    assert ours["Trials"] == tuned["Trials"] == "20" and ours["Failed"] == tuned["Failed"] == "0"
    cells = [f"{float(ours['relativePositionRmse']):.4f}", f"{float(tuned['relativePositionRmse']):.4f}",
             f"{float(ours['targetSpeedRmse']):.4f}", f"{float(tuned['targetSpeedRmse']):.4f}",
             f"{float(ours['courseRmseDeg']):.3f}", f"{float(tuned['courseRmseDeg']):.3f}"]
    dagger = r"(?:\$\^\\dagger\$)?"
    match = re.search(re.escape(label) + r"\s*&\s*" + (dagger + r"\s*&\s*").join(map(re.escape, cells)), manuscript)
    record(f"comparison table row: {label}", match is not None, " & ".join(cells))
    for metric in wins:
        wins[metric] += float(ours[metric]) < float(tuned[metric])
        global_wins += float(ours[metric]) < float(single[metric])
record("lower mean in five of six scenarios per metric", all(v == 5 for v in wins.values()), str(wins))


def design_row(scenario):
    return next(r for r in table if r["Scenario"] == scenario and "this change" in r["Synthesis"])


def gains(row, digits):
    return "[" + ",".join(f"{float(row[k]):.{d}f}" for k, d in zip(("gain1", "gain2", "gain3"), digits)) + "]"


present("curved-following gains", gains(design_row("circularFollowing"), (2, 2, 1)))
present("gains for the 100 m domains", gains(design_row("straightOncoming"), (2, 2, 2)))
record("crossing and swerve share the 100 m design",
       gains(design_row("circularCrossing"), (2, 2, 2)) == gains(design_row("straightOncoming"), (2, 2, 2))
       == gains(design_row("avoidanceSwerve"), (2, 2, 2)))
present("weaving gains", gains(design_row("aggressiveEgo"), (2, 2, 2)))
record("decay floors 0.4 and 0.5", float(design_row("straightOncoming")["decayFloorPerS"]) == 0.4
       and float(design_row("aggressiveEgo")["decayFloorPerS"]) == 0.5)
import math
present("scalar gains of the wider domains", f"{math.log(20) / 2.5:.3f} and {math.log(20) / 2.0:.3f}")
extra = {"targetAccelerationRmse": 0, "fullPositionRmse": 0, "fullSpeedRmse": 0}
noise_free = {"relativePositionRmse": 0, "targetSpeedRmse": 0, "courseRmseDeg": 0}
stress = {"relativePositionRmse": 0, "targetSpeedRmse": 0, "courseRmseDeg": 0}
stress_cells = 0
for key in names:
    def pick(variant, estimator):
        return next(r for r in summary if r["Scenario"] == key and r["Variant"] == variant
                    and r["Estimator"] == estimator)
    for metric in extra:
        extra[metric] += float(pick("nominal", "structured")[metric]) < float(
            pick("nominal", "sharmaScenarioTunedPredictor")[metric])
    for metric in noise_free:
        noise_free[metric] += float(pick("noiseFree", "sharmaTunedPredictor")[metric]) < float(
            pick("noiseFree", "structured")[metric])
    for variant in ("kinematicMismatch", "largeOffset", "noise2x", "radarDropout", "sample100Hz", "sample20Hz"):
        stress_cells += 1
        assert pick(variant, "structured")["Trials"] == "5"
        for metric in stress:
            stress[metric] += float(pick(variant, "structured")[metric]) < float(
                pick(variant, "sharmaTunedPredictor")[metric])
record("acceleration lower in two, full-run position in none, full-run speed in six",
       (extra["targetAccelerationRmse"], extra["fullPositionRmse"], extra["fullSpeedRmse"]) == (2, 0, 6), str(extra))
record("noise-free: reference lower in 4 position, 2 speed, 1 course cells",
       (noise_free["relativePositionRmse"], noise_free["targetSpeedRmse"], noise_free["courseRmseDeg"]) == (4, 2, 1),
       str(noise_free))
record("stress cells: 36 cells, position 34, speed 36, course 36",
       (stress_cells, stress["relativePositionRmse"], stress["targetSpeedRmse"], stress["courseRmseDeg"])
       == (36, 34, 36, 36), str(stress))
swerve = next(r for r in summary if r["Scenario"] == "avoidanceSwerve" and r["Variant"] == "nominal"
              and r["Estimator"] == "structured")
swerve_reference = next(r for r in summary if r["Scenario"] == "avoidanceSwerve" and r["Variant"] == "nominal"
                        and r["Estimator"] == "sharmaScenarioTunedPredictor")
present("swerve speed RMSE", f"{float(swerve['targetSpeedRmse']):.3f} against "
        f"{float(swerve_reference['targetSpeedRmse']):.3f}\\,m/s")
trials = rows(repo / "report/TARGET_NOISE_SHAPE_GAINS_20261001/audit/trials.csv")
tuned_trials = rows(repo / "report/TARGET_NOISE_SHAPE_GAINS_20261001/audit/scenario-tuned.csv")


def convergence(source, scenario, estimator):
    values = [float(r["convergenceTime"]) for r in source if r["Scenario"] == scenario
              and r["Variant"] == "nominal" and r["Estimator"] == estimator]
    finite = [v for v in values if math.isfinite(v)]
    return (sum(finite) / len(finite) if finite else math.nan), len(finite), len(values)


ours = [convergence(trials, key, "structured") for key in names]
record("estimator converges in every nominal trial", all(f == t == 20 for _, f, t in ours))
from decimal import Decimal, ROUND_HALF_UP


def half_up(value):
    """Two decimals, rounding an exact tie upward as the source report does."""
    return str(Decimal(repr(round(value, 9))).quantize(Decimal("0.01"), rounding=ROUND_HALF_UP))


present("estimator convergence range", f"{half_up(min(m for m, _, _ in ours))} to "
        f"{half_up(max(m for m, _, _ in ours))}\\,s")
reference = {key: convergence(tuned_trials, key, "sharmaScenarioTunedPredictor") for key in names}
always = [m for m, f, t in reference.values() if f == t]
present("reference convergence range", f"{min(always):.2f} to {max(always):.2f}\\,s")
record("reference converges in 14 of 20 swerve trials and no weaving trial",
       reference["avoidanceSwerve"][1] == 14 and reference["aggressiveEgo"][1] == 0 and len(always) == 4)
record("lower mean than the single tuning in all 18 cells", global_wins == 18, str(global_wins))
intervals = rows(repo / "report/TARGET_NOISE_SHAPE_GAINS_20261001/audit/paired_intervals.csv")
tie = next(r for r in intervals if r["Scenario"] == "circularFollowing"
           and r["Comparator"] == "sharmaScenarioTunedPredictor" and r["Metric"] == "courseRmseDeg")
present("curved-following course interval", f"[{float(tie['Ci95Low']):.3f},{float(tie['Ci95High']):.3f}]")
containing_zero = sorted((r["Scenario"], r["Metric"]) for r in intervals
                         if r["Comparator"] == "sharmaScenarioTunedPredictor"
                         and r["Metric"] in ("relativePositionRmse", "targetSpeedRmse", "courseRmseDeg")
                         and float(r["Ci95Low"]) <= 0 <= float(r["Ci95High"]))
record("dagger marks: exactly two paired intervals contain zero",
       containing_zero == [("circularCrossing", "relativePositionRmse"), ("circularFollowing", "courseRmseDeg")]
       and manuscript.count("$^\\dagger$") == 3, str(containing_zero))
oracle = rows(repo / "report/TARGET_NOISE_SHAPE_GAINS_20261001/audit/oracle_summary.csv")
pair = [next(r for r in oracle if r["Scenario"] == "straightOncoming" and r["Estimator"] == e)
        for e in ("structured", "sharmaSameInjection")]
present("oracle agreement", f"{float(pair[0]['targetSpeedRmse']):.6f} and {float(pair[1]['targetSpeedRmse']):.6f}")
ratios = {}
for scenario in ("straightOncoming", "circularCrossing", "circularFollowing", "lowRelativeVelocity"):
    covariant, companion = [float(next(r for r in oracle if r["Scenario"] == scenario and r["Estimator"] == e)
                                  ["targetSpeedRmse"]) for e in ("structured", "sharmaSameInjection")]
    ratios[scenario] = abs(covariant - companion) / companion
    if scenario == "circularCrossing":
        present("oracle difference in curved crossing", f"{covariant:.3f} against {companion:.3f}\\,m/s")
record("oracle agreement within 0.2% in three of four scenarios",
       sum(1 for v in ratios.values() if v < 0.002) == 3 and ratios["circularCrossing"] > 0.1, str(ratios))
benchmark = rows(repo / "report/TARGET_NOISE_SHAPE_GAINS_20261001/benchmark_summary.csv")
new = next(r for r in benchmark if r["Case"] == "retained-noise" and r["Estimator"] == "structured-high-gain")
old = next(r for r in benchmark if r["Case"] == "retained-noise" and r["Estimator"] == "baseline-high-gain")
present("paired benchmark", f"{float(new['mean_PositionRmseM']):.4f} against {float(old['mean_PositionRmseM']):.4f}\\,m and "
        f"{float(new['mean_VelocityRmseMps']):.3f} against {float(old['mean_VelocityRmseMps']):.3f}\\,m/s")
noisy = sorted({r["Case"] for r in benchmark if r["Case"] != "retained-noise-free"})
better = sum(
    float(next(r for r in benchmark if r["Case"] == c and r["Estimator"] == "structured-high-gain")[m])
    < float(next(r for r in benchmark if r["Case"] == c and r["Estimator"] == "baseline-high-gain")[m])
    for c in noisy for m in ("mean_PositionRmseM", "mean_VelocityRmseMps"))
record("benchmark: lower position and velocity RMSE in all ten noisy cases",
       len(noisy) == 10 and better == 20, f"{len(noisy)} cases, {better} of 20 comparisons")

# ------------------------------------------------------------ controller constants
constants = json.loads((bundle / "controller-constants.json").read_text())
slow, fast = constants["speed8"], constants["speed15"]
present("terminal radius at 8 m/s", f"r_B={slow['terminalRadius'] * 1e4:.1f}\\times10^{{-4}}")
present("terminal radius at 15 m/s", f"${fast['terminalRadius'] * 1e4:.1f}\\times10^{{-4}}$ at 15")
present("contraction bounds", f"\\gamma_c={slow['contractionBound']:.4f}$ and ${fast['contractionBound']:.4f}")
present("horizon lengths", f"8, {slow['horizonSteps']}; 16, {fast['horizonSteps']}")
radii = [constants[s][p]["tailSpectralRadius"] for s in ("speed8", "speed15") for p in ("straight", "curved")]
present("spectral radius of A_pi", f"{min(radii):.3f} to {max(radii):.3f}")
record("one RK4 step per hold", slow["integrationStepsPerHold"] == fast["integrationStepsPerHold"] == 1)
record("K = 2400", slow["policyEvaluationSteps"] == 2400)
reach = fast["egoCircumradius"]
yaw_box = 0.5 * fast["trustRadius"]
present("ego circumradius", f"r_E={reach:.2f}")
present("yaw trust box", f"|\\Delta\\psi|\\leq{yaw_box:.2f}")
present("affine separation bound", f"${0.10 - 0.5 * reach * yaw_box ** 2:.3f}$\\,m")

# --------------------------------------------------------------- closed-loop table
closed = json.loads((bundle / "closed-loop-statistics.json").read_text())
total = closed["summary"]
order = ["headOn", "acceleratingHeadOn", "brakingLead", "crossing", "turningCrossing", "curvedHeadOn",
         "curvedCrossing"]
labels = ["Head-on", "Accelerating head-on", "Braking lead", "Crossing", "Turning crossing",
          "Curved head-on", "Curved crossing"]
for scenario, label in zip(order, labels):
    cells = []
    for speed in (8, 15):
        run = next(r for r in closed["runs"] if r["speed"] == speed and r["scenario"] == scenario)
        gap = (f"{run['minimumBodyDistanceMeters']:.3f}" if run["minimumBodyDistanceMeters"] > 0
               else r"collision$^{\rm b}$")
        recovery = (f"{run['recoveryConfirmationSeconds']:.2f}" if run["recovered"] else r"stopped$^{\rm a}$")
        cells += [gap, recovery, f"{run['maximumAbsLateralErrorMeters']:.2f}", str(run["positivePrimarySamples"]),
                  f"{run['medianCallSeconds']:.3f}", f"{run['maximumCallSeconds']:.3f}"]
    match = re.search(re.escape(label) + r"\s*&\s*" + r"\s*&\s*".join(map(re.escape, cells)), manuscript)
    record(f"closed-loop table row: {label}", match is not None, " & ".join(cells))

record("twelve collision-free with recovery, one collision, one interruption",
       (total["collisionFreeAndRecovered"], total["collisions"], total["interrupted"]) == (12, 1, 1))
present("recovery range", f"{total['recoveryRangeSeconds'][0]:.2f} and {total['recoveryRangeSeconds'][1]:.2f}\\,s")
record("seven successful runs below the node margin", total["successfulRunsBelowNodeMargin"] == 7)
record("four of them with executed relaxations", total["subMarginSuccessesWithExecutedRelaxation"] == 4)
present("fresh-anchor counts", f"formulated at {total['freshAnchorSamples']} samples and succeeds at "
        f"{total['freshAnchorRestoredZero']}")
record("positive samples reconcile: unrestored fresh anchors plus samples at the lower bound",
       total["positivePrimarySamples"] == total["freshAnchorSamples"] - total["freshAnchorRestoredZero"]
       + total["positiveWithoutFreshAnchor"] and total["positiveWithoutFreshAnchor"] == 2
       and total["positiveAtLowerBound"] >= 2,
       f"{total['positivePrimarySamples']} = {total['freshAnchorSamples']} - {total['freshAnchorRestoredZero']} + "
       f"{total['positiveWithoutFreshAnchor']}")
present("largest ego yaw rate", f"{total['maximumAbsYawRate']:.2f}\\,rad/s")
record("at most three solves per sample", total["maximumSolverCallsPerSample"] == 3)
present("smallest successful distance", f"{total['smallestSuccessfulDistanceMeters']:.3f}\\,m")
present("lateral deviation range", f"{total['lateralDeviationRangeMeters'][0]:.1f} to "
        f"{total['lateralDeviationRangeMeters'][1]:.1f}\\,m")
record("twelve runs leave the 4 m corridor", total["runsLeavingFourMeterCorridor"] == 12)
present("largest steering angle", f"{total['maximumAbsSteeringRadians']:.2f}\\,rad")
present("force-ratio range", f"${total['forceRatioRange'][0]:.2f}$ to ${total['forceRatioRange'][1]:.2f}$")
record("22 positive-optimum samples in five runs",
       (total["positivePrimarySamples"], total["runsWithPositivePrimary"]) == (22, 5))
present("positive-optimum time range", f"{total['positivePrimaryTimeRange'][0]:.2f} and "
        f"{total['positivePrimaryTimeRange'][1]:.2f}\\,s")
present("controller calls", f"All {total['controllerCalls']} calls exceed")
record("every call exceeds the hold", total["callsOver50ms"] == total["controllerCalls"])
present("call-time statistics", f"the median is {total['medianCallSeconds']:.3f}\\,s and the 95th percentile "
        f"{total['p95CallSeconds']:.3f}\\,s. The maximum of {total['maximumCallSeconds']:.3f}\\,s")
present("largest call outside process start",
        f"the largest other call takes {total['maximumCallSecondsExcludingFirstAndResume']:.3f}\\,s")
record("solver share about two thirds", 0.6 < total["solverShareOfCallTime"] < 0.72,
       f"{total['solverShareOfCallTime']:.3f}")
present("non-converged executed solutions", f"At {total['nonconvergedSamples']} samples")
clf = total["clf"]
present("CLF pairs", f"Of {clf['pairs']} consecutive state pairs, {clf['zeroSlackPairs']} have zero")
present("CLF misses", f"In {clf['decreaseNotAchieved']} of those")
present("CLF increases", f"in {clf['valueIncreased']} the value increases, by at most {clf['largestIncrease']:.3f}")
present("cruise-collision times", f"{total['baselineFirstCollisionRangeSeconds'][0]:.2f} and "
        f"{total['baselineFirstCollisionRangeSeconds'][1]:.2f}\\,s")
seed = total["precedingSeed"]
record("preceding seed: 10 of 14, two and two stops, one after collision",
       (seed["completedEightSeconds"], seed["stoppedWithoutPrimarySolution"], seed["stoppedOnDistanceDualCheck"],
        seed["interruptedAfterCollision"]) == (10, 2, 2, 1), json.dumps(seed))
collision = next(r for r in closed["runs"] if r["minimumBodyDistanceMeters"] <= 0)
present("contact interval", f"{collision['contactInterval'][0]:.3f} to {collision['contactInterval'][1]:.3f}\\,s")

# ------------------------------------------------------------- failure mechanisms
findings = json.loads((repo / "report/CONTROLLER_FAILURE_CAUSES_20261002/findings.json").read_text())
record("exact replay: 31 calls with identical inputs",
       len(findings["exactTurningReplay"]) == 31
       and all(entry["inputDifference"] == 0 for entry in findings["exactTurningReplay"]))
for hold in (0.0, 0.75, 0.85, 0.90, 1.05, 1.10):
    entry = next(e for e in findings["turningFrameModels"] if abs(e["time"] - hold) < 1e-9)
    def cell(value):
        return f"{value:.3f}" if value >= 0 else f"$-{abs(value):.3f}$"
    primary = entry["pcbf"]
    if primary > 1e-3:
        optimum = f"{primary:.3f}"
    else:
        mantissa, exponent = f"{primary:.1e}".split("e")
        optimum = f"${mantissa}\\times10^{{{int(exponent)}}}$"
    cells = [f"{hold:.2f}", cell(entry["affineMinimum"]), cell(entry["nonlinearMinimum"]), optimum]
    match = re.search(r"\s*&\s*".join(map(re.escape, cells)), manuscript)
    record(f"turning-crossing table row at {hold:.2f} s", match is not None, " & ".join(cells))
braking = findings["brakingCounterfactual"]
present("counterfactual recovery", f"{braking['recovery']['confirmationTimeSeconds']:.2f}\\,s with a smallest distance of "
        f"{braking['minimumReplayClearanceMeters']:.3f}\\,m")
present("interruption time", f"{findings['brakingOriginal']['failureTime']:.2f}\\,s")
present("V_K before the interruption",
        "(" + ", ".join(f"{entry['value']:.2f}" for entry in findings["brakingOriginal"]["lastClf"]) + ")")
failure_report = (repo / "report/CONTROLLER_FAILURE_CAUSES_20261002.tex").read_text()
for name, source, quoted in [
        ("affine row value", "0.099998997", "0.1000\\,m"),
        ("nonlinear row value", "-0.0452861", "$-0.045$\\,m"),
        ("pose position error", "0.07057", "0.071\\,m"),
        ("separation removed by position", "0.07043", "0.070\\,m"),
        ("pose heading error", "0.03139", "0.031\\,rad"),
        ("separation removed by heading", "0.07485", "0.075\\,m"),
        ("tangent force", "3225", "3225\\,N"),
        ("tire-model force", "1365", "1365\\,N"),
        ("primary clearance at 0.85 s", "0.09293", "0.093\\,m"),
        ("interpolated affine distance", "0.006911", "0.007\\,m"),
        ("flow rollout lateral position", "3.02530", "3.03\\,m"),
        ("guide lateral position", "5.75143", "5.75\\,m"),
        ("largest penetration", "0.152024", "0.152\\,m"),
        ("body distance at the interrupted anchor", "27.633427417605", "27.63\\,m"),
        ("primal-dual difference", "1.675604", "1.68\\times10^{-6}"),
]:
    record(f"failure analysis: {name}", source in failure_report and quoted in manuscript, f"{source} -> {quoted}")

# -------------------------------------------------------------------- structure
labels_defined = re.findall(r"\\label\{([^}]+)\}", manuscript)
references = set(re.findall(r"\\(?:eq)?ref\{([^}]+)\}", manuscript))
record("labels are unique", len(labels_defined) == len(set(labels_defined)), f"{len(labels_defined)} labels")
record("every reference has a label", references <= set(labels_defined), str(sorted(references - set(labels_defined))))
bibliography = set(re.findall(r"@\w+\{([^,]+),", (repo / "paper/references.bib").read_text()))
cited = {key.strip() for group in re.findall(r"\\cite\{([^}]+)\}", manuscript) for key in group.split(",")}
record("every citation key is in references.bib", cited <= bibliography, str(sorted(cited - bibliography)))
for environment in sorted(set(re.findall(r"\\begin\{([^}]+)\}", manuscript))):
    opened = manuscript.count(f"\\begin{{{environment}}}")
    closed_count = manuscript.count(f"\\end{{{environment}}}")
    if opened != closed_count:
        record(f"environment balance: {environment}", False, f"{opened} begin, {closed_count} end")
record("environments balanced", all(manuscript.count(f"\\begin{{{e}}}") == manuscript.count(f"\\end{{{e}}}")
                                    for e in set(re.findall(r"\\begin\{([^}]+)\}", manuscript))))
retired = ["separating angle", "modal cruise", "exit deadline", "frenetModel", "subsec:uncertainty",
           "single-restriction", "joint-support", "independent verifier", "v_\\tau"]
record("no retired-controller vocabulary", not any(term in manuscript for term in retired),
       str([term for term in retired if term in manuscript]))

previous_revision = subprocess.run(["git", "-C", str(repo), "rev-parse", sys.argv[3]], capture_output=True,
                                   text=True, check=True).stdout.strip() if len(sys.argv) > 3 else None
previous_text = subprocess.run(["git", "-C", str(repo), "show", previous_revision + ":paper/manuscript.tex"],
                               capture_output=True, text=True, check=True).stdout if previous_revision else None
if previous_text:
    def span(text, start, stop):
        return text[text.index(start):text.index(stop)]
    record("perception section unchanged",
           span(manuscript, "\\section{Perception}", "\\section{Estimator Design}")
           == span(previous_text, "\\section{Perception}", "\\section{Estimator Design}"))
    record("introduction literature discussion unchanged",
           span(manuscript, "\\section{Introduction}", "This paper does not close that gap.")
           == span(previous_text, "\\section{Introduction}", "We carry the safe continuation itself"))
    import difflib
    before = span(previous_text, "\\section{Estimator Design}", "The normalized direction \\(\\bm l\\) solves")
    after = span(manuscript, "\\section{Estimator Design}", "The shape \\(\\bm l\\) and the bandwidth")
    changed = sum(1 for line in difflib.unified_diff(before.splitlines(), after.splitlines(), lineterm="", n=0)
                  if line.startswith(("+", "-")) and not line.startswith(("+++", "---")))
    record("estimator derivations: only small clarifications before the tracker passage", changed <= 60,
           f"{changed} changed lines of {len(before.splitlines())}")

failed = [c for c in checks if not c["passed"]]
output.write_text(json.dumps({"checks": len(checks), "failed": len(failed), "precedingRevision": previous_revision,
                              "results": checks}, indent=1) + "\n")
for item in failed:
    print("FAIL", item["check"], "|", item["detail"])
print(f"{len(checks) - len(failed)} of {len(checks)} checks passed")
sys.exit(1 if failed else 0)
