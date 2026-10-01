#!/usr/bin/env python3
"""Summarize paired MATLAB trials without treating frames as replicates.

Requires NumPy; Matplotlib is used only with --figure-directory. Bootstrap
intervals resample paired seeds within a fixed scenario, never time samples.
Intervals are descriptive, pointwise 95% intervals, not simultaneous tests.
"""
from __future__ import annotations

import argparse
import csv
import json
import platform
from collections import defaultdict
from pathlib import Path

import numpy as np


METRICS = [
    "relativePositionRmse", "targetSpeedRmse", "relativeSpeedRmse",
    "targetVelocityRmse", "targetVelocityInertialRmse", "courseRmseDeg",
    "egoYawRmseDeg", "targetAccelerationRmse", "steadyRelativePositionRmse",
    "steadyTargetSpeedRmse", "steadyCourseRmseDeg", "fullPositionRmse",
    "fullSpeedRmse", "meanStepMilliseconds", "maximumStepMilliseconds",
]


def read_table(path: Path) -> list[dict]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def write_table(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def is_failed(row: dict) -> bool:
    return row["diverged"].lower() in ("1", "true")


def summarize(rows: list[dict], group_fields: list[str], metrics: list[str]) -> list[dict]:
    groups = defaultdict(list)
    for row in rows:
        groups[tuple(row[field] for field in group_fields)].append(row)
    summary = []
    for key, group in groups.items():
        record = dict(zip(group_fields, key))
        record.update(Trials=len(group), Failed=sum(map(is_failed, group)))
        for metric in metrics:
            # Failed runs are counted explicitly. No silently pooled survivor
            # comparison is used for the paired primary confidence intervals.
            values = np.array([float(row[metric]) for row in group if not is_failed(row)])
            record[metric] = float(values.mean()) if len(values) else float("nan")
        summary.append(record)
    return summary


def paired_intervals(rows: list[dict]) -> list[dict]:
    groups = defaultdict(dict)
    for row in rows:
        if row["Variant"] == "nominal":
            groups[(row["Scenario"], row["Seed"])][row["Estimator"]] = row
    rng = np.random.default_rng(9302026)
    intervals = []
    for scenario in dict.fromkeys(row["Scenario"] for row in rows):
        pairs = [pair for (name, _), pair in groups.items() if name == scenario]
        comparators = ["sharmaMatchedPredictor", "sharmaTunedPredictor", "sharmaTunedCorrectedPredictor"]
        if all("sharmaScenarioTunedPredictor" in pair for pair in pairs):
            comparators.append("sharmaScenarioTunedPredictor")
        for comparator in comparators:
            if any(is_failed(pair[method]) for pair in pairs for method in ("structured", comparator)):
                raise ValueError(f"Cannot silently omit failed pairs: {scenario}/{comparator}")
            for metric in ("relativePositionRmse", "targetSpeedRmse", "courseRmseDeg"):
                ours = np.array([float(pair["structured"][metric]) for pair in pairs])
                baseline = np.array([float(pair[comparator][metric]) for pair in pairs])
                draws = rng.integers(0, len(pairs), size=(10000, len(pairs)))
                difference = baseline - ours
                bootstrap = difference[draws].mean(axis=1)
                low, high = np.quantile(bootstrap, [0.025, 0.975])
                intervals.append(dict(
                    Scenario=scenario, Comparator=comparator, Metric=metric, PairedSeeds=len(pairs),
                    OursMean=ours.mean(), ComparatorMean=baseline.mean(),
                    ImprovementPercent=100*(1-ours.mean()/baseline.mean()),
                    ComparatorMinusOurs=difference.mean(), Ci95Low=low, Ci95High=high,
                    OursLowerCount=int((difference > 0).sum()),
                ))
    return intervals


def numerical_check(rows: list[dict], refined: list[dict]) -> list[dict]:
    original = {(r["Scenario"], r["Seed"], r["Estimator"]): r for r in rows if r["Variant"] == "nominal"}
    checks = []
    for row in refined:
        base = original[(row["Scenario"], row["Seed"], row["Estimator"])]
        record = dict(Scenario=row["Scenario"], Estimator=row["Estimator"],
                      FailureAgrees=is_failed(row) == is_failed(base))
        for metric in ("relativePositionRmse", "targetSpeedRmse", "courseRmseDeg"):
            record[metric + "AbsoluteChange"] = (
                abs(float(row[metric])-float(base[metric])) if not is_failed(base) and not is_failed(row) else float("nan")
            )
        checks.append(record)
    return checks


def verify_training_selection(directory: Path) -> None:
    groups = defaultdict(list)
    scenario_groups = defaultdict(lambda: defaultdict(list))
    for row in read_table(directory / "training.csv"):
        groups[row["Estimator"]].append(float(row["loss"]))
        scenario_groups[row["Scenario"]][row["Estimator"]].append(float(row["loss"]))
    selected = json.loads((directory / "completion.json").read_text())["selectedCandidate"]
    assert selected == min(groups, key=lambda name: np.mean(groups[name]))
    path = directory / "scenario-selection.csv"
    if path.exists():
        for row in read_table(path):
            losses = scenario_groups[row["Scenario"]]
            assert row["Candidate"] == min(losses, key=lambda name: np.mean(losses[name]))


def figures(summary: list[dict], destination: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    destination.mkdir(parents=True, exist_ok=True)
    scenarios = list(dict.fromkeys(row["Scenario"] for row in summary))
    methods = ["structured", "sharmaMatchedPredictor", "sharmaTunedPredictor"]
    labels = ["Current estimator", "Sharma: prior matched gains", "Sharma: training-selected gains"]
    colors = ["#0072B2", "#999999", "#D55E00"]
    if any(row["Estimator"] == "sharmaScenarioTunedPredictor" for row in summary):
        methods.append("sharmaScenarioTunedPredictor")
        labels.append("Sharma: scenario-specific training")
        colors.append("#009E73")
    names = ["Oncoming", "Curved crossing", "Curved following", "Swerve + braking", "Low relative speed", "Ego weaving"]
    lookup = {(r["Scenario"], r["Estimator"]): r for r in summary if r["Variant"] == "nominal"}
    fig, axes = plt.subplots(3, 1, figsize=(10.0, 9.0), sharex=True, layout="constrained")
    for ax, metric, label in zip(axes, ["relativePositionRmse", "targetSpeedRmse", "courseRmseDeg"],
                                 ["Position RMSE (m)", "Target speed RMSE (m/s)", "Course RMSE (deg)"]):
        for index, method in enumerate(methods):
            values = [lookup[(scenario, method)][metric] for scenario in scenarios]
            width = 0.8/len(methods)
            ax.bar(np.arange(len(scenarios)) + (index-(len(methods)-1)/2)*width, values, width=0.95*width,
                   color=colors[index], label=labels[index])
        ax.set_yscale("log")
        ax.set_ylabel(label)
        ax.grid(axis="y", alpha=0.2)
        ax.set_axisbelow(True)
    axes[0].legend(loc="upper left", fontsize=9)
    axes[-1].set_xticks(np.arange(len(scenarios)), names, rotation=15, ha="right")
    fig.suptitle("Paired synthetic comparison: 20 held-out noise seeds\n50 Hz; errors after 2 s; different ego sensor usage disclosed", fontsize=12)
    fig.savefig(destination / "held_out_comparison.png", dpi=180)
    fig.savefig(destination / "held_out_comparison.pdf")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--figure-directory", type=Path)
    args = parser.parse_args()
    verify_training_selection(args.directory)
    rows = read_table(args.directory / "trials.csv")
    supplemental = args.directory / "scenario-tuned.csv"
    if supplemental.exists():
        rows += read_table(supplemental)
    noisefree_supplement = args.directory / "noisefree-supplement.csv"
    if noisefree_supplement.exists():
        rows += read_table(noisefree_supplement)
    summary = summarize(rows, ["Scenario", "Variant", "Estimator"], METRICS)
    intervals = paired_intervals(rows)
    oracle = read_table(args.directory / "oracle.csv")
    oracle_summary = summarize(oracle, ["Scenario", "Estimator"],
                               ["relativePositionRmse", "targetSpeedRmse", "targetVelocityRmse", "courseRmseDeg"])
    numeric = numerical_check(rows, read_table(args.directory / "numerical.csv"))
    write_table(args.directory / "summary.csv", summary)
    write_table(args.directory / "paired_intervals.csv", intervals)
    write_table(args.directory / "oracle_summary.csv", oracle_summary)
    write_table(args.directory / "numerical_check.csv", numeric)
    protocol = json.loads((args.directory / "protocol.json").read_text())
    training_seeds = set(protocol["options"]["TrainingSeeds"])
    test_seeds = {int(row["Seed"]) for row in rows}
    assert not (training_seeds & test_seeds)
    count = defaultdict(lambda: [0, 0])
    for row in rows:
        count[row["Estimator"]][0] += 1
        count[row["Estimator"]][1] += is_failed(row)
    integrity = dict(testTrials=len(rows), oracleTrials=len(oracle), seedsDisjoint=True,
                     trainingSelectionsIndependentlyVerified=True,
                     pythonVersion=platform.python_version(), numpyVersion=np.__version__,
                     trialAndFailureCounts=dict(count), numericalFailureStatusAgrees=all(r["FailureAgrees"] for r in numeric),
                     inferenceUnit="One complete noise seed within a fixed scenario",
                     bootstrap="10000 paired seed draws; RNG 9302026; pointwise percentile 95% intervals",
                     scope="Noise variability only: not road-population uncertainty or real-vehicle replication")
    (args.directory / "analysis.json").write_text(json.dumps(integrity, indent=2) + "\n")
    if args.figure_directory:
        figures(summary, args.figure_directory)
    print(json.dumps(integrity, indent=2))
    for row in intervals:
        if row["Comparator"] == "sharmaTunedPredictor":
            print(row)


if __name__ == "__main__":
    main()
