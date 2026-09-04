#!/usr/bin/env python3
"""Attribute localization error by handing the rectangle fit a known heading.

This is a diagnostic, not a perception stage.  It replaces the heading that
``yolo_lidar_target_position.py`` would estimate with the CARLA ground-truth
target heading, so every number it produces is an oracle bound that assumes
information the perception pipeline does not have.  Never report its output as
perception performance; it exists to separate heading error from everything
else in the error budget.

The LiDAR-to-rectangle geometry is otherwise untouched: the same detector, the
same point association, the same known-size rectangle, the same gates.  Only
the heading argument passed to ``fit_known_size_rectangle`` changes.

Two spans answer two different questions:

* ``--heading-span 0`` locks the fit to the true heading.  What remains is the
  error floor that a perfect heading estimator would leave behind.
* A nonzero span seeds the search at the true heading and lets the fit's own
  objective move away from it.  If the result degrades, the objective prefers a
  wrong heading, and no better heading search can rescue it.

Every other flag is forwarded verbatim to ``yolo_lidar_target_position.py``.

Example:

    python3 perception/heading_oracle_diagnostic.py \\
        --dataset simulation_output/intersection_crossing_fov120 \\
        --model /path/to/yolo26x.pt \\
        --output simulation_output/crossing_heading_oracle \\
        --camera-id front --device 0 --image-size 1109 \\
        --confidence 0.40 --min-bbox-area-ratio 0.000666 \\
        --min-lidar-points 16 --center-mode rectangle \\
        --heading-span 0
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

import yolo_lidar_target_position as pipeline


ORACLE_NOTE = (
    "Heading taken from the CARLA target transform, not estimated. These "
    "figures are an oracle bound on the perception pipeline and are not "
    "perception performance."
)


def truth_headings(dataset_root: Path) -> list[float]:
    """Ego-frame target heading of every frame, from the CARLA transforms."""

    headings: list[float] = []
    for frame in pipeline.iter_jsonl(dataset_root / "frames.jsonl"):
        ego_yaw_deg = float(frame["ego"]["transform"]["rotation_deg"][1])
        target_yaw_deg = float(frame["target"]["transform"]["rotation_deg"][1])
        headings.append(
            pipeline.wrap_angle(math.radians(target_yaw_deg - ego_yaw_deg))
        )
    return headings


def install_truth_heading(dataset_root: Path) -> None:
    """Serve the true heading where the pipeline reads its motion heading.

    The pipeline alternates between fitting the body at a heading and reading
    the heading back off the fitted track.  Replacing the track reader hands
    every fit the true heading from the second pass onward and leaves the rest
    of the loop, including its convergence test, exactly as it is.
    """

    headings = truth_headings(dataset_root)

    def truth_motion_headings(
        times: list[float],
        _world_positions: Any,
        _ego_yaws: Any,
        _window_seconds: float,
        _minimum_displacement: float,
    ) -> list[float | None]:
        if len(headings) < len(times):
            raise ValueError(
                "The dataset has fewer frames than the pipeline processed."
            )
        return list(headings[: len(times)])

    pipeline.motion_headings = truth_motion_headings


def parse_diagnostic_args(
    argv: list[str],
) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--heading-span",
        type=float,
        default=0.0,
        help=(
            "Degrees the fit may move away from the true heading. Zero locks "
            "the fit to it."
        ),
    )
    parser.add_argument(
        "--heading-prior-weight",
        type=float,
        default=None,
        help=(
            "Quadratic penalty on leaving the true heading. Defaults to the "
            "pipeline's own heading smoothness; set it low to let the "
            "objective move freely within the span."
        ),
    )
    return parser.parse_known_args(argv)


def main(argv: list[str] | None = None) -> int:
    diagnostic, forwarded = parse_diagnostic_args(
        sys.argv[1:] if argv is None else argv
    )
    args = pipeline.parse_args(forwarded)
    if args.center_mode != "rectangle":
        raise ValueError(
            "The heading oracle only applies to --center-mode rectangle; "
            f"received {args.center_mode}."
        )
    if diagnostic.heading_span < 0.0:
        raise ValueError("--heading-span must not be negative.")

    args.rectangle_heading_span = diagnostic.heading_span
    # Pass one has no heading and falls back to the principal direction; pass
    # two receives the true heading.  A third would only repeat pass two.
    args.heading_iterations = 2
    if diagnostic.heading_prior_weight is not None:
        args.rectangle_heading_smoothness = diagnostic.heading_prior_weight
    if diagnostic.heading_span <= 0.0:
        # The refinement stage would otherwise walk off the locked heading.
        args.rectangle_refine_heading_span = 0.0

    install_truth_heading(args.dataset.resolve())
    summary = pipeline.run_pipeline(args)

    summary_path = args.output.resolve() / "summary.json"
    written = json.loads(summary_path.read_text(encoding="utf-8"))
    written["heading_source"] = "carla_ground_truth"
    written["heading_span_deg"] = diagnostic.heading_span
    written["heading_prior_weight"] = args.rectangle_heading_smoothness
    written["oracle_note"] = ORACLE_NOTE
    summary_path.write_text(
        json.dumps(written, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"\n{ORACLE_NOTE}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
