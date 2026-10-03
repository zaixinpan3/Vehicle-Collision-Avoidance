"""Local known-size rectangle fitting from a predicted planar pose."""

from __future__ import annotations

import math
from typing import Any

import numpy as np


def fit_rectangle_pose(
    points_ego: np.ndarray,
    initial_pose: np.ndarray,
    vehicle_length: float,
    vehicle_width: float,
    support_quantile: float = 2.0,
    placement_mode: str = "edge",
    symmetry_minimum_support: float = 0.8,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Refine x, y and yaw using geometry, with the prior used only as a seed.

    The objective retains capped face distance and weight-20 containment.
    Symmetric placement adds an observed-support midpoint residual along the
    best-observed axis. It uses the same visibility assumption as the discrete
    fitter. Neither position nor heading is penalized against the initial pose.
    A partially visible face can leave tangential translation unobservable;
    local initialization chooses a solution but does not resolve that ambiguity.
    """
    pose = np.asarray(initial_pose, dtype=float).copy()
    points = np.asarray(points_ego, dtype=float)[:, :2]
    half_size = 0.5 * np.asarray([vehicle_length, vehicle_width], dtype=float)
    if pose.shape != (3,) or not np.all(np.isfinite(pose)):
        raise ValueError("Rectangle initial pose must contain finite x, y and heading.")
    if points.shape[0] < 3 or not np.all(np.isfinite(points)):
        raise ValueError("Rectangle fitting requires at least three finite points.")
    if not np.all(np.isfinite(half_size)) or np.any(half_size <= 0.0):
        raise ValueError("Rectangle dimensions must be finite and positive.")
    if placement_mode not in ("edge", "symmetric"):
        raise ValueError("Rectangle placement must be edge or symmetric.")
    if not 0.0 < symmetry_minimum_support <= 1.0:
        raise ValueError("Rectangle symmetry minimum support must be in (0, 1].")
    quantile = min(max(float(support_quantile), 0.0), 25.0)
    initial = pose.copy()

    def axes(heading: float) -> np.ndarray:
        c, s = math.cos(heading), math.sin(heading)
        return np.asarray([[c, -s], [s, c]])

    initial_projection = points @ axes(pose[2])
    low, high = np.percentile(initial_projection, [quantile, 100.0 - quantile], axis=0)
    coverage = (high - low) / (2.0 * half_size)
    symmetric_axis = None
    if placement_mode == "symmetric" and np.max(coverage) >= symmetry_minimum_support:
        symmetric_axis = int(np.argmax(coverage))

    def residual(candidate: np.ndarray) -> np.ndarray:
        basis = axes(candidate[2])
        local = (points - candidate[:2]) @ basis
        excess = np.abs(local) - half_size
        boundary = np.minimum(np.min(np.abs(excess), axis=1), 0.5)
        pieces = [boundary, math.sqrt(20.0) * np.maximum(excess, 0.0).ravel()]
        result = np.concatenate(pieces) / math.sqrt(len(points))
        if symmetric_axis is not None:
            support = np.percentile(local[:, symmetric_axis], [quantile, 100.0 - quantile])
            result = np.append(result, 0.5 * (support[0] + support[1]))
        return result

    value = residual(pose)
    cost = float(value @ value)
    initial_cost = cost
    damping = 1.0e-3
    # Scale angular increments by their corner displacement in metres.
    scale = np.asarray([1.0, 1.0, 1.0 / np.linalg.norm(half_size)])
    converged = False
    iterations = 0
    for iterations in range(1, 61):
        jacobian = np.empty((len(value), 3))
        for axis in range(3):
            delta = np.zeros(3)
            delta[axis] = 1.0e-5 * scale[axis]
            jacobian[:, axis] = (residual(pose + delta) - residual(pose - delta)) / 2.0e-5
        gradient = jacobian.T @ value
        if np.linalg.norm(gradient, ord=np.inf) < 1.0e-7:
            converged = True
            break
        normal = jacobian.T @ jacobian
        diagonal = np.maximum(np.diag(normal), 1.0e-6)
        step = np.linalg.solve(normal + damping * np.diag(diagonal), -gradient)
        # This bounds individual numerical steps, not the final pose correction.
        step /= max(1.0, float(np.linalg.norm(step)) / 0.5)
        candidate = pose + scale * step
        next_value = residual(candidate)
        next_cost = float(next_value @ next_value)
        if next_cost < cost:
            improvement = cost - next_cost
            pose, value, cost = candidate, next_value, next_cost
            damping = max(damping / 3.0, 1.0e-9)
            if np.linalg.norm(step) < 1.0e-6 or improvement < 1.0e-10:
                converged = True
                break
        else:
            damping *= 10.0
            if damping > 1.0e10:
                break

    pose[2] = math.atan2(math.sin(pose[2]), math.cos(pose[2]))
    basis = axes(pose[2])
    projections = points @ basis
    low, high = np.percentile(projections, [quantile, 100.0 - quantile], axis=0)
    local = (points - pose[:2]) @ basis
    excess = np.abs(local) - half_size
    center_projection = pose[:2] @ basis
    corners = np.asarray([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * half_size
    correction = pose - initial
    correction[2] = math.atan2(math.sin(correction[2]), math.cos(correction[2]))
    fit = {
        "center": pose[:2].tolist(),
        "heading_rad": float(pose[2]),
        "length_m": vehicle_length,
        "width_m": vehicle_width,
        "corners": (corners @ basis.T + pose[:2]).tolist(),
        "rear_support_m": float(center_projection[0] - half_size[0]),
        "lateral_center_m": float(center_projection[1]),
        "observed_longitudinal_span_m": float(np.ptp(projections[:, 0])),
        "observed_lateral_span_m": float(high[1] - low[1]),
        "longitudinal_support_m": [float(low[0]), float(high[0])],
        "lateral_support_m": [float(low[1]), float(high[1])],
        "support_quantile_percent": quantile,
        "placement_mode": placement_mode,
        "symmetry_minimum_support": symmetry_minimum_support,
        "symmetric_axis": None if symmetric_axis is None else ["longitudinal", "lateral"][symmetric_axis],
        "mean_boundary_distance_m": float(np.mean(np.min(np.abs(excess), axis=1))),
        "maximum_containment_violation_m": float(np.max(np.maximum(excess, 0.0))),
        "objective": cost,
        "seed_objective": initial_cost,
        "initial_pose": initial.tolist(),
        "pose_correction": correction.tolist(),
        "pose_prior_penalty_weight": 0.0,
        "heading_constraint": "free",
        "optimizer_iterations": iterations,
        "optimizer_converged": converged,
        "used_temporal_heading_hint": True,
    }
    return pose[:2].copy(), fit
