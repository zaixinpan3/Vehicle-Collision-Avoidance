#!/usr/bin/env python3
"""Causal YOLO + LiDAR localization initialized by predicted target pose.

The estimator predicts a current-frame relative center and body heading before
the point cloud refines all three pose variables. The fitted center is then
delivered to the estimator as a radar-compatible position measurement.
"""

from __future__ import annotations

import argparse
import csv
import importlib
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Protocol

import numpy as np

from rectangle_pose_fit import fit_rectangle_pose


@dataclass(frozen=True)
class Detection:
    camera_id: str
    bbox_xyxy: tuple[float, float, float, float]
    confidence: float
    class_id: int
    class_name: str


@dataclass
class CameraEstimate:
    camera_id: str
    detection: Detection
    point_count: int
    relative_position: np.ndarray
    capsule: dict[str, Any]
    rectangle: dict[str, Any] | None


@dataclass(frozen=True)
class PosePrior:
    """Current-time predicted center and body heading in point-cloud ego axes.

    Position may be absent only for initialization before the first target
    acquisition. It is then bootstrapped from the current observed geometry.
    Both position and heading remain free during the subsequent local fit.
    """

    position_ego_m: tuple[float, float] | None
    heading_ego_rad: float
    state_time: float
    last_measurement_time: float | None
    source: str


class PoseFeedback(Protocol):
    """Adapter to an estimator that predicts before consuming each position."""

    def predict(self, frame: int, time: float) -> PosePrior:
        ...

    def update(self, frame: int, time: float, position: np.ndarray | None) -> None:
        ...


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def iter_jsonl(path: Path) -> Iterable[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                yield json.loads(line)


def matrix_from_record(record: dict[str, Any]) -> np.ndarray:
    for key in ("transform", "sensor_transform_world", "extrinsic_ego"):
        nested = record.get(key)
        if isinstance(nested, dict):
            record = nested
            break
    matrix = np.asarray(record["matrix"], dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("Transform matrix must be 4-by-4.")
    return matrix


def transform_points(matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
    if points.size == 0:
        return np.zeros((0, 3), dtype=float)
    homogeneous = np.column_stack([points[:, :3], np.ones(points.shape[0])])
    return (matrix @ homogeneous.T).T[:, :3]


def camera_intrinsic(width: int, height: int, fov_degrees: float) -> np.ndarray:
    focal = width / (2.0 * math.tan(math.radians(fov_degrees) / 2.0))
    return np.asarray(
        [[focal, 0.0, width / 2.0], [0.0, focal, height / 2.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )


def project_world_points_to_camera(
    world_points: np.ndarray,
    camera_to_world: np.ndarray,
    intrinsic: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    world_to_camera = np.linalg.inv(camera_to_world)
    camera_unreal = transform_points(world_to_camera, world_points)

    # CARLA camera projection convention: Unreal x is optical depth,
    # image x is Unreal y, and image y is -Unreal z.
    depth = camera_unreal[:, 0]
    camera_cv = np.column_stack([camera_unreal[:, 1], -camera_unreal[:, 2], depth])
    valid = depth > 0.1
    pixels_h = (intrinsic @ camera_cv.T).T
    pixels = np.zeros((world_points.shape[0], 2), dtype=float)
    pixels[valid] = pixels_h[valid, :2] / pixels_h[valid, 2:3]
    return pixels, depth


def load_lidar_points(path: Path) -> np.ndarray:
    data = np.load(path)
    if isinstance(data, np.ndarray):
        points = np.asarray(data, dtype=float)
    else:
        try:
            if "points" not in data:
                raise ValueError(
                    f"LiDAR archive {path} does not contain a 'points' array."
                )
            points = np.asarray(data["points"], dtype=float)
        finally:
            data.close()
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError(f"LiDAR points in {path} must be N-by-3 or N-by-4.")
    return points[:, :4] if points.shape[1] >= 4 else points[:, :3]


def union_find_components(points_xy: np.ndarray, radius: float) -> np.ndarray:
    count = points_xy.shape[0]
    parent = np.arange(count)
    rank = np.zeros(count, dtype=int)
    radius_squared = radius * radius

    def find(idx: int) -> int:
        root = idx
        while parent[root] != root:
            root = parent[root]
        while parent[idx] != idx:
            next_idx = parent[idx]
            parent[idx] = root
            idx = next_idx
        return root

    def union(a: int, b: int) -> None:
        ra = find(a)
        rb = find(b)
        if ra == rb:
            return
        if rank[ra] < rank[rb]:
            parent[ra] = rb
        elif rank[ra] > rank[rb]:
            parent[rb] = ra
        else:
            parent[rb] = ra
            rank[ra] += 1

    for i in range(count):
        delta = points_xy[(i + 1) :, :] - points_xy[i, :]
        close = np.flatnonzero(np.sum(delta * delta, axis=1) <= radius_squared)
        for offset in close:
            union(i, i + 1 + int(offset))

    roots = np.asarray([find(i) for i in range(count)])
    _, inverse = np.unique(roots, return_inverse=True)
    return inverse


def largest_component(points_ego: np.ndarray, cluster_radius: float) -> np.ndarray:
    if points_ego.shape[0] < 3:
        return points_ego
    labels = union_find_components(points_ego[:, :2], cluster_radius)
    counts = np.bincount(labels)
    largest_label = int(np.argmax(counts))
    return points_ego[labels == largest_label, :]


def robust_near_depth_mask(depth: np.ndarray, max_depth_span: float) -> np.ndarray:
    finite = np.isfinite(depth)
    if not np.any(finite):
        return finite
    near = float(np.percentile(depth[finite], 5.0))
    return finite & (depth <= near + max_depth_span)


def principal_heading(points_xy: np.ndarray) -> float:
    if points_xy.shape[0] < 3:
        return 0.0
    centered = points_xy - np.mean(points_xy, axis=0, keepdims=True)
    covariance = centered.T @ centered / max(points_xy.shape[0] - 1, 1)
    values, vectors = np.linalg.eigh(covariance)
    axis = vectors[:, int(np.argmax(values))]
    return math.atan2(float(axis[1]), float(axis[0]))


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def axis_placements(
    projections: np.ndarray,
    size: float,
    quantile: float,
) -> tuple[list[float], float, float]:
    """Candidate centers of a known extent along one axis.

    A LiDAR sees only the faces of a vehicle that point at it, so along most
    axes just one end returns points and the observed span falls short of the
    object.  Centering on that span pulls the estimate toward the sensor by
    half the unobserved remainder, which is a bias, not noise.  Each observed
    end is instead a candidate real face, and anchoring the known extent to a
    real face is exact.

    Which end is the real one is not decided here.  Both anchors are returned
    and the caller picks between them with the same objective it uses for the
    heading, so the choice is made by how well the whole rectangle explains the
    points rather than by a rule about which end ought to be visible.  When the
    observation already covers the full extent, both ends are real and the
    midpoint is the only consistent placement.
    """

    low = float(np.percentile(projections, quantile))
    high = float(np.percentile(projections, 100.0 - quantile))
    if high - low >= size:
        return [0.5 * (low + high)], low, high
    return [low + 0.5 * size, high - 0.5 * size], low, high


def fit_known_size_rectangle(
    points_ego: np.ndarray,
    vehicle_length: float,
    vehicle_width: float,
    heading_hint: float | None,
    support_quantile: float,
    heading_step_degrees: float,
    initial_heading_span_degrees: float,
    max_heading_change_degrees: float,
    heading_smoothness: float,
    placement_mode: str = "edge",
    symmetry_minimum_support: float = 0.8,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Fit visible faces, optionally assuming symmetric tangential support.

    Symmetric placement centers the most completely observed axis when its
    span covers enough of the known dimension. This addresses a visible body
    face narrower than the declared bounding box. It assumes approximately
    symmetric visibility along that axis; asymmetric occlusion can violate it.
    The face-normal axis retains the existing edge anchoring.
    """
    if placement_mode not in ("edge", "symmetric"):
        raise ValueError("Rectangle placement must be edge or symmetric.")
    if not 0.0 < symmetry_minimum_support <= 1.0:
        raise ValueError("Rectangle symmetry minimum support must be in (0, 1].")
    points_xy = points_ego[:, :2]
    median_xy = np.median(points_xy, axis=0)
    if heading_hint is None:
        # With no external heading, the point set's own principal direction is
        # the only cue, and it is ambiguous by 90 degrees when a single face is
        # visible.  The search therefore covers the full half-plane and lets
        # the containment term reject placements that would put a long observed
        # span across the vehicle's short extent.
        reference_heading = principal_heading(points_xy)
        heading_span = initial_heading_span_degrees
        prior_weight = 0.05
    else:
        reference_heading = heading_hint
        heading_span = max_heading_change_degrees
        prior_weight = heading_smoothness

    step = max(float(heading_step_degrees), 1.0e-3)
    offsets = np.arange(-heading_span, heading_span + 0.5 * step, step)
    quantile = min(max(float(support_quantile), 0.0), 25.0)
    best: tuple[float, np.ndarray, dict[str, Any]] | None = None

    for offset_degrees in offsets:
        heading = wrap_angle(reference_heading + math.radians(float(offset_degrees)))
        longitudinal_axis = np.asarray([math.cos(heading), math.sin(heading)], dtype=float)
        if float(longitudinal_axis @ median_xy) < 0.0:
            heading = wrap_angle(heading + math.pi)
            longitudinal_axis = -longitudinal_axis
        lateral_axis = np.asarray(
            [-longitudinal_axis[1], longitudinal_axis[0]],
            dtype=float,
        )
        longitudinal = points_xy @ longitudinal_axis
        lateral = points_xy @ lateral_axis
        longitudinal_placements, longitudinal_low, longitudinal_high = (
            axis_placements(longitudinal, vehicle_length, quantile)
        )
        lateral_placements, lateral_low, lateral_high = axis_placements(
            lateral, vehicle_width, quantile
        )
        observed_lateral_span = lateral_high - lateral_low
        symmetric_axis = None
        if placement_mode == "symmetric":
            longitudinal_coverage = (longitudinal_high - longitudinal_low) / vehicle_length
            lateral_coverage = observed_lateral_span / vehicle_width
            if lateral_coverage >= max(symmetry_minimum_support, longitudinal_coverage):
                lateral_placements = [0.5 * (lateral_low + lateral_high)]
                symmetric_axis = "lateral"
            elif longitudinal_coverage >= symmetry_minimum_support:
                longitudinal_placements = [0.5 * (longitudinal_low + longitudinal_high)]
                symmetric_axis = "longitudinal"
        # The axis is flipped above to point away from the sensor, and a
        # rectangle is unchanged by a half turn, so the deviation from the
        # reference heading must be measured modulo 180 degrees.  Taken modulo
        # 360 it would jump by the full prior penalty partway through a search
        # window whenever the reference is near perpendicular to the line of
        # sight, which is exactly the crossing geometry.
        heading_delta = wrap_angle(heading - reference_heading)
        if heading_delta > 0.5 * math.pi:
            heading_delta -= math.pi
        elif heading_delta < -0.5 * math.pi:
            heading_delta += math.pi
        heading_penalty = prior_weight * heading_delta**2

        placed: tuple[float, float, float, np.ndarray, float] | None = None
        for longitudinal_center in longitudinal_placements:
            rear_support = longitudinal_center - 0.5 * vehicle_length
            rear_distance = np.abs(longitudinal - rear_support)
            front_distance = np.abs(
                longitudinal - (rear_support + vehicle_length)
            )
            longitudinal_distance = np.minimum(rear_distance, front_distance)
            longitudinal_overflow = (
                np.maximum(rear_support - longitudinal, 0.0) ** 2
                + np.maximum(
                    longitudinal - (rear_support + vehicle_length), 0.0
                )
                ** 2
            )
            for lateral_center in lateral_placements:
                left_distance = np.abs(
                    lateral - (lateral_center - 0.5 * vehicle_width)
                )
                right_distance = np.abs(
                    lateral - (lateral_center + 0.5 * vehicle_width)
                )
                boundary_distance = np.minimum(
                    longitudinal_distance,
                    np.minimum(left_distance, right_distance),
                )
                lateral_overflow = (
                    np.maximum(
                        np.abs(lateral - lateral_center)
                        - 0.5 * vehicle_width,
                        0.0,
                    )
                    ** 2
                )
                # No term rewards an observed span for matching a vehicle
                # dimension.  A side-viewed vehicle returns one flank, whose
                # span across the width is near zero, and rewarding a
                # full-width span there rotates the rectangle until one face
                # appears to fill both dimensions.  What the observation does
                # constrain is that no point may fall outside the rectangle,
                # which the containment term enforces, and that points should
                # lie on its boundary, which the closeness term enforces.
                candidate_objective = (
                    float(np.mean(np.minimum(boundary_distance**2, 0.25)))
                    + 20.0
                    * float(
                        np.mean(longitudinal_overflow + lateral_overflow)
                    )
                    + heading_penalty
                )
                if placed is None or candidate_objective < placed[0]:
                    placed = (
                        candidate_objective,
                        longitudinal_center,
                        lateral_center,
                        boundary_distance,
                        float(np.max(np.maximum(
                            np.maximum(
                                rear_support - longitudinal,
                                longitudinal - (rear_support + vehicle_length),
                            ),
                            np.abs(lateral - lateral_center)
                            - 0.5 * vehicle_width,
                        ).clip(min=0.0))),
                    )

        if placed is None:
            raise RuntimeError("Rectangle placement produced no candidate.")
        (
            objective,
            longitudinal_center,
            lateral_center,
            boundary_distance,
            maximum_containment_violation,
        ) = placed
        rear_support = longitudinal_center - 0.5 * vehicle_length

        center_xy = (
            longitudinal_center * longitudinal_axis
            + lateral_center * lateral_axis
        )
        half_length_vector = 0.5 * vehicle_length * longitudinal_axis
        half_width_vector = 0.5 * vehicle_width * lateral_axis
        corners = np.vstack(
            [
                center_xy - half_length_vector - half_width_vector,
                center_xy + half_length_vector - half_width_vector,
                center_xy + half_length_vector + half_width_vector,
                center_xy - half_length_vector + half_width_vector,
            ]
        )
        fit = {
            "center": center_xy.tolist(),
            "heading_rad": heading,
            "length_m": vehicle_length,
            "width_m": vehicle_width,
            "corners": corners.tolist(),
            "rear_support_m": rear_support,
            "lateral_center_m": lateral_center,
            "support_quantile_percent": quantile,
            "placement_mode": placement_mode,
            "symmetry_minimum_support": symmetry_minimum_support,
            "symmetric_axis": symmetric_axis,
            "observed_longitudinal_span_m": float(
                np.max(longitudinal) - np.min(longitudinal)
            ),
            "observed_lateral_span_m": observed_lateral_span,
            "longitudinal_support_m": [longitudinal_low, longitudinal_high],
            "lateral_support_m": [lateral_low, lateral_high],
            "longitudinal_placement_count": len(longitudinal_placements),
            "lateral_placement_count": len(lateral_placements),
            "mean_boundary_distance_m": float(np.mean(boundary_distance)),
            "maximum_containment_violation_m": maximum_containment_violation,
            "objective": objective,
            "heading_reference_rad": reference_heading,
            "used_temporal_heading_hint": heading_hint is not None,
        }
        if best is None or objective < best[0]:
            best = (objective, center_xy, fit)

    if best is None:
        raise RuntimeError("Known-size rectangle fitting produced no candidate.")
    return best[1], best[2]


def rectangle_containment_violation(
    points_ego: np.ndarray,
    rectangle: dict[str, Any],
) -> np.ndarray:
    heading = float(rectangle["heading_rad"])
    longitudinal_axis = np.asarray([math.cos(heading), math.sin(heading)], dtype=float)
    lateral_axis = np.asarray(
        [-longitudinal_axis[1], longitudinal_axis[0]],
        dtype=float,
    )
    points_xy = points_ego[:, :2]
    longitudinal = points_xy @ longitudinal_axis
    lateral = points_xy @ lateral_axis
    rear_support = float(rectangle["rear_support_m"])
    vehicle_length = float(rectangle["length_m"])
    lateral_center = float(rectangle["lateral_center_m"])
    vehicle_width = float(rectangle["width_m"])
    return np.maximum.reduce(
        [
            rear_support - longitudinal,
            longitudinal - (rear_support + vehicle_length),
            np.abs(lateral - lateral_center) - 0.5 * vehicle_width,
            np.zeros_like(longitudinal),
        ]
    )


def capsule_from_points(
    points_ego: np.ndarray,
    center_xy: np.ndarray,
    vehicle_length: float,
    vehicle_width: float,
    heading_override: float | None = None,
) -> dict[str, Any]:
    heading = (
        principal_heading(points_ego[:, :2])
        if heading_override is None
        else heading_override
    )
    direction = np.asarray([math.cos(heading), math.sin(heading)], dtype=float)
    half_length = 0.5 * vehicle_length
    radius = 0.5 * vehicle_width
    endpoints = np.vstack([center_xy - direction * half_length, center_xy + direction * half_length])
    return {
        "center": center_xy.tolist(),
        "heading_rad": heading,
        "length_m": vehicle_length,
        "radius_m": radius,
        "endpoints": endpoints.tolist(),
    }


def associate_detection_points(
    detection: Detection,
    lidar_points: np.ndarray,
    lidar_to_world: np.ndarray,
    camera: dict[str, Any],
    ego_to_world: np.ndarray,
    args: argparse.Namespace,
) -> np.ndarray | None:
    """Select the ego-frame LiDAR cluster belonging to one image detection.

    Association depends on the image box, the sensor calibration and the range
    gates only.  It does not depend on the target's heading, so the result can
    be reused while the heading estimate is revised.
    """

    camera_to_world = matrix_from_record(camera["transform"])
    intrinsic = camera_intrinsic(
        int(camera["width"]),
        int(camera["height"]),
        float(camera["fov"]),
    )

    world_points = transform_points(lidar_to_world, lidar_points[:, :3])
    pixels, depth = project_world_points_to_camera(world_points, camera_to_world, intrinsic)
    x1, y1, x2, y2 = detection.bbox_xyxy
    pad_x = args.bbox_margin * (x2 - x1)
    pad_y = args.bbox_margin * (y2 - y1)
    inside = (
        (pixels[:, 0] >= x1 - pad_x)
        & (pixels[:, 0] <= x2 + pad_x)
        & (pixels[:, 1] >= y1 - pad_y)
        & (pixels[:, 1] <= y2 + pad_y)
        & (depth > args.min_depth)
        & (depth < args.max_depth)
    )
    selected_world = world_points[inside, :]
    selected_depth = depth[inside]
    if selected_world.shape[0] < args.min_lidar_points:
        return None

    selected_world = selected_world[robust_near_depth_mask(selected_depth, args.max_depth_span), :]
    if selected_world.shape[0] < args.min_lidar_points:
        return None

    world_to_ego = np.linalg.inv(ego_to_world)
    points_ego = transform_points(world_to_ego, selected_world)
    finite = np.all(np.isfinite(points_ego), axis=1)
    points_ego = points_ego[finite, :]
    points_ego = points_ego[points_ego[:, 2] >= args.min_object_height, :]
    if points_ego.shape[0] < args.min_lidar_points:
        return None

    if points_ego.shape[0] > args.max_cluster_points:
        # Deterministic down-sampling keeps the clustering cost bounded.
        stride = int(math.ceil(points_ego.shape[0] / args.max_cluster_points))
        points_ego = points_ego[::stride, :]

    cluster = largest_component(points_ego, args.cluster_radius)
    if cluster.shape[0] < args.min_lidar_points:
        return None
    return cluster


def estimate_from_cluster(
    detection: Detection,
    cluster: np.ndarray,
    args: argparse.Namespace,
    pose_prior: PosePrior,
) -> CameraEstimate | None:
    """Refine the predicted center and heading using current point-cloud geometry.

    First acquisition obtains its missing center from observed face placement.
    Both fitting stages then optimize position and heading freely. A cluster
    with insufficient observed support remains a missing measurement.
    """
    seed_center = pose_prior.position_ego_m
    if seed_center is None:
        seed_center, _ = fit_known_size_rectangle(
            cluster, args.vehicle_length, args.vehicle_width,
            pose_prior.heading_ego_rad, args.rectangle_support_quantile,
            1.0, 0.0, 0.0, 0.0, "symmetric", args.rectangle_symmetry_minimum_support,
        )
    seed = np.asarray([*seed_center, pose_prior.heading_ego_rad])
    center_xy, initial_rectangle = fit_rectangle_pose(
        cluster, seed, args.vehicle_length, args.vehicle_width,
        args.rectangle_support_quantile, args.rectangle_symmetry_minimum_support,
    )
    initial_violation = rectangle_containment_violation(cluster, initial_rectangle)
    fit_cluster = cluster[initial_violation <= args.rectangle_outlier_tolerance, :]
    if fit_cluster.shape[0] >= args.min_lidar_points:
        center_xy, rectangle = fit_rectangle_pose(
            fit_cluster, np.asarray([*center_xy, initial_rectangle["heading_rad"]]),
            args.vehicle_length, args.vehicle_width, 0.0,
            args.rectangle_symmetry_minimum_support,
        )
    else:
        fit_cluster = cluster
        rectangle = initial_rectangle
    rectangle.update(
        {
            "initial_objective": float(initial_rectangle["objective"]),
            "heading_constraint": "free",
            "raw_cluster_point_count": int(cluster.shape[0]),
            "fit_point_count": int(fit_cluster.shape[0]),
            "rejected_outlier_count": int(cluster.shape[0] - fit_cluster.shape[0]),
            "fit_point_fraction": float(fit_cluster.shape[0] / cluster.shape[0]),
            "outlier_tolerance_m": args.rectangle_outlier_tolerance,
            "all_point_maximum_containment_violation_m": float(
                np.max(rectangle_containment_violation(cluster, rectangle))
            ),
            "prediction_initial_pose": initial_rectangle["initial_pose"],
            "first_stage_pose": [*initial_rectangle["center"], initial_rectangle["heading_rad"]],
            "first_stage_converged": initial_rectangle["optimizer_converged"],
        }
    )
    support_fraction = max(
        float(rectangle["observed_longitudinal_span_m"]) / args.vehicle_length,
        float(rectangle["observed_lateral_span_m"]) / args.vehicle_width,
    )
    rectangle["support_fraction"] = support_fraction
    if support_fraction < args.minimum_support_fraction:
        return None
    capsule = capsule_from_points(
        cluster, center_xy, args.vehicle_length, args.vehicle_width,
        float(rectangle["heading_rad"]),
    )
    return CameraEstimate(
        camera_id=detection.camera_id, detection=detection,
        point_count=int(cluster.shape[0]), relative_position=center_xy,
        capsule=capsule, rectangle=rectangle,
    )


def estimate_from_detection(
    detection: Detection,
    lidar_points: np.ndarray,
    lidar_to_world: np.ndarray,
    camera: dict[str, Any],
    ego_to_world: np.ndarray,
    args: argparse.Namespace,
    pose_prior: PosePrior,
) -> CameraEstimate | None:
    cluster = associate_detection_points(
        detection,
        lidar_points,
        lidar_to_world,
        camera,
        ego_to_world,
        args,
    )
    if cluster is None:
        return None
    return estimate_from_cluster(detection, cluster, args, pose_prior)


class UltralyticsCarDetector:
    def __init__(
        self,
        model_path: Path,
        confidence: float,
        iou: float,
        device: str | None,
        car_class_id: int | None,
        image_size: int,
    ) -> None:
        try:
            from ultralytics import YOLO  # type: ignore
        except Exception as exc:  # pragma: no cover - depends on local YOLO env
            raise RuntimeError("ultralytics is not installed. Install the YOLO runtime.") from exc

        self.model = YOLO(str(model_path))
        self.confidence = confidence
        self.iou = iou
        self.device = device
        self.car_class_id = car_class_id if car_class_id is not None else self._infer_car_class_id()
        self.image_size = image_size

    def _infer_car_class_id(self) -> int:
        names = getattr(self.model, "names", None)
        if isinstance(names, dict):
            for class_id, name in names.items():
                if str(name).strip().lower() == "car":
                    return int(class_id)
            if len(names) == 1 and str(next(iter(names.values()))).strip().lower() == "car":
                return int(next(iter(names.keys())))
        if isinstance(names, (list, tuple)):
            for class_id, name in enumerate(names):
                if str(name).strip().lower() == "car":
                    return class_id
            if len(names) == 1 and str(names[0]).strip().lower() == "car":
                return 0
        # COCO fallback.  This still enforces car-only behavior.
        return 2

    def detect(self, image_path: Path, camera_id: str) -> list[Detection]:
        kwargs: dict[str, Any] = {
            "source": str(image_path),
            "conf": self.confidence,
            "iou": self.iou,
            "verbose": False,
            # Keep the model's complete candidate set through NMS, then
            # enforce car-only behavior below.  YOLO26x can emit overlapping
            # car/truck hypotheses for the same vehicle; passing
            # classes=[car] to Ultralytics suppresses the car hypothesis in
            # some of those frames before this code can inspect it.
            "max_det": 64,
            "imgsz": self.image_size,
        }
        if self.device:
            kwargs["device"] = self.device
        results = self.model.predict(**kwargs)
        detections: list[Detection] = []
        if not results:
            return detections
        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return detections
        names = getattr(self.model, "names", {})
        for box in boxes:
            cls = int(box.cls.item())
            if cls != self.car_class_id:
                continue
            conf = float(box.conf.item())
            if conf < self.confidence:
                continue
            xyxy = tuple(float(value) for value in box.xyxy[0].tolist())
            detections.append(
                Detection(
                    camera_id=camera_id,
                    bbox_xyxy=xyxy,  # type: ignore[arg-type]
                    confidence=conf,
                    class_id=cls,
                    class_name=str(names.get(cls, "car") if isinstance(names, dict) else "car"),
                )
            )
        detections.sort(key=lambda det: det.confidence, reverse=True)
        return detections


def gt_boxes_not_available(*_: Any) -> list[Detection]:
    return []


def bbox_area(detection: Detection) -> float:
    x1, y1, x2, y2 = detection.bbox_xyxy
    return max(x2 - x1, 0.0) * max(y2 - y1, 0.0)


def filter_detections_by_size(
    detections: list[Detection],
    camera_metadata: dict[str, Any],
    min_area_ratio: float,
) -> list[Detection]:
    if min_area_ratio <= 0.0:
        return detections
    image_area = float(camera_metadata["width"]) * float(camera_metadata["height"])
    min_area = min_area_ratio * image_area
    return [detection for detection in detections if bbox_area(detection) >= min_area]


def camera_frame_calibration(
    camera_metadata: dict[str, Any],
    camera_frame: dict[str, Any],
    ego_to_world: np.ndarray,
) -> dict[str, Any]:
    calibration = dict(camera_metadata)
    frame_transform = camera_frame.get("transform", camera_frame.get("sensor_transform_world"))
    if frame_transform is not None:
        calibration["transform"] = frame_transform
    else:
        relative_transform = matrix_from_record(camera_metadata["transform"])
        transform = ego_to_world @ relative_transform
        calibration["transform"] = {"matrix": transform.tolist()}
    return calibration


def camera_metadata_table(metadata: dict[str, Any]) -> dict[str, dict[str, Any]]:
    raw_cameras = metadata["sensors"]["cameras"]
    if isinstance(raw_cameras, list):
        return {str(camera["id"]): camera for camera in raw_cameras}
    if not isinstance(raw_cameras, dict):
        raise ValueError("Camera metadata must be a list or mapping.")

    table: dict[str, dict[str, Any]] = {}
    for camera_id, camera in raw_cameras.items():
        resolution = camera.get("resolution")
        if not isinstance(resolution, list) or len(resolution) != 2:
            raise ValueError(f"Camera {camera_id} must declare a two-element resolution.")
        transform = camera.get("transform", camera.get("extrinsic_ego"))
        if not isinstance(transform, dict):
            raise ValueError(f"Camera {camera_id} does not declare an ego-relative transform.")
        table[str(camera_id)] = {
            "id": str(camera_id),
            "width": int(resolution[0]),
            "height": int(resolution[1]),
            "fov": float(camera.get("fov", camera.get("fov_degrees"))),
            "transform": transform,
        }
    return table


def frame_camera_table(frame: dict[str, Any]) -> dict[str, dict[str, Any]]:
    sensors = frame["sensors"]
    nested_cameras = sensors.get("cameras")
    if isinstance(nested_cameras, dict):
        return nested_cameras
    return {
        str(sensor_id): sensor
        for sensor_id, sensor in sensors.items()
        if sensor_id != "lidar" and isinstance(sensor, dict) and "path" in sensor
    }


def frame_ego_transform(frame: dict[str, Any]) -> dict[str, Any]:
    transform = frame.get("ego_transform")
    if isinstance(transform, dict):
        return transform
    ego = frame.get("ego", {})
    transform = ego.get("transform") if isinstance(ego, dict) else None
    if not isinstance(transform, dict):
        raise ValueError("Frame does not declare an ego transform.")
    return transform


def frame_identity(frame: dict[str, Any], fallback_index: int) -> tuple[int, float]:
    frame_number = frame.get("frame", frame.get("sample_index", fallback_index))
    frame_time = frame.get("time", frame.get("time_s"))
    if frame_time is None:
        raise ValueError("Frame does not declare a timestamp.")
    return int(frame_number), float(frame_time)


def target_vehicle_dimensions(metadata: dict[str, Any]) -> tuple[float, float]:
    target = metadata.get("target", {})
    bounding_box = target.get("bounding_box", {}) if isinstance(target, dict) else {}
    if not bounding_box:
        passes = metadata.get("passes", {})
        if isinstance(passes, dict):
            for pass_record in passes.values():
                candidate = pass_record.get("target_bounding_box_m", {})
                if candidate:
                    bounding_box = candidate
                    break
    length = float(bounding_box.get("length_m", bounding_box.get("length", 4.5)))
    width = float(bounding_box.get("width_m", bounding_box.get("width", 1.9)))
    return length, width


def choose_single_detection(detections: list[Detection]) -> list[Detection]:
    if not detections:
        return []
    detections = sorted(
        detections,
        key=lambda det: (
            det.confidence,
            (det.bbox_xyxy[2] - det.bbox_xyxy[0]) * (det.bbox_xyxy[3] - det.bbox_xyxy[1]),
        ),
        reverse=True,
    )
    return detections[:1]


def detection_overlap(first: Detection, second: Detection) -> float:
    """Return image intersection-over-union for two boxes in one camera."""
    if first.camera_id != second.camera_id:
        return 0.0
    a, b = first.bbox_xyxy, second.bbox_xyxy
    intersection = max(0.0, min(a[2], b[2]) - max(a[0], b[0])) * max(
        0.0, min(a[3], b[3]) - max(a[1], b[1])
    )
    union = bbox_area(first) + bbox_area(second) - intersection
    return intersection / union if union > 0.0 else 0.0


class PersistentTargetSelector:
    """Keep the first acquired car's image track through short detection gaps.

    The first acquisition uses confidence. Later acquisitions require overlap
    with the last accepted box. A missing/expired match produces no detection;
    it never silently replaces the target. Create a new selector to acquire a
    new identity. This is a single-camera association policy, not reidentification
    across occlusions or a guarantee that the initial car is the desired target.
    """

    def __init__(self, minimum_iou: float = 0.1, maximum_gap: float = 1.0) -> None:
        if not math.isfinite(minimum_iou) or not 0.0 < minimum_iou <= 1.0:
            raise ValueError("Minimum association IoU must be finite and in (0, 1].")
        if not math.isfinite(maximum_gap) or maximum_gap <= 0.0:
            raise ValueError("Maximum association gap must be finite and positive.")
        self.minimum_iou = minimum_iou
        self.maximum_gap = maximum_gap
        self.last_detection: Detection | None = None
        self.last_match_time = -float("inf")
        self.last_timestamp = -float("inf")

    def select(self, detections: list[Detection], timestamp: float) -> list[Detection]:
        if not math.isfinite(timestamp) or timestamp <= self.last_timestamp:
            raise ValueError("Target association requires strictly increasing finite timestamps.")
        self.last_timestamp = timestamp
        if not detections:
            return []
        if self.last_detection is None:
            selected = choose_single_detection(detections)[0]
        else:
            if timestamp - self.last_match_time > self.maximum_gap:
                return []
            selected = max(detections, key=lambda d: (
                detection_overlap(self.last_detection, d), d.confidence, bbox_area(d)
            ))
            if detection_overlap(self.last_detection, selected) < self.minimum_iou:
                return []
        self.last_detection = selected
        self.last_match_time = timestamp
        return [selected]


def fuse_estimates(estimates: list[CameraEstimate]) -> tuple[np.ndarray, dict[str, Any]] | None:
    if not estimates:
        return None
    weights = np.asarray(
        [max(est.point_count, 1) * max(est.detection.confidence, 1.0e-3) for est in estimates],
        dtype=float,
    )
    positions = np.vstack([est.relative_position for est in estimates])
    fused = np.average(positions, axis=0, weights=weights)
    return fused, {
        "camera_count": len(estimates),
        "cameras": [est.camera_id for est in estimates],
        "point_counts": [est.point_count for est in estimates],
        "weights": weights.tolist(),
    }


def select_consistent_estimates(estimates: list[CameraEstimate], max_spread: float) -> list[CameraEstimate]:
    if len(estimates) <= 1 or max_spread <= 0.0:
        return estimates
    weights = np.asarray(
        [max(est.point_count, 1) * max(est.detection.confidence, 1.0e-3) for est in estimates],
        dtype=float,
    )
    anchor = estimates[int(np.argmax(weights))]
    anchor_position = anchor.relative_position
    selected = [
        estimate
        for estimate in estimates
        if float(np.linalg.norm(estimate.relative_position - anchor_position)) <= max_spread
    ]
    return selected if selected else [anchor]


def target_truth_relative(frame: dict[str, Any]) -> np.ndarray | None:
    relative = frame.get("target_relative_position_ego_m")
    if relative is not None:
        value = np.asarray(relative, dtype=float)
        return value[:2] if value.size >= 2 else None
    target = frame.get("target", {})
    relative = target.get("relative_position_ego")
    if relative is None:
        return None
    value = np.asarray(relative, dtype=float)
    return value[:2] if value.size >= 2 else None


def write_outputs(output_dir: Path, measurements: list[dict[str, Any]]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    jsonl_path = output_dir / "target_measurements.jsonl"
    with jsonl_path.open("w", encoding="utf-8") as handle:
        for item in measurements:
            handle.write(json.dumps(item, ensure_ascii=False) + "\n")

    csv_path = output_dir / "target_measurements.csv"
    fieldnames = [
        "frame",
        "time",
        "valid",
        "relativePositionX",
        "relativePositionY",
        "truthRelativePositionX",
        "truthRelativePositionY",
        "positionErrorX",
        "positionErrorY",
        "positionError",
        "cameraCount",
        "pointCount",
        "confidence",
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for item in measurements:
            writer.writerow(
                {
                    "frame": item["frame"],
                    "time": item["time"],
                    "valid": item["valid"],
                    "relativePositionX": item.get("relativePositionX", ""),
                    "relativePositionY": item.get("relativePositionY", ""),
                    "truthRelativePositionX": item.get("truthRelativePositionX", ""),
                    "truthRelativePositionY": item.get("truthRelativePositionY", ""),
                    "positionErrorX": item.get("positionErrorX", ""),
                    "positionErrorY": item.get("positionErrorY", ""),
                    "positionError": item.get("positionError", ""),
                    "cameraCount": item.get("cameraCount", 0),
                    "pointCount": item.get("pointCount", 0),
                    "confidence": item.get("confidence", ""),
                }
            )


def summarize(measurements: list[dict[str, Any]]) -> dict[str, Any]:
    valid = [item for item in measurements if item.get("valid")]
    valid_with_truth = [item for item in valid if item.get("positionError") is not None]
    errors = np.asarray(
        [item["positionError"] for item in valid_with_truth],
        dtype=float,
    )
    raw_detection_counts = np.asarray(
        [item.get("rawDetectionCount", 0) for item in measurements],
        dtype=float,
    )
    summary: dict[str, Any] = {
        "frame_count": len(measurements),
        "valid_count": len(valid),
        "truth_comparison_count": len(valid_with_truth),
        "detection_rate": len(valid) / max(len(measurements), 1),
        "localization_rate": len(valid) / max(len(measurements), 1),
        "frames_with_yolo_detection": int(np.count_nonzero(raw_detection_counts)),
        "raw_detection_count_mean": float(np.mean(raw_detection_counts))
        if raw_detection_counts.size
        else 0.0,
        "raw_detection_count_max": int(np.max(raw_detection_counts))
        if raw_detection_counts.size
        else 0,
    }
    if valid:
        confidences = np.asarray([item["confidence"] for item in valid], dtype=float)
        point_counts = np.asarray([item["pointCount"] for item in valid], dtype=float)
        summary.update(
            {
                "confidence_mean": float(np.mean(confidences)),
                "confidence_min": float(np.min(confidences)),
                "confidence_max": float(np.max(confidences)),
                "lidar_point_count_mean": float(np.mean(point_counts)),
                "lidar_point_count_min": int(np.min(point_counts)),
                "lidar_point_count_max": int(np.max(point_counts)),
            }
        )
    if errors.size:
        component_errors = np.asarray(
            [[item["positionErrorX"], item["positionErrorY"]] for item in valid_with_truth],
            dtype=float,
        )
        estimated_positions = np.asarray(
            [[item["relativePositionX"], item["relativePositionY"]] for item in valid_with_truth],
            dtype=float,
        )
        truth_positions = np.asarray(
            [
                [item["truthRelativePositionX"], item["truthRelativePositionY"]]
                for item in valid_with_truth
            ],
            dtype=float,
        )
        worst_item = valid_with_truth[int(np.argmax(errors))]
        summary.update(
            {
                "position_error_rmse_m": float(math.sqrt(np.mean(errors * errors))),
                "position_error_mean_m": float(np.mean(errors)),
                "position_error_median_m": float(np.median(errors)),
                "position_error_max_m": float(np.max(errors)),
                "position_error_p95_m": float(np.percentile(errors, 95.0)),
                "position_error_within_0_25_m_rate": float(np.mean(errors <= 0.25)),
                "position_error_within_0_30_m_rate": float(np.mean(errors <= 0.30)),
                "position_error_within_0_50_m_rate": float(np.mean(errors <= 0.50)),
                "longitudinal_error_mean_m": float(np.mean(component_errors[:, 0])),
                "longitudinal_error_rmse_m": float(
                    math.sqrt(np.mean(component_errors[:, 0] ** 2))
                ),
                "longitudinal_error_mae_m": float(np.mean(np.abs(component_errors[:, 0]))),
                "longitudinal_error_max_abs_m": float(np.max(np.abs(component_errors[:, 0]))),
                "lateral_error_mean_m": float(np.mean(component_errors[:, 1])),
                "lateral_error_rmse_m": float(
                    math.sqrt(np.mean(component_errors[:, 1] ** 2))
                ),
                "lateral_error_mae_m": float(np.mean(np.abs(component_errors[:, 1]))),
                "lateral_error_max_abs_m": float(np.max(np.abs(component_errors[:, 1]))),
                "estimated_position_mean_m": np.mean(estimated_positions, axis=0).tolist(),
                "truth_position_mean_m": np.mean(truth_positions, axis=0).tolist(),
                "worst_frame": int(worst_item["frame"]),
                "worst_time_s": float(worst_item["time"]),
            }
        )
    return summary


def estimator_track_estimates(
    associations: list[dict[str, Any]],
    args: argparse.Namespace,
    feedback: PoseFeedback,
) -> tuple[list[list[CameraEstimate]], list[float], int]:
    """Predict the current pose, fit once, then consume the measured position.

    Both geometric fitting stages precede the single estimator update. The
    prior state time must match this frame; its latest input must precede it.
    Missing detections deliver an explicit missing position to the estimator.
    """
    estimates_by_frame = []
    headings = []
    predicted_count = 0
    previous_time = -float("inf")
    for record in associations:
        stamp = float(record["time"])
        if not math.isfinite(stamp) or stamp <= previous_time:
            raise ValueError("Estimator feedback requires strictly increasing finite timestamps.")
        previous_time = stamp
        prior = feedback.predict(record["frame_number"], stamp)
        if not isinstance(prior, PosePrior):
            raise ValueError("The estimator must supply a PosePrior for every frame.")
        if not math.isfinite(prior.heading_ego_rad) or not math.isfinite(prior.state_time):
            raise ValueError("Estimator heading and state time must be finite.")
        if abs(prior.state_time - stamp) > 1.0e-9:
            raise ValueError("Estimator pose must be predicted to the current frame time.")
        if prior.source not in ("prediction", "initialization"):
            raise ValueError("Pose prior source must be prediction or initialization.")
        if prior.last_measurement_time is None:
            if prior.source != "initialization":
                raise ValueError("A predicted pose must identify its latest measurement time.")
        elif (not math.isfinite(prior.last_measurement_time)
              or prior.last_measurement_time >= stamp):
            raise ValueError("Pose feedback cannot consume the current or a future measurement.")
        if prior.position_ego_m is None:
            if prior.source != "initialization":
                raise ValueError("A predicted pose must include its relative center.")
        else:
            position = np.asarray(prior.position_ego_m, dtype=float)
            if position.shape != (2,) or not np.all(np.isfinite(position)):
                raise ValueError("Predicted relative position must contain finite x and y.")
        predicted_count += int(prior.source == "prediction")
        record["estimator_pose_prior"] = asdict(prior)
        estimates = [
            estimate for detection, cluster in record["clusters"]
            if (estimate := estimate_from_cluster(detection, cluster, args, prior)) is not None
        ]
        estimates = select_consistent_estimates(estimates, args.max_fusion_spread)
        fused = fuse_estimates(estimates)
        # A transport must not be able to mutate a published measurement.
        feedback.update(record["frame_number"], stamp, None if fused is None else fused[0].copy())
        estimates_by_frame.append(estimates)
        headings.append(wrap_angle(prior.heading_ego_rad))
    return estimates_by_frame, headings, predicted_count


def run_pipeline(
    args: argparse.Namespace,
    *,
    pose_feedback: PoseFeedback,
) -> dict[str, Any]:
    if not callable(getattr(pose_feedback, "predict", None)) or not callable(getattr(pose_feedback, "update", None)):
        raise ValueError("Predicted-pose perception requires an estimator with predict and update methods.")
    dataset_root = args.dataset.resolve()
    metadata = load_json(dataset_root / "metadata.json")
    frames = list(iter_jsonl(dataset_root / "frames.jsonl"))
    camera_table = camera_metadata_table(metadata)
    selected_camera_ids = set(args.camera_ids or camera_table)
    unknown_camera_ids = selected_camera_ids.difference(camera_table)
    if unknown_camera_ids:
        unknown = ", ".join(sorted(unknown_camera_ids))
        raise ValueError(f"Unknown camera id(s): {unknown}.")
    selector = None
    if args.target_association == "persistent":
        if len(selected_camera_ids) != 1:
            raise ValueError("Persistent target association requires exactly one selected camera.")
        selector = PersistentTargetSelector(args.association_minimum_iou, args.association_maximum_gap)
    lidar_info = metadata["sensors"]["lidar"]
    lidar_to_world_static = matrix_from_record(lidar_info)

    default_vehicle_length, default_vehicle_width = target_vehicle_dimensions(metadata)
    if args.vehicle_length <= 0.0:
        args.vehicle_length = default_vehicle_length
    if args.vehicle_width <= 0.0:
        args.vehicle_width = default_vehicle_width

    detector = UltralyticsCarDetector(
        args.model,
        args.confidence,
        args.iou,
        args.device,
        args.car_class_id,
        args.image_size,
    )

    # Detection and association are independent of the predicted pose. The
    # subsequent estimator updates still consume frames strictly in order.
    associations: list[dict[str, Any]] = []
    for frame_idx, frame in enumerate(frames):
        if args.max_frames is not None and frame_idx >= args.max_frames:
            break
        frame_number, frame_time = frame_identity(frame, frame_idx)
        ego_to_world = matrix_from_record(frame_ego_transform(frame))
        lidar_frame = frame["sensors"]["lidar"]
        lidar_to_world = matrix_from_record(lidar_frame)
        if args.use_static_lidar_transform:
            lidar_to_world = ego_to_world @ lidar_to_world_static

        lidar_points = load_lidar_points(dataset_root / lidar_frame["path"])
        clusters: list[tuple[Detection, np.ndarray]] = []
        all_detections: list[Detection] = []
        raw_detection_count = 0

        for camera_id, camera_frame in frame_camera_table(frame).items():
            if camera_id not in camera_table or camera_id not in selected_camera_ids:
                continue
            image_path = dataset_root / camera_frame["path"]
            detections = detector.detect(image_path, camera_id)
            raw_detection_count += len(detections)
            detections = filter_detections_by_size(
                detections,
                camera_table[camera_id],
                args.min_bbox_area_ratio,
            )
            detections = (selector.select(detections, frame_time) if selector is not None
                          else choose_single_detection(detections))
            all_detections.extend(detections)
            camera_calibration = camera_frame_calibration(
                camera_table[camera_id],
                camera_frame,
                ego_to_world,
            )
            for detection in detections:
                cluster = associate_detection_points(
                    detection,
                    lidar_points,
                    lidar_to_world,
                    camera_calibration,
                    ego_to_world,
                    args,
                )
                if cluster is not None:
                    clusters.append((detection, cluster))

        associations.append(
            {
                "frame": frame,
                "frame_idx": frame_idx,
                "frame_number": frame_number,
                "time": frame_time,
                "ego_to_world": ego_to_world,
                "clusters": clusters,
                "detections": all_detections,
                "raw_detection_count": raw_detection_count,
            }
        )
        if args.report_interval > 0 and (frame_idx % args.report_interval == 0):
            print(
                f"[associate {frame_idx:05d}] t={frame_time:.2f}s "
                f"detections={raw_detection_count} clusters={len(clusters)}"
            )

    estimates_by_frame, headings, predicted_count = estimator_track_estimates(
        associations, args, pose_feedback
    )

    measurements: list[dict[str, Any]] = []
    for record, camera_estimates, heading in zip(
        associations, estimates_by_frame, headings
    ):
        frame = record["frame"]
        frame_idx = record["frame_idx"]
        frame_number = record["frame_number"]
        frame_time = record["time"]
        all_detections = record["detections"]
        raw_detection_count = record["raw_detection_count"]
        fused = fuse_estimates(camera_estimates)
        truth_relative = target_truth_relative(frame)
        item: dict[str, Any] = {
            "frame": frame_number,
            "time": frame_time,
            "targetIdx": 1,
            "valid": fused is not None,
            "rawDetectionCount": raw_detection_count,
            "acceptedDetectionCount": len(all_detections),
            "usedDetectionCount": len(camera_estimates),
        }
        item["estimatorPosePrior"] = record["estimator_pose_prior"]
        item["headingSource"] = "point-cloud fit initialized by predicted pose"
        if truth_relative is not None:
            item.update(
                {
                    "truthRelativePositionX": float(truth_relative[0]),
                    "truthRelativePositionY": float(truth_relative[1]),
                }
            )
        if fused is not None:
            position, fusion_info = fused
            errors = None
            component_errors = None
            if truth_relative is not None:
                component_errors = position - truth_relative
                errors = float(np.linalg.norm(component_errors))
            item.update(
                {
                    "relativePositionX": float(position[0]),
                    "relativePositionY": float(position[1]),
                    "radarRelativePosition": [float(position[0]), float(position[1])],
                    "positionError": errors,
                    "cameraCount": fusion_info["camera_count"],
                    "pointCount": int(sum(est.point_count for est in camera_estimates)),
                    "confidence": float(max(est.detection.confidence for est in camera_estimates)),
                    "fusion": fusion_info,
                    "predictedHeadingRad": heading,
                    "cameraEstimates": [
                        {
                            "cameraId": est.camera_id,
                            "relativePosition": est.relative_position.tolist(),
                            "pointCount": est.point_count,
                            "confidence": est.detection.confidence,
                            "bbox": list(est.detection.bbox_xyxy),
                            "capsule": est.capsule,
                            "rectangle": est.rectangle,
                        }
                        for est in camera_estimates
                    ],
                }
            )
            if component_errors is not None:
                item.update(
                    {
                        "positionErrorX": float(component_errors[0]),
                        "positionErrorY": float(component_errors[1]),
                    }
                )
        measurements.append(item)

        if args.report_interval > 0 and (frame_idx % args.report_interval == 0):
            status = "valid" if item["valid"] else "miss"
            error_text = ""
            if item.get("positionError") is not None:
                error_text = f" err={item['positionError']:.3f} m"
            print(
                f"[frame {frame_idx:05d}] t={frame_time:.2f}s {status} "
                f"detections={item['rawDetectionCount']} used={item['usedDetectionCount']}{error_text}"
            )

    write_outputs(args.output, measurements)
    summary = summarize(measurements)
    summary.update(
        {
            "model": str(args.model.resolve()),
            "dataset": str(dataset_root),
            "camera_ids": sorted(selected_camera_ids),
            "image_size": args.image_size,
            "confidence_threshold": args.confidence,
            "target_association": args.target_association,
            "association_minimum_iou": args.association_minimum_iou,
            "association_maximum_gap_s": args.association_maximum_gap,
            "iou_threshold": args.iou,
            "vehicle_length_m": args.vehicle_length,
            "vehicle_width_m": args.vehicle_width,
            "center_mode": "rectangle",
            "rectangle_support_quantile_percent": args.rectangle_support_quantile,
            "rectangle_placement": "current point support",
            "rectangle_symmetry_minimum_support": args.rectangle_symmetry_minimum_support,
            "tracking_method": "predicted pose initialization",
            "heading_source": "point-cloud fit initialized by predicted pose",
            "temporal_mode": "causal, frozen past outputs",
            "predicted_pose_frame_count": predicted_count,
            "initialization_frame_count": len(associations) - predicted_count,
            "rectangle_outlier_tolerance_m": args.rectangle_outlier_tolerance,
            "minimum_support_fraction": args.minimum_support_fraction,
            "min_object_height_m": args.min_object_height,
            "synchronization": metadata.get("synchronization"),
        }
    )
    with (args.output / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, ensure_ascii=False)
    return summary


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True, help="Dataset root containing metadata.json and frames.jsonl.")
    parser.add_argument("--model", type=Path, required=True, help="YOLO car detector model, e.g. yolo26n.pt.")
    parser.add_argument("--output", type=Path, default=Path("simulation_output"), help="Output directory.")
    parser.add_argument("--confidence", type=float, default=0.18, help="YOLO confidence threshold.")
    parser.add_argument("--iou", type=float, default=0.45, help="YOLO NMS IoU threshold.")
    parser.add_argument("--device", default=None, help="Ultralytics device string.")
    parser.add_argument("--image-size", type=int, default=640, help="Square YOLO inference size in pixels.")
    parser.add_argument(
        "--camera-id",
        action="append",
        dest="camera_ids",
        default=None,
        help="Camera id to process; repeat for multiple cameras. The default processes all cameras.",
    )
    parser.add_argument("--car-class-id", type=int, default=None, help="Override the model's car class id.")
    parser.add_argument(
        "--min-lidar-points",
        type=int,
        default=10,
        help=(
            "Returns required inside a detection box. The support-fraction "
            "gate rejects clusters that cannot anchor the rectangle, so this "
            "only has to keep the fit numerically sane."
        ),
    )
    parser.add_argument("--min-depth", type=float, default=1.0)
    parser.add_argument(
        "--target-association", choices=["confidence", "persistent"], default="confidence",
        help="Persistent keeps the first acquired car using image overlap; select exactly one camera.",
    )
    parser.add_argument("--association-minimum-iou", type=float, default=0.1)
    parser.add_argument(
        "--association-maximum-gap", type=float, default=1.0,
        help="Seconds since the last matched box before the persistent track expires; no automatic replacement.",
    )
    parser.add_argument("--max-depth", type=float, default=80.0)
    parser.add_argument("--max-depth-span", type=float, default=8.0)
    parser.add_argument("--bbox-margin", type=float, default=0.02)
    parser.add_argument("--min-bbox-area-ratio", type=float, default=0.002)
    parser.add_argument(
        "--min-object-height",
        type=float,
        default=0.3,
        help=(
            "Height above the ego origin, which sits near the vehicle floor, "
            "below which returns are treated as ground. Ground points inside "
            "the image box drag the near support of the fit toward the sensor."
        ),
    )
    parser.add_argument(
        "--minimum-support-fraction",
        type=float,
        default=0.5,
        help=(
            "Largest observed span, as a fraction of the vehicle extent along "
            "the same axis, required to accept a fit. A cluster smaller than "
            "this in both directions cannot anchor a known-size rectangle."
        ),
    )
    parser.add_argument("--cluster-radius", type=float, default=1.2)
    parser.add_argument("--max-cluster-points", type=int, default=1600)
    parser.add_argument("--max-fusion-spread", type=float, default=4.0)
    parser.add_argument("--vehicle-length", type=float, default=-1.0)
    parser.add_argument("--vehicle-width", type=float, default=-1.0)
    parser.add_argument("--use-static-lidar-transform", action="store_true")
    parser.add_argument("--rectangle-support-quantile", type=float, default=2.0)
    parser.add_argument(
        "--rectangle-symmetry-minimum-support", type=float, default=0.8,
        help="Minimum fraction of a known dimension observed before symmetric placement, in (0, 1].",
    )
    parser.add_argument("--rectangle-outlier-tolerance", type=float, default=0.15)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--report-interval", type=int, default=25)
    parser.add_argument(
        "--estimator-factory",
        help="Required by the CLI: module:callable that accepts these arguments and returns a PoseFeedback adapter.",
    )
    args = parser.parse_args(argv)
    if not 0.0 < args.rectangle_symmetry_minimum_support <= 1.0:
        parser.error("--rectangle-symmetry-minimum-support must be in (0, 1].")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if not args.estimator_factory or ":" not in args.estimator_factory:
        raise SystemExit("Supply --estimator-factory module:callable to connect the target estimator.")
    module_name, factory_name = args.estimator_factory.rsplit(":", 1)
    factory = getattr(importlib.import_module(module_name), factory_name)
    feedback = factory(args)
    try:
        summary = run_pipeline(args, pose_feedback=feedback)
    finally:
        close = getattr(feedback, "close", None)
        if callable(close):
            close()
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
