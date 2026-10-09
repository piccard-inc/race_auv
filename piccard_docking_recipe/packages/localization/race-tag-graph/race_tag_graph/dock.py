"""The dock point from the tag landmarks and the station's tag layout (operational geometry from the lab's station
URDF, config/station_layout.yaml; never ground truth).

The fit is level-constrained: the station is taken as level in the EKF's gravity-aligned odom frame, so the dock
pose is a yaw about the vertical, the dock frame's vertical sign (z_up or z_down against odom's up, chosen by the
fit's residual when frame_up is auto) and a translation. Yaw needs horizontal spread among the observed tags: the
forward camera's three tags sit on the station's vertical centreline (4 cm apart horizontally), so with only those
the yaw is unobserved, the dock pose is not reported, and the tags' centroid (the pivot) is reported instead. The two
flat tags the downward camera sees, 0.5 m apart, fix it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import gtsam
import numpy as np

FLIPS = {"z_up": np.eye(3), "z_down": np.diag([1.0, -1.0, -1.0])}


@dataclass
class DockFit:
    status: str                     # fitted | yaw_unobserved | up_ambiguous | too_few_tags
    pose: gtsam.Pose3 | None        # the dock frame in the odom frame
    pivot: np.ndarray | None        # the observed tags' centroid in the odom frame
    tags: list
    up: str | None
    yaw_baseline_m: float
    residual_rms_m: float | None


def _fit(layout_points: np.ndarray, points: np.ndarray, flip: np.ndarray):
    a = layout_points @ flip.T
    a_mean, b_mean = a.mean(axis=0), points.mean(axis=0)
    ah, bh = a[:, :2] - a_mean[:2], points[:, :2] - b_mean[:2]
    yaw = math.atan2(float(np.sum(ah[:, 0] * bh[:, 1] - ah[:, 1] * bh[:, 0])),
                     float(np.sum(ah[:, 0] * bh[:, 0] + ah[:, 1] * bh[:, 1])))
    c, s = math.cos(yaw), math.sin(yaw)
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    rotation = rz @ flip
    translation = b_mean - rotation @ layout_points.mean(axis=0)
    residual = points - (layout_points @ rotation.T + translation)
    return rotation, translation, float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))


def fit_dock(landmarks: dict, layout: dict, config: dict) -> DockFit:
    cfg = config["dock"]
    tags = sorted(tag for tag in landmarks if tag in layout)
    if not tags:
        return DockFit("too_few_tags", None, None, [], None, 0.0, None)
    points = np.array([landmarks[tag] for tag in tags], dtype=float)
    pivot = points.mean(axis=0)
    if len(tags) < 2:
        return DockFit("too_few_tags", None, pivot, tags, None, 0.0, None)
    layout_points = np.array([layout[tag]["position"] for tag in tags], dtype=float)
    horizontal = layout_points[:, :2]
    baseline = max(float(np.linalg.norm(p - q)) for p in horizontal for q in horizontal)
    choices = list(FLIPS) if cfg["frame_up"] == "auto" else [cfg["frame_up"]]
    fits = sorted(((_fit(layout_points, points, FLIPS[name]), name) for name in choices), key=lambda f: f[0][2])
    (rotation, translation, residual), up = fits[0]
    if len(fits) > 1 and fits[1][0][2] < cfg["up_min_residual_ratio"] * max(residual, 1e-9):
        return DockFit("up_ambiguous", None, pivot, tags, None, baseline, residual)
    if baseline < cfg["yaw_min_baseline_m"]:
        return DockFit("yaw_unobserved", None, pivot, tags, up, baseline, residual)
    return DockFit("fitted", gtsam.Pose3(gtsam.Rot3(rotation), translation), pivot, tags, up, baseline, residual)


def dock_in_base(pose: gtsam.Pose3, dock: DockFit):
    """The dock point in base_link at a keyframe (the lab fuser's /dock_point/pose semantics), or None."""
    if dock.pose is None:
        return None
    return pose.inverse().compose(dock.pose)
