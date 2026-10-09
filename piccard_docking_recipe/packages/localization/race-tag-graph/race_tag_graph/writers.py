"""Outputs: the trajectories and landmarks as CSV, the dock point as CSV (the lab fuser's /dock_point/pose semantics:
race_station/dock_point in race_auv/base_link), the results JSON the page builder reads, and figures."""
from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from .dock import DockFit, dock_in_base
from .factors import Problem
from .geometry import xyzw
from .metrics import RESULT_CLASSES
from .solvers import Solution

RESULTS_SCHEMA = "piccard.race-tag-graph.results/v1"


def write_trajectory(path: Path, problem: Problem, solutions: list) -> None:
    with Path(path).open("w", newline="") as stream:
        writer = csv.writer(stream)
        header = ["stamp", "ekf_x", "ekf_y", "ekf_z", "ekf_qx", "ekf_qy", "ekf_qz", "ekf_qw"]
        for solution in solutions:
            header += [f"{solution.solver}_{c}" for c in ("x", "y", "z", "qx", "qy", "qz", "qw")]
        writer.writerow(header)
        for i, keyframe in enumerate(problem.keyframes):
            row = [f"{keyframe.stamp:.9f}", *np.asarray(keyframe.ekf.translation()), *xyzw(keyframe.ekf.rotation())]
            for solution in solutions:
                pose = solution.poses[i]
                row += [*np.asarray(pose.translation()), *xyzw(pose.rotation())]
            writer.writerow([row[0], *(f"{float(v):.6f}" for v in row[1:])])


def write_landmarks(path: Path, solutions: list) -> None:
    with Path(path).open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["solver", "tag", "x", "y", "z"])
        for solution in solutions:
            for tag, position in sorted(solution.landmarks.items()):
                writer.writerow([solution.solver, tag, *(f"{float(v):.6f}" for v in position)])


def write_dock_point(path: Path, problem: Problem, solution: Solution, dock: DockFit) -> int:
    """One row per keyframe where the dock pose is fitted; returns the row count."""
    frames = problem.config["frames"]
    rows = 0
    with Path(path).open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["stamp", "frame_id", "child_frame_id", "x", "y", "z", "qx", "qy", "qz", "qw"])
        for keyframe, pose in zip(problem.keyframes, solution.poses):
            relative = dock_in_base(pose, dock)
            if relative is None:
                continue
            writer.writerow([f"{keyframe.stamp:.9f}", frames["base"], frames["dock_output"],
                             *(f"{float(v):.6f}" for v in (*np.asarray(relative.translation()),
                                                           *xyzw(relative.rotation())))])
            rows += 1
    return rows


def check_result_classes(metrics: dict) -> None:
    for name, metric in metrics.items():
        if not isinstance(metric, dict) or metric.get("result_class") not in RESULT_CLASSES:
            raise ValueError(f"metric {name!r} has no valid result_class (one of {RESULT_CLASSES})")


def results(problem: Problem, solutions: list, dock: DockFit, metrics: dict, inputs: dict) -> dict:
    check_result_classes(metrics)
    config = {k: v for k, v in problem.config.items() if not k.startswith("_")}
    return {
        "schema": RESULTS_SCHEMA,
        "inputs": inputs,
        "config": config,
        "config_sha256": hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest(),
        "counts": problem.counts,
        "notes": list(problem.notes),
        "solvers": [{"solver": s.solver, "initial_error": s.initial_error, "final_error": s.final_error,
                     "iterations": s.iterations, "elapsed_s": s.elapsed_s} for s in solutions],
        "landmarks": {s.solver: {tag: [float(v) for v in p] for tag, p in sorted(s.landmarks.items())}
                      for s in solutions},
        "dock": {"status": dock.status, "tags": dock.tags, "up": dock.up, "yaw_baseline_m": dock.yaw_baseline_m,
                 "residual_rms_m": dock.residual_rms_m,
                 "pivot_odom_m": None if dock.pivot is None else [float(v) for v in dock.pivot],
                 "pose_odom": None if dock.pose is None else {
                     "position_m": [float(v) for v in np.asarray(dock.pose.translation())],
                     "orientation_q": xyzw(dock.pose.rotation())}},
        "metrics": metrics,
    }


def write_results(path: Path, document: dict) -> None:
    check_result_classes(document["metrics"])
    Path(path).write_text(json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n")


def write_figures(directory: Path, problem: Problem, solutions: list) -> list:
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot

    directory = Path(directory)
    written = []
    figure, axis = pyplot.subplots(figsize=(7, 6))
    ekf = np.array([np.asarray(k.ekf.translation()) for k in problem.keyframes])
    axis.plot(ekf[:, 0], ekf[:, 1], label="EKF", linewidth=1.0)
    for solution in solutions:
        path = np.array([np.asarray(p.translation()) for p in solution.poses])
        axis.plot(path[:, 0], path[:, 1], label=solution.solver, linewidth=1.0)
        if solution.landmarks:
            marks = np.array(list(solution.landmarks.values()))
            axis.scatter(marks[:, 0], marks[:, 1], marker="x", label=f"{solution.solver} landmarks")
    axis.set_xlabel("odom x (m)")
    axis.set_ylabel("odom y (m)")
    axis.set_aspect("equal", adjustable="datalim")
    axis.legend()
    axis.set_title("Trajectory in the odom frame (top view)")
    figure.tight_layout()
    figure.savefig(directory / "trajectory.png", dpi=120)
    pyplot.close(figure)
    written.append("trajectory.png")
    return written
