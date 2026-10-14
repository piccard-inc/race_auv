"""Evaluation only: metrics against ground truth (result_class scoring), for simulation. Kept apart from the
consistency metrics (metrics.py), which need no truth.

- position_error_vs_truth: base_link position error over the keyframes, of the EKF and of the candidate, after
  aligning each trajectory's first pose to the truth (planar yaw and translation, as the D0 drift audit and
  runtime/docking_metric.py do);
- error_at_resightings_vs_truth: the same error at the re-sighting keyframes;
- dock_point_error_vs_truth: the candidate's dock point in base_link, and the lab fuser's, against the true station
  dock point in base_link (pairs within PAIR_S).
"""
from __future__ import annotations

import bisect
import math

import gtsam
import numpy as np

from .dock import DockFit, dock_in_base
from .factors import Problem
from .metrics import SCORING, resightings, summary
from .solvers import Solution
from .truth import Truth

PAIR_S = 0.2


def is_development(trial: str | None, config: dict) -> bool:
    """Whether a trial is a development trial (config evaluation.development_trials): the range model was chosen
    after looking at it, so it is never part of the headline evaluation."""
    return trial is not None and trial in set(config["evaluation"]["development_trials"])


def headline_trials(trials, config: dict) -> list:
    """The trials a headline evaluation may use: every given trial except the development trials."""
    return [trial for trial in trials if not is_development(trial, config)]


def _heading(rotation: np.ndarray) -> float:
    return math.atan2(rotation[1, 0], rotation[0, 0])


def _aligned_errors(poses: list, stamps: list, truth: Truth) -> dict:
    """{keyframe index: error} after the first pose with truth is aligned (planar yaw + translation)."""
    pairs = [(i, poses[i], truth.base_link.at(s)) for i, s in enumerate(stamps)]
    pairs = [p for p in pairs if p[2] is not None]
    if not pairs:
        return {}
    _, first, first_truth = pairs[0]
    yaw = _heading(first_truth.rotation().matrix()) - _heading(first.rotation().matrix())
    rz = gtsam.Rot3.Rz(yaw).matrix()
    origin = np.asarray(first_truth.translation()) - rz @ np.asarray(first.translation())
    return {i: float(np.linalg.norm(np.asarray(t.translation()) - (rz @ np.asarray(p.translation()) + origin)))
            for i, p, t in pairs}


def _nearest(rows: list, stamp: float):
    stamps = [r[0] for r in rows]
    i = bisect.bisect_left(stamps, stamp)
    best = min((j for j in (i - 1, i) if 0 <= j < len(rows)), key=lambda j: abs(stamps[j] - stamp), default=None)
    if best is None or abs(stamps[best] - stamp) > PAIR_S:
        return None
    return rows[best][1]


def scoring(problem: Problem, solution: Solution, truth: Truth, dock: DockFit) -> dict:
    stamps = [k.stamp for k in problem.keyframes]
    ekf = _aligned_errors([k.ekf for k in problem.keyframes], stamps, truth)
    candidate = _aligned_errors(solution.poses, stamps, truth)
    revisit_keyframes = sorted({problem.observations[i].keyframe for i, _ in resightings(problem)})
    candidate_dock, fuser_dock = [], []
    for keyframe, pose in zip(problem.keyframes, solution.poses):
        true_dock = _nearest(truth.dock_in_base, keyframe.stamp)
        if true_dock is None:
            continue
        estimate = dock_in_base(pose, dock)
        if estimate is not None:
            candidate_dock.append(float(np.linalg.norm(np.asarray(estimate.translation()) - true_dock)))
    for stamp, position in truth.lab_fuser:
        true_dock = _nearest(truth.dock_in_base, stamp)
        if true_dock is not None:
            fuser_dock.append(float(np.linalg.norm(position - true_dock)))
    last = max(ekf) if ekf else None
    return {
        "position_error_vs_truth": {
            "result_class": SCORING,
            "definition": "base_link position error over the keyframes after first-pose alignment (planar yaw + "
                          "translation) to the simulator's truth",
            "ekf_m": summary(ekf.values()), "candidate_m": summary(candidate.values()),
            "final": {"ekf_m": ekf.get(last), "candidate_m": candidate.get(last)}},
        "error_at_resightings_vs_truth": {
            "result_class": SCORING,
            "definition": "the same error at the keyframes of re-sightings after >= revisit_span_s",
            "ekf_m": summary([ekf[k] for k in revisit_keyframes if k in ekf]),
            "candidate_m": summary([candidate[k] for k in revisit_keyframes if k in candidate])},
        "dock_point_error_vs_truth": {
            "result_class": SCORING,
            "definition": f"dock point in base_link against the true station dock point in base_link, pairs within "
                          f"{PAIR_S} s; the candidate only where its dock pose is fitted",
            "candidate_status": dock.status,
            "candidate_m": summary(candidate_dock), "lab_fuser_m": summary(fuser_dock)},
    }
