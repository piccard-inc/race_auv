"""Consistency metrics that need no truth (result_class sensor-grounded): what Yuewei's bag can give.

- resighting_residuals: each accepted detection's residual against its landmark, before (EKF poses, landmarks at
  first sighting) and after the solve, over all detections and at re-sightings;
- drift_at_resightings: at each re-sighting of a tag after an out-of-view span, the jump of its projected position
  since its last sighting, through the EKF's poses and through the solved poses (re-anchoring is not improvement:
  a smaller jump through the solved poses only shows the trajectory is consistent with the landmark);
- dock_point_stability: the dock point (or the tag pivot when yaw is unobserved) fitted separately per in-view
  segment, through the EKF's poses and through the solved poses, and its spread across segments; the lab fuser's
  recorded dock point per segment when it is supplied as a comparison input;
- range_scale_residual: measured over predicted range of each accepted detection after the solve, reported, never
  absorbed (no scale parameter is estimated).
"""
from __future__ import annotations

import itertools

import numpy as np
from gtsam.symbol_shorthand import L

from .dock import fit_dock
from .factors import Problem, load_layout
from .geometry import PoseSeries
from .solvers import Solution

SENSOR_GROUNDED = "sensor-grounded"
SCORING = "scoring"
RESULT_CLASSES = (SENSOR_GROUNDED, SCORING)


def summary(values) -> dict:
    values = [float(v) for v in values]
    if not values:
        return {"n": 0}
    array = np.asarray(values)
    return {"n": len(values), "median": float(np.median(array)), "p90": float(np.percentile(array, 90)),
            "max": float(array.max()), "mean": float(array.mean())}


def resightings(problem: Problem) -> list:
    """(observation index, previous observation index) for each tag's first accepted sighting after a gap of at least
    metrics.revisit_span_s since its previous accepted sighting."""
    span = problem.config["metrics"]["revisit_span_s"]
    last, out = {}, []
    for i, observation in enumerate(problem.observations):
        if not observation.accepted:
            continue
        previous = last.get(observation.tag)
        if previous is not None and observation.stamp - problem.observations[previous].stamp >= span:
            out.append((i, previous))
        last[observation.tag] = i
    return out


def segments(problem: Problem) -> list:
    """Lists of accepted observation indices, split wherever no tag was seen for metrics.revisit_span_s."""
    span = problem.config["metrics"]["revisit_span_s"]
    out, current, last = [], [], None
    for i, observation in enumerate(problem.observations):
        if not observation.accepted:
            continue
        if last is not None and observation.stamp - last >= span:
            out.append(current)
            current = []
        current.append(i)
        last = observation.stamp
    if current:
        out.append(current)
    return out


def _residual(pose, landmark, base_point) -> float:
    return float(np.linalg.norm(pose.transformTo(landmark) - base_point))


def _segment_dock(problem: Problem, indices: list, poses: list, layout: dict) -> dict:
    by_tag = {}
    for i in indices:
        observation = problem.observations[i]
        by_tag.setdefault(observation.tag, []).append(poses[observation.keyframe].transformFrom(observation.base_point))
    landmarks = {tag: np.mean(np.asarray(points), axis=0) for tag, points in by_tag.items()}
    fit = fit_dock(landmarks, layout, problem.config)
    return {"status": fit.status, "tags": fit.tags,
            "dock_position": None if fit.pose is None else np.asarray(fit.pose.translation()),
            "pivot": fit.pivot}


def _spread(points: list) -> float | None:
    points = [p for p in points if p is not None]
    if len(points) < 2:
        return None
    return max(float(np.linalg.norm(a - b)) for a, b in itertools.combinations(points, 2))


def sensor_grounded(problem: Problem, solution: Solution, lab_fuser=None) -> dict:
    """The consistency metrics. lab_fuser: optional [(stamp, position in base_link)] from the recorded
    /race_station/dock_point/pose, a comparison input read on the evaluation side."""
    layout = load_layout(problem.config)
    ekf = [k.ekf for k in problem.keyframes]
    initial = problem.initial()
    first = {tag: np.asarray(initial.atPoint3(L(i))) for tag, i in problem.landmarks.items()}
    accepted = [i for i, o in enumerate(problem.observations) if o.accepted]
    before = [_residual(ekf[problem.observations[i].keyframe], first[problem.observations[i].tag],
                        problem.observations[i].base_point) for i in accepted]
    after = [_residual(solution.poses[problem.observations[i].keyframe],
                       solution.landmarks[problem.observations[i].tag], problem.observations[i].base_point)
             for i in accepted]
    revisits = resightings(problem)
    at_before = [_residual(ekf[problem.observations[i].keyframe], first[problem.observations[i].tag],
                           problem.observations[i].base_point) for i, _ in revisits]
    at_after = [_residual(solution.poses[problem.observations[i].keyframe],
                          solution.landmarks[problem.observations[i].tag], problem.observations[i].base_point)
                for i, _ in revisits]

    def jump(poses, i, j):
        a, b = problem.observations[i], problem.observations[j]
        return float(np.linalg.norm(poses[a.keyframe].transformFrom(a.base_point) -
                                    poses[b.keyframe].transformFrom(b.base_point)))

    segment_list = segments(problem)
    stability = {"segments": len(segment_list)}
    for name, poses in (("ekf", ekf), ("candidate", solution.poses)):
        fits = [_segment_dock(problem, s, poses, layout) for s in segment_list]
        stability[name] = {"dock_spread_m": _spread([f["dock_position"] for f in fits]),
                           "pivot_spread_m": _spread([f["pivot"] for f in fits]),
                           "statuses": [f["status"] for f in fits]}
    if lab_fuser:
        series = PoseSeries([k.stamp for k in problem.keyframes], ekf)
        per_segment = []
        for s in segment_list:
            t0, t1 = problem.observations[s[0]].stamp, problem.observations[s[-1]].stamp
            points = [series.at(t).transformFrom(np.asarray(p)) for t, p in lab_fuser
                      if t0 <= t <= t1 and series.at(t) is not None]
            per_segment.append(np.median(np.asarray(points), axis=0) if points else None)
        stability["lab_fuser"] = {"dock_spread_m": _spread(per_segment),
                                  "note": "the recorded dock point mapped through the EKF's poses, per segment"}

    def scale(indices):
        ratios, numerator, denominator = [], 0.0, 0.0
        for i in indices:
            observation = problem.observations[i]
            extrinsic = problem.extrinsics[observation.camera]
            predicted = extrinsic.transformTo(solution.poses[observation.keyframe].transformTo(
                solution.landmarks[observation.tag]))
            ratios.append(float(np.linalg.norm(observation.measured) / np.linalg.norm(predicted)))
            numerator += float(observation.measured @ predicted)
            denominator += float(predicted @ predicted)
        return {"ratio": summary(ratios), "fitted_scale": numerator / denominator if denominator else None}

    return {
        "resighting_residuals": {
            "result_class": SENSOR_GROUNDED,
            "definition": "|landmark seen from the keyframe - detection|, in base_link, before (EKF poses, landmarks "
                          "at first sighting) and after the solve",
            "all": {"before_m": summary(before), "after_m": summary(after)},
            "at_resightings": {"before_m": summary(at_before), "after_m": summary(at_after)}},
        "drift_at_resightings": {
            "result_class": SENSOR_GROUNDED,
            "definition": "at a re-sighting after >= revisit_span_s, the jump of the tag's projected position since "
                          "its previous sighting; re-anchoring is not counted as improvement",
            "resightings": len(revisits),
            "ekf_m": summary([jump(ekf, i, j) for i, j in revisits]),
            "candidate_m": summary([jump(solution.poses, i, j) for i, j in revisits])},
        "dock_point_stability": {
            "result_class": SENSOR_GROUNDED,
            "definition": "the dock point (or the tag pivot) fitted per in-view segment; its largest pairwise "
                          "distance across segments",
            **stability},
        "range_scale_residual": {
            "result_class": SENSOR_GROUNDED,
            "definition": "measured / predicted range of each accepted detection after the solve, and a scale fitted "
                          "afterwards; reported only (the estimator has no scale parameter). range_used: the "
                          "detections whose range constrains the graph; all: every accepted detection",
            "range_used": scale([i for i in accepted if problem.observations[i].range_used]),
            "all": scale(accepted)},
    }
