"""Two solvers over one builder's factors: a batch Levenberg-Marquardt solve, and an ISAM2 replay that consumes the
keyframes in time order (the path to the lab's gtsam_dock_fuser node, phases 1-2). Neither adds or changes a factor."""
from __future__ import annotations

import time
from dataclasses import dataclass

import gtsam
import numpy as np
from gtsam.symbol_shorthand import L, O, X

from .factors import Problem


@dataclass
class Solution:
    solver: str
    poses: list                 # gtsam.Pose3 per keyframe, odom frame
    landmarks: dict             # tag -> position in the odom frame
    orientations: dict          # tag -> Rot3 in the odom frame (orientation path only)
    initial_error: float
    final_error: float
    iterations: int | None
    elapsed_s: float


def _solution(name: str, problem: Problem, values: gtsam.Values, initial_error: float, final_error: float,
              iterations, started: float) -> Solution:
    return Solution(solver=name, poses=[values.atPose3(X(k.index)) for k in problem.keyframes],
                    landmarks={tag: np.asarray(values.atPoint3(L(i))) for tag, i in problem.landmarks.items()},
                    orientations={tag: values.atRot3(O(i)) for tag, i in problem.landmarks.items()
                                  if values.exists(O(i))},
                    initial_error=float(initial_error), final_error=float(final_error), iterations=iterations,
                    elapsed_s=time.monotonic() - started)


def solve_batch(problem: Problem) -> Solution:
    started = time.monotonic()
    graph, initial = problem.graph(), problem.initial()
    cfg = problem.config["batch"]
    params = gtsam.LevenbergMarquardtParams()
    params.setMaxIterations(int(cfg["max_iterations"]))
    params.setRelativeErrorTol(float(cfg["relative_error_tol"]))
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    result = optimizer.optimize()
    return _solution("batch", problem, result, graph.error(initial), graph.error(result), optimizer.iterations(),
                     started)


def solve_isam2(problem: Problem) -> Solution:
    """One ISAM2 update per keyframe, in time order. A new keyframe starts from the current estimate of the previous
    one composed with the EKF increment; a new landmark from its first sighting seen from that start."""
    started = time.monotonic()
    cfg = problem.config["isam2"]
    params = gtsam.ISAM2Params()
    params.setRelinearizeThreshold(float(cfg["relinearize_threshold"]))
    params.relinearizeSkip = int(cfg["relinearize_skip"])
    isam = gtsam.ISAM2(params)
    previous = None
    for step in problem.steps:
        k = step.keyframe
        start = problem.keyframes[0].ekf if k == 0 else previous.compose(step.odometry)
        graph, values = gtsam.NonlinearFactorGraph(), gtsam.Values()
        for factor in step.factors:
            graph.add(factor)
        values.insert(X(k), start)
        for tag, base_point in step.new_landmarks.items():
            values.insert(L(problem.landmarks[tag]), start.transformFrom(base_point))
        for tag, base_rotation in step.new_orientations.items():
            values.insert(O(problem.landmarks[tag]), start.rotation().compose(base_rotation))
        isam.update(graph, values)
        previous = isam.calculateEstimatePose3(X(k))
    for _ in range(int(cfg["final_updates"])):
        isam.update()
    result = isam.calculateEstimate()
    graph = problem.graph()
    return _solution("isam2", problem, result, graph.error(problem.initial()), graph.error(result), None, started)
