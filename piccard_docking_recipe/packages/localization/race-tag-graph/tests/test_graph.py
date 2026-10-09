"""The formulation and the two solvers on synthetic telemetry (tests/synthetic.py): odometry enters once, the
landmarks start at first sighting, orientation is off by default, the gate rejects an outlier, batch and ISAM2
agree, and the drift is reduced against the synthetic truth."""
import copy
import math
import tempfile
import unittest
from pathlib import Path

import gtsam
import numpy as np

import synthetic
from race_tag_graph import dock, evaluate, factors, sensors, solvers, truth


def config(**changes):
    value = copy.deepcopy(factors.load_config())
    for dotted, setting in changes.items():
        section, key = dotted.split("__")
        value[section][key] = setting
    return value


class GraphTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.path = synthetic.write(Path(cls.tmp.name) / "telemetry.jsonl")
        cls.stream = sensors.read_telemetry(cls.path)
        cls.problem = factors.build(cls.stream, config())
        cls.batch = solvers.solve_batch(cls.problem)
        cls.isam2 = solvers.solve_isam2(cls.problem)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_odometry_enters_once_as_motion(self):
        kinds = [type(f).__name__ for step in self.problem.steps for f in step.factors]
        keyframes = len(self.problem.keyframes)
        self.assertEqual(kinds.count("PriorFactorPose3"), 1)
        self.assertEqual(kinds.count("BetweenFactorPose3"), keyframes - 1)
        self.assertEqual(kinds.count("CustomFactor"), self.problem.counts["detections"])
        self.assertEqual(len(kinds), 1 + (keyframes - 1) + self.problem.counts["detections"])
        self.assertIsInstance(self.problem.steps[0].factors[0], gtsam.PriorFactorPose3)
        for step in self.problem.steps:
            for factor in step.factors:
                if isinstance(factor, gtsam.CustomFactor):
                    self.assertEqual(len(factor.keys()), 2)   # a pose and a landmark, never a pose alone

    def test_each_landmark_starts_at_its_first_sighting(self):
        initial = self.problem.initial()
        for tag, index in self.problem.landmarks.items():
            first = next(o for o in self.problem.observations if o.tag == tag and o.accepted)
            self.assertTrue(first.first)
            expected = self.problem.keyframes[first.keyframe].ekf.transformFrom(first.base_point)
            np.testing.assert_allclose(initial.atPoint3(gtsam.symbol_shorthand.L(index)), expected, atol=1e-9)

    def test_orientation_is_off_by_default_and_needs_recorded_orientations(self):
        self.assertFalse(factors.load_config()["orientation"]["enabled"])
        self.assertEqual(self.problem.counts["orientation"], 0)
        self.assertEqual(self.batch.orientations, {})
        with self.assertRaisesRegex(factors.ConfigError, "positions only"):
            factors.build(self.stream, config(orientation__enabled=True))

    def test_far_detections_constrain_their_bearing_only(self):
        cfg = config()
        size = 0.12
        self.assertTrue(factors.range_used(cfg, 2.0, size))
        self.assertFalse(factors.range_used(cfg, 6.0, size))
        used = [o.range_used for o in self.problem.observations if o.accepted]
        self.assertTrue(any(used) and not all(used))

    def test_no_range_scale_is_estimated(self):
        keys = [gtsam.Symbol(key).chr() for key in self.problem.initial().keys()]
        self.assertEqual(set(map(chr, keys)), {"x", "l"})

    def test_batch_and_isam2_agree(self):
        worst_pose = max(float(np.linalg.norm(a.translation() - b.translation()))
                         for a, b in zip(self.batch.poses, self.isam2.poses))
        worst_landmark = max(float(np.linalg.norm(self.batch.landmarks[t] - self.isam2.landmarks[t]))
                             for t in self.batch.landmarks)
        self.assertLess(worst_pose, 0.01)
        self.assertLess(worst_landmark, 0.01)

    def test_drift_is_reduced_against_the_synthetic_truth(self):
        fit = dock.fit_dock(self.batch.landmarks, factors.load_layout(self.problem.config), self.problem.config)
        metrics = evaluate.scoring(self.problem, self.batch, truth.read_truth(self.path), fit)
        error = metrics["position_error_vs_truth"]
        self.assertGreater(error["ekf_m"]["max"], 0.2)                 # the synthetic EKF drifts
        self.assertLess(error["candidate_m"]["max"], 0.5 * error["ekf_m"]["max"])
        self.assertLess(error["candidate_m"]["median"], 0.5 * error["ekf_m"]["median"])

    def test_the_dock_point_is_fitted_once_the_flat_tags_fix_yaw(self):
        cfg = self.problem.config
        layout = factors.load_layout(cfg)
        fit = dock.fit_dock(self.batch.landmarks, layout, cfg)
        self.assertEqual((fit.status, fit.up), ("fitted", "z_up"))
        self.assertLess(float(np.linalg.norm(fit.pose.translation() - synthetic.DOCK.translation())), 0.05)
        centreline = {t: p for t, p in self.batch.landmarks.items() if t not in ("tag36h11:176", "tag36h11:185")}
        unobserved = dock.fit_dock(centreline, layout, cfg)
        self.assertEqual(unobserved.status, "yaw_unobserved")
        self.assertIsNone(unobserved.pose)
        self.assertIsNotNone(unobserved.pivot)


class GateTests(unittest.TestCase):
    def test_an_injected_outlier_is_gated_and_recorded(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = synthetic.write(Path(tmp) / "telemetry.jsonl", duration=40.0,
                                   outlier=(20.0, "tag36h11:146", [0.0, 0.0, 3.0]))
            problem = factors.build(sensors.read_telemetry(path), config())
        gated = [o for o in problem.observations if not o.accepted and o.reason and o.reason.startswith("gated")]
        self.assertEqual(problem.counts["gated"], 1)
        self.assertEqual([(o.tag, round(o.stamp - synthetic.START)) for o in gated], [("tag36h11:146", 20)])


class OrientationTests(unittest.TestCase):
    def test_the_opt_in_orientation_factor_is_range_gated_and_solves(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = synthetic.write(Path(tmp) / "telemetry.jsonl", orientations=True)
            cfg = config(orientation__enabled=True)
            problem = factors.build(sensors.read_telemetry(path), cfg)
        accepted = [o for o in problem.observations if o.accepted]
        within = sum(o.range_m <= cfg["orientation"]["max_range_m"] for o in accepted)
        self.assertEqual(problem.counts["orientation"] + problem.counts["orientation_gated"], within)
        self.assertGreater(problem.counts["orientation"], 0)
        solution = solvers.solve_batch(problem)
        layout = factors.load_layout(cfg)
        for tag, rotation in solution.orientations.items():
            expected = synthetic.DOCK.rotation().compose(layout[tag]["rotation"])
            with self.subTest(tag=tag):
                self.assertLess(float(np.linalg.norm(gtsam.Rot3.Logmap(expected.between(rotation)))), 0.05)


if __name__ == "__main__":
    unittest.main()
