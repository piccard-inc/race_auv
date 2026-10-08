"""The collector's perception gate: a planner mission starts only with both AprilTag detectors publishing (after
verify-planner-1, whose detectors died at startup). No ROS."""
import inspect
import math
import unittest

import collect_trial

PUBLISHING = {"cam_front": 1, "cam_down": 1}


class PerceptionGateTests(unittest.TestCase):
    def test_both_detectors_publishing(self):
        self.assertEqual(collect_trial.perception_missing({"cam_front": 9.9, "cam_down": 9.8}, PUBLISHING, 10.0), [])

    def test_a_detector_not_running(self):
        cases = {
            "neither ever published (verify-planner-1)": ({}, {"cam_front": 0, "cam_down": 0}, ["cam_front", "cam_down"]),
            "no array yet, publishers up": ({}, PUBLISHING, ["cam_front", "cam_down"]),
            "one camera silent": ({"cam_front": 9.9}, PUBLISHING, ["cam_down"]),
            "an array older than STALE_S": ({"cam_front": 9.9, "cam_down": 10.0 - collect_trial.STALE_S - 0.1},
                                            PUBLISHING, ["cam_down"]),
            "a recent array but no publisher left": ({"cam_front": 9.9, "cam_down": 9.9},
                                                     {"cam_front": 1, "cam_down": 0}, ["cam_down"]),
            "publishers unknown": ({"cam_front": 9.9, "cam_down": 9.9}, {}, ["cam_front", "cam_down"])}
        for label, (last_t, publishers, missing) in cases.items():
            with self.subTest(label):
                self.assertEqual(collect_trial.perception_missing(last_t, publishers, 10.0), missing)

    def test_an_array_exactly_stale_s_old_still_counts(self):
        self.assertEqual(collect_trial.perception_missing(
            {"cam_front": 10.0 - collect_trial.STALE_S, "cam_down": 10.0}, PUBLISHING, 10.0), [])

    def test_the_gate_covers_every_detection_topic(self):
        self.assertEqual(sorted(collect_trial.perception_missing({}, {}, math.inf)),
                         sorted(collect_trial.DETECTIONS.values()))
        self.assertEqual(collect_trial.PERCEPTION_GATE_S, 30.0)

    def test_a_planner_mission_is_gated_after_readiness_and_before_anything_moves(self):
        source = inspect.getsource(collect_trial.ros_node_class)
        run = source[source.index("        def run(self):"):source.index("        def perception_gate(self):")]
        order = [run.index(step) for step in ("while not self.ready():", "if self.planner_mission:\n"
                                              "                self.perception_gate()", "self.discover_thrusters()",
                                              "self.enable(True)", 'event="mission_start"')]
        self.assertEqual(order, sorted(order))
        gate = source[source.index("        def perception_gate(self):"):source.index("        def fly(self, pose):")]
        self.assertIn('raise RuntimeError("perception_not_running:" + ",".join(missing))', gate)
        self.assertIn("PERCEPTION_GATE_S", gate)


if __name__ == "__main__":
    unittest.main()
