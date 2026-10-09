"""The replay command's outputs: the CSVs, the figures and the results JSON, every metric in it carrying a
result_class (sensor-grounded for the consistency metrics, scoring for the truth-based ones)."""
import csv
import json
import tempfile
import unittest
from pathlib import Path

import synthetic
from race_tag_graph import evaluate, factors, metrics, writers
from race_tag_graph.__main__ import main

SENSOR_GROUNDED = {"resighting_residuals", "drift_at_resightings", "dock_point_stability", "range_scale_residual"}
SCORING = {"position_error_vs_truth", "error_at_resightings_vs_truth", "dock_point_error_vs_truth"}


class ReplayOutputTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        root = Path(cls.tmp.name)
        cls.telemetry = synthetic.write(root / "telemetry.jsonl")
        cls.evaluated, cls.plain = root / "evaluated", root / "plain"
        main(["replay", "--telemetry", str(cls.telemetry), "--out", str(cls.evaluated), "--solver", "both",
              "--evaluate-against-truth"])
        main(["replay", "--telemetry", str(cls.telemetry), "--out", str(cls.plain), "--solver", "isam2"])
        cls.document = json.loads((cls.evaluated / "results.json").read_text())

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_every_output_is_written(self):
        for name in ("trajectory.csv", "landmarks.csv", "dock_point.csv", "results.json", "trajectory.png"):
            with self.subTest(name=name):
                self.assertGreater((self.evaluated / name).stat().st_size, 0)

    def test_every_metric_has_a_valid_result_class(self):
        found = {name: metric["result_class"] for name, metric in self.document["metrics"].items()}
        self.assertEqual(found, {**{n: metrics.SENSOR_GROUNDED for n in SENSOR_GROUNDED},
                                 **{n: metrics.SCORING for n in SCORING}})
        with self.assertRaises(ValueError):
            writers.check_result_classes({"unlabelled": {"value": 1.0}})
        with self.assertRaises(ValueError):
            writers.check_result_classes({"oracle": {"result_class": "oracle"}})

    def test_without_truth_only_the_consistency_metrics_are_reported(self):
        plain = json.loads((self.plain / "results.json").read_text())
        self.assertEqual(set(plain["metrics"]), SENSOR_GROUNDED)
        self.assertEqual([s["solver"] for s in plain["solvers"]], ["isam2"])

    def test_the_dock_point_is_written_in_base_link(self):
        rows = list(csv.DictReader((self.evaluated / "dock_point.csv").open()))
        self.assertEqual(self.document["dock"]["status"], "fitted")
        self.assertEqual(len(rows), self.document["counts"]["keyframes"])
        self.assertEqual({(r["frame_id"], r["child_frame_id"]) for r in rows},
                         {("race_auv/base_link", "race_station/dock_point")})

    def test_the_development_trial_is_marked_and_kept_out_of_the_headline_set(self):
        config = factors.load_config()
        self.assertEqual(config["evaluation"]["development_trials"], ["pr-m3a-p10-contact-r1"])
        self.assertEqual(evaluate.headline_trials(["pr-m3a-p10-contact-r1", "pr-m3a-p10-planner-r1",
                                                   "drift-and-revisit-r1"], config),
                         ["pr-m3a-p10-planner-r1", "drift-and-revisit-r1"])
        out = Path(self.tmp.name) / "development"
        main(["replay", "--telemetry", str(self.telemetry), "--out", str(out), "--trial", "pr-m3a-p10-contact-r1"])
        inputs = json.loads((out / "results.json").read_text())["inputs"]
        self.assertEqual((inputs["trial"], inputs["development_trial"], inputs["headline_eligible"]),
                         ("pr-m3a-p10-contact-r1", True, False))
        self.assertEqual((self.document["inputs"]["development_trial"], self.document["inputs"]["headline_eligible"]),
                         (False, True))

    def test_an_opaque_directory_id_is_not_recorded_as_the_trial(self):
        root = Path(self.tmp.name) / "0123456789abcdef0123456789abcdef" / "output"
        root.mkdir(parents=True)
        (root / "telemetry.jsonl").write_text(self.telemetry.read_text())
        out = Path(self.tmp.name) / "opaque"
        main(["replay", "--telemetry", str(root / "telemetry.jsonl"), "--out", str(out)])
        self.assertIsNone(json.loads((out / "results.json").read_text())["inputs"]["trial"])

    def test_both_solvers_are_in_the_trajectory_and_the_results(self):
        header = (self.evaluated / "trajectory.csv").read_text().splitlines()[0].split(",")
        self.assertIn("batch_x", header)
        self.assertIn("isam2_x", header)
        self.assertEqual([s["solver"] for s in self.document["solvers"]], ["batch", "isam2"])
        self.assertEqual(self.document["inputs"]["solver_for_metrics"], "batch")


if __name__ == "__main__":
    unittest.main()
