from __future__ import annotations

import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("race_onboard_load_check", ROOT / "race_onboard_load_check.py")
check = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(check)

RECORDER = "/piccard_media_recorder"


def make_run(root: Path, front: int, down: int, onboard: bool, *, recorder_in_audit=None, mission="m1",
             nested=False) -> Path:
    """A trial output dir: mission window 10..110 s; each camera's detector republishes every image stamp 3 times."""
    out = root / "output" if nested else root
    out.mkdir(parents=True)
    rows = [{"kind": "event", "t": 0.1, "wall_time_ns": 1}]
    for camera, images in (("cam_front", front), ("cam_down", down)):
        for i in range(images):
            for repeat in range(3):
                rows.append({"kind": "detections", "t": 10 + i * 0.5 + repeat * 0.1, "camera": camera,
                             "stamp": 1790000000.0 + i})
        rows.append({"kind": "detections", "t": 5.0, "camera": camera, "stamp": 1.0})  # before the mission
    rows.append({"kind": "fused_dock", "t": 20.0, "stamp": 42.0})
    (out / "telemetry.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    in_audit = onboard if recorder_in_audit is None else recorder_in_audit
    nodes = {RECORDER: ["/race_auv/cam_front/stonefish/data/image_color"]} if in_audit else {}
    audit = {"problems": [], "media_nodes": {"allowed_topics": [], "nodes": nodes}}
    (out / "trial.json").write_text(json.dumps({
        "status": "completed", "stop_reason": "mission_complete", "mission_sha256": mission,
        "expected_gains": {"surge": {"p": 1}}, "mission_start_t": 10.0, "mission_end_t": 110.0,
        "consumption_audit_before_mission": audit, "consumption_audit_at_end": audit}))
    (out / "runner.json").write_text(json.dumps({"media_requested": {"display_video": False,
                                                                     "onboard_camera_video": onboard}}))
    (out / "docking.json").write_text(json.dumps({"poses": [{"label": "dive"},
                                                            {"label": "hold", "dwell": {"distance_mean_m": 1.6}}]}))
    if onboard:
        (out / "media.json").write_text(json.dumps({"clips": [
            {"name": "cam_front", "synchronization": {"camera_rate_hz_measured": 0.73}}]}))
        (out / "onboard-recorder.json").write_text(json.dumps({"qos": "reliable, keep_last 2",
                                                               "frames": {"received": 220, "written": 220}}))
    return root


class LoadCheckTests(unittest.TestCase):
    def test_distinct_stamps_inside_the_mission_window(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = check.describe(make_run(Path(tmp) / "a", front=30, down=40, onboard=True))
        self.assertEqual(run["detector_images"], {"cam_front": 30, "cam_down": 40})  # repeats and t=5 rows excluded
        self.assertEqual(run["front_down_ratio"], 0.75)
        self.assertEqual(run["detector_image_rate_hz"], {"cam_front": 0.3, "cam_down": 0.4})
        self.assertEqual(run["recorder"]["camera_rate_hz_measured"], 0.73)
        self.assertEqual(run["hold_distance_mean_m"], 1.6)
        self.assertTrue(run["recorder_in_audits"])

    def test_a_ratio_drop_with_the_recorder_is_flagged(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            with_runs = [check.describe(make_run(root / "w1", 20, 40, True)),
                         check.describe(make_run(root / "w2", 24, 40, True))]
            without_runs = [check.describe(make_run(root / "p1", 40, 40, False))]
        report = check.compare(with_runs, without_runs)
        self.assertEqual((report["front_down_ratio_with_onboard"], report["front_down_ratio_without_onboard"]),
                         (0.55, 1.0))
        self.assertEqual((report["relative"], report["verdict"], report["comparable"]), (0.55, "drop", True))
        self.assertIn("BEST_EFFORT", report["recommendation"])

    def test_no_drop_within_the_threshold(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            report = check.compare([check.describe(make_run(root / "w", 36, 40, True))],
                                   [check.describe(make_run(root / "p", 40, 40, False))])
        self.assertEqual((report["relative"], report["verdict"]), (0.9, "no_drop"))

    def test_incomparable_runs_are_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            report = check.compare(
                [check.describe(make_run(root / "w", 30, 40, True, recorder_in_audit=False))],
                [check.describe(make_run(root / "p", 40, 40, False, recorder_in_audit=True, mission="m2"))])
        self.assertFalse(report["comparable"])
        self.assertEqual(len(report["reasons"]), 3)  # recorder missing, recorder present, missions differ

    def test_no_cam_down_images_is_undetermined(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            report = check.compare([check.describe(make_run(root / "w", 30, 0, True))],
                                   [check.describe(make_run(root / "p", 40, 40, False))])
        self.assertEqual(report["verdict"], "undetermined")

    def test_cli_accepts_downloaded_artifact_dirs_and_writes_the_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            make_run(root / "with", 20, 40, True, nested=True)  # <job>/output/... as downloaded
            make_run(root / "without", 40, 40, False)
            report_path = root / "report.json"
            code = check.main(["--with-onboard", str(root / "with"), "--without-onboard", str(root / "without"),
                               "--output", str(report_path)])
            self.assertEqual(code, 0)
            self.assertEqual(json.loads(report_path.read_text())["verdict"], "drop")
            with mock.patch("sys.stderr", new_callable=io.StringIO) as stderr:
                code = check.main(["--with-onboard", str(root / "missing"), "--without-onboard", str(root / "without")])
            self.assertEqual(code, 2)
            self.assertIn("no telemetry.jsonl", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
