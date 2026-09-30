#!/usr/bin/env python3
"""Does the race-auv-docking onboard recorder cost the AprilTag detector front-camera images? (#86, #92)

Offline comparison of trial artifact directories run with and without media.onboard_camera_video. The detector
republishes detections on a timer, so detection counts say nothing about image delivery; every detection row in
telemetry.jsonl carries its source image's stamp, so the number of distinct stamps per camera is the number of
images the detector processed. cam_down shares the simulator and the host but not the recorder, so the
cam_front / cam_down ratio within a run controls for load. A drop of that ratio in the recorder runs points at
the recorder's reliable subscription; the fallback is BEST_EFFORT in runtime/onboard_recorder.py.

Standard library only. Decision aid, not a scientific claim: one pair of runs is noisy, pass several of each.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import sys

SCHEMA = "piccard.race-auv.onboard-load-check/v1"
CAMERAS = ("cam_front", "cam_down")
RECORDER = "/piccard_media_recorder"
DEFAULT_THRESHOLD = 0.8


def trial_root(path: Path) -> Path:
    """A trial's output directory, or a downloaded artifact directory that holds it under output/."""
    if (path / "telemetry.jsonl").is_file():
        return path
    if (path / "output" / "telemetry.jsonl").is_file():
        return path / "output"
    raise FileNotFoundError(f"{path}: no telemetry.jsonl (or output/telemetry.jsonl)")


def load_json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def finite(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def detector_images(telemetry: Path, start, end) -> dict:
    """Distinct source-image stamps per camera among detection rows inside [start, end] (collector t)."""
    stamps = {camera: set() for camera in CAMERAS}
    with telemetry.open(encoding="utf-8") as stream:
        for line in stream:
            if '"detections"' not in line:
                continue
            row = json.loads(line)
            if row.get("kind") != "detections" or row.get("camera") not in stamps or not finite(row.get("stamp")):
                continue
            if finite(start) and finite(end) and not start <= row.get("t", start) <= end:
                continue
            stamps[row["camera"]].add(row["stamp"])
    return {camera: len(values) for camera, values in stamps.items()}


def hold_distance(docking: dict):
    poses = docking.get("poses") if isinstance(docking.get("poses"), list) else []
    if not poses:
        return None
    dwell = (poses[-1].get("dwell") or {}) if isinstance(poses[-1], dict) else {}
    return dwell.get("distance_mean_m")


def describe(path: Path) -> dict:
    root = trial_root(path)
    trial, runner = load_json(root / "trial.json"), load_json(root / "runner.json")
    media, recorder = load_json(root / "media.json"), load_json(root / "onboard-recorder.json")
    start, end = trial.get("mission_start_t"), trial.get("mission_end_t")
    images = detector_images(root / "telemetry.jsonl", start, end)
    window = end - start if finite(start) and finite(end) and end > start else None
    audits = {name: trial.get(name) or {} for name in ("consumption_audit_before_mission", "consumption_audit_at_end")}
    cam_front_clip = next((clip for clip in media.get("clips", []) if clip.get("name") == "cam_front"), {})
    return {
        "path": str(path), "status": trial.get("status"), "stop_reason": trial.get("stop_reason"),
        "mission_sha256": trial.get("mission_sha256"), "expected_gains": trial.get("expected_gains"),
        "onboard_requested": (runner.get("media_requested") or {}).get("onboard_camera_video"),
        "recorder_in_audits": all(RECORDER in ((audit.get("media_nodes") or {}).get("nodes") or {})
                                  for audit in audits.values()),
        "recorder_absent_from_audits": all(RECORDER not in ((audit.get("media_nodes") or {}).get("nodes") or {})
                                           for audit in audits.values()),
        "audit_problems": sorted({problem for audit in audits.values() for problem in audit.get("problems") or []}),
        "mission_window_s": window,
        "detector_images": images,
        "detector_image_rate_hz": {camera: round(count / window, 4) if window else None
                                   for camera, count in images.items()},
        "front_down_ratio": round(images["cam_front"] / images["cam_down"], 4) if images["cam_down"] else None,
        "recorder": {"camera_rate_hz_measured": (cam_front_clip.get("synchronization") or {}).get("camera_rate_hz_measured"),
                     "frames": recorder.get("frames"), "qos": recorder.get("qos")} if recorder or cam_front_clip else None,
        "hold_distance_mean_m": hold_distance(load_json(root / "docking.json")),
    }


def compare(with_runs: list[dict], without_runs: list[dict], threshold: float = DEFAULT_THRESHOLD) -> dict:
    reasons = []
    if any(not run["onboard_requested"] or not run["recorder_in_audits"] for run in with_runs):
        reasons.append("a --with-onboard run did not request onboard or lacks the recorder in both audits")
    if any(run["onboard_requested"] or not run["recorder_absent_from_audits"] for run in without_runs):
        reasons.append("a --without-onboard run requested onboard or has the recorder in an audit")
    runs = with_runs + without_runs
    if len({run["mission_sha256"] for run in runs}) != 1:
        reasons.append("missions differ")
    if len({json.dumps(run["expected_gains"], sort_keys=True) for run in runs}) != 1:
        reasons.append("gains differ")
    if any(run["audit_problems"] for run in runs):
        reasons.append("an audit reported problems")

    def mean_ratio(group):
        values = [run["front_down_ratio"] for run in group if run["front_down_ratio"] is not None]
        return round(statistics.fmean(values), 4) if values else None

    ratio_with, ratio_without = mean_ratio(with_runs), mean_ratio(without_runs)
    relative = round(ratio_with / ratio_without, 4) if ratio_with is not None and ratio_without else None
    if relative is None:
        verdict = "undetermined"
        reasons.append("a group has no cam_down detector images")
    elif relative < threshold:
        verdict = "drop"
    else:
        verdict = "no_drop"
    return {
        "schema": SCHEMA,
        "method": "distinct source-image stamps per camera in the collector's detection rows inside the mission "
                  "window; cam_front / cam_down within a run; mean over runs per group",
        "threshold": threshold,
        "front_down_ratio_with_onboard": ratio_with, "front_down_ratio_without_onboard": ratio_without,
        "relative": relative, "verdict": verdict, "comparable": not reasons, "reasons": reasons,
        "recommendation": {"drop": "switch the recorder to BEST_EFFORT (runtime/onboard_recorder.py) or keep "
                                   "onboard_camera_video out of scored trials",
                           "no_drop": "no evidence that the reliable recorder costs the detector cam_front images",
                           "undetermined": "rerun with detections from both cameras"}[verdict],
        "claim_boundary": "decision aid for recorder QoS; few runs are noisy and this is not a perception metric",
        "runs": {"with_onboard": with_runs, "without_onboard": without_runs},
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--with-onboard", type=Path, action="append", required=True, metavar="DIR")
    parser.add_argument("--without-onboard", type=Path, action="append", required=True, metavar="DIR")
    parser.add_argument("--threshold", type=float, default=DEFAULT_THRESHOLD,
                        help="flag a drop when the with-onboard ratio is below this fraction of the without ratio")
    parser.add_argument("--output", type=Path, help="write the report here instead of stdout")
    args = parser.parse_args(argv)
    try:
        report = compare([describe(path) for path in args.with_onboard],
                         [describe(path) for path in args.without_onboard], args.threshold)
    except (FileNotFoundError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
