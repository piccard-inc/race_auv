#!/usr/bin/env python3
"""Write shutdown outcome after process groups have received bounded stop signals."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time


def process_group_members(pgid):
    members = []
    for stat_path in Path("/proc").glob("[0-9]*/stat"):
        try:
            raw = stat_path.read_text(encoding="utf-8")
            close = raw.rfind(")")
            fields = raw[close + 2:].split()
            if int(fields[2]) == pgid:
                members.append({
                    "pid": int(stat_path.parent.name),
                    "state": fields[0],
                    "zombie": fields[0] == "Z",
                })
        except (OSError, ValueError, IndexError):
            continue
    return sorted(members, key=lambda member: member["pid"])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--exit-code", type=int, required=True)
    parser.add_argument("--launch-pid", type=int, required=True)
    parser.add_argument("--collector-pid", type=int, default=0)
    parser.add_argument("--video-pid", type=int, default=0)
    parser.add_argument("--xvfb-pid", type=int, default=0)
    parser.add_argument("--view-pid", type=int, default=0)
    parser.add_argument("--onboard-pid", type=int, default=0)
    parser.add_argument("--video-requested", type=int, choices=(0, 1), required=True)
    parser.add_argument("--onboard-requested", type=int, choices=(0, 1), default=0)
    args = parser.parse_args()
    root = Path(args.output)
    groups = {}
    for label in ("launch", "collector", "video", "view", "onboard", "xvfb"):
        pid = getattr(args, label + "_pid")
        alive = False
        if pid > 0:
            try:
                os.killpg(pid, 0)
                alive = True
            except ProcessLookupError:
                pass
        groups[label] = {
            "pid": pid,
            "alive": alive,
            "members": process_group_members(pid) if pid > 0 else [],
        }
    cleanup_verified = not any(group["alive"] for group in groups.values())
    metadata_present = (root / "trial.json").is_file()
    if not metadata_present:
        with (root / "trial.json").open("x", encoding="utf-8") as stream:
            json.dump({"schema": "piccard.race-auv.trial/v1", "status": "failed",
                       "stop_reason": "collector_did_not_finalize", "gains_verified": False,
                       "mission_start_t": None, "mission_end_t": None,
                       "comparison_context": {}}, stream, indent=2)
    inventory = []
    for name in ("telemetry.jsonl", "trial.json", "docking.json", "candidate.json",
                 "candidate-install-verification.json", "apriltag-black-square-edge.yaml", "launch.log",
                 "collector.log", "display.mp4", "display-poster.png", "media.json", "follow-view.json",
                 "cam_front.mp4", "cam_front-poster.png", "cam_front-frames.csv", "onboard-recorder.json"):
        path = root / name
        if not path.is_file():
            continue
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            while chunk := stream.read(1024 * 1024):
                digest.update(chunk)
        inventory.append({"path": name, "bytes": path.stat().st_size, "sha256": digest.hexdigest()})
    result = {"schema": "piccard.race-auv.runner/v1", "exit_code": args.exit_code,
              "finished_wall_time_ns": time.time_ns(), "process_groups": groups,
              "process_cleanup_verified": cleanup_verified, "video_requested": bool(args.video_requested),
              "media_requested": {"display_video": bool(args.video_requested),
                                  "onboard_camera_video": bool(args.onboard_requested)},
              "collector_finalized": metadata_present, "inventory": inventory}
    with (root / "runner.json").open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    return 0 if cleanup_verified else 74


if __name__ == "__main__":
    raise SystemExit(main())
