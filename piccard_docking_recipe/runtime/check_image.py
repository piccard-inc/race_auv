#!/usr/bin/env python3
"""Build-time check against the real pinned files: the recipe's configured limits equal config_sim.yaml, the
M1 smoke candidate (both tag-size variants) installs and verifies on a scratch copy of the workspace files, and the
CPU AprilTag binding solves a pose on a pinned tag texture (an identity-pose fallback would fail it)."""
from __future__ import annotations

import json
from pathlib import Path
import shutil
import sys
import tempfile

import yaml

import mission as mission_module
import prepare_candidate

PACKAGE = Path(__file__).resolve().parent.parent


def check_tag_pose(workspace: Path) -> None:
    import cv2
    from apriltag import apriltag
    texture = workspace / "install/share/world_of_stonefish/data/apriltags/tag36_11_00146_1000px.png"
    image = cv2.imread(str(texture), cv2.IMREAD_GRAYSCALE)
    detector = apriltag("tag36h11")
    detections = [d for d in detector.detect(image) if d["id"] == 146]
    if not detections:
        raise SystemExit(f"tag 146 not detected in {texture}")
    # black square is 800 px wide: a 0.12 m edge seen with f = 1000 px sits at 1000 * 0.12 / 800 = 0.15 m
    pose = detector.estimate_tag_pose(detections[0], 0.12, 1000.0, 1000.0, 500.0, 500.0)
    depth = float(pose["t"][2][0] if hasattr(pose["t"][2], "__len__") else pose["t"][2])
    if abs(depth - 0.15) > 0.005:
        raise SystemExit(f"apriltag pose solve returned depth {depth}, expected 0.15")


def main(workspace: Path) -> int:
    recipe = json.loads((PACKAGE / "recipe-v1.json").read_text())
    request = json.loads((PACKAGE / "examples/m1-smoke-request.json").read_text())
    files = prepare_candidate.paths(workspace)
    limits = prepare_candidate.configured_limits(yaml.safe_load(files["control"][-1].read_text()))
    recorded = {axis: tuple(value) for axis, value in recipe["fixed_execution"]["configured_output_limits"].items()}
    if limits != recorded:
        raise SystemExit(f"config_sim.yaml flight limits {limits} differ from recipe-v1.json {recorded}")
    for variant in mission_module.TAG_SIZE_CONVENTIONS:
        with tempfile.TemporaryDirectory() as tmp:
            scratch = Path(tmp) / "ws"
            for source, target in zip(sum((files[key] for key in ("control", "helm", "bhv")), []) + [files["apriltag"]],
                                      sum((prepare_candidate.paths(scratch)[key] for key in ("control", "helm", "bhv")), [])
                                      + [prepare_candidate.paths(scratch)["apriltag"]]):
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source, target)
            mission = mission_module.validate_mission({**request["mission"], "apriltag_tag_size": variant})
            prepare_candidate.install(request["gains"], mission, scratch, Path(tmp) / "out")
            prepare_candidate.verify(request["gains"], mission, scratch, Path(tmp) / "out")
    check_tag_pose(workspace)
    print("race-auv-docking image check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(Path(sys.argv[1] if len(sys.argv) > 1 else "/opt/race_ws")))
