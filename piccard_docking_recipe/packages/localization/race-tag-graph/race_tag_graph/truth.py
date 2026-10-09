"""Evaluation only: the records the estimator may never read (ground truth and the lab fuser's comparison output).

Nothing in the estimator imports this module (a test checks it). It reads, from a collector telemetry.jsonl:
- gt_auv, /race_auv/stonefish/odometry: the vehicle in Stonefish's world_ned; its origin is the nose tip and its
  quaternion the FLU body (base_link's axes) in world_ned (verified in the #265 drift audit, D0);
- dock: the collector's ground-truth dock relation, incl. the station dock point in the AUV's base_link;
- fused_dock, /race_station/dock_point/pose: the lab fuser's recorded dock point in base_link (comparison input);
- tf_static: for the base_link -> nose tip offset.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import gtsam
import numpy as np

from .geometry import PoseSeries, pose3, static_chain

NED_TO_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
TRUTH_KINDS = {"gt_auv", "dock", "fused_dock", "tf_static"}


@dataclass
class Truth:
    base_link: PoseSeries        # the true base_link pose, axes mapped to ENU (world_ned's x/y swapped, z negated)
    dock_in_base: list           # [(stamp, position)] the true station dock point in base_link
    lab_fuser: list              # [(stamp, position)] the lab fuser's dock point in base_link


def read_truth(path) -> Truth:
    gt, dock_rows, fused, edges, clock = [], [], [], [], []
    with Path(path).open() as stream:
        for line in stream:
            if '"kind"' not in line[:40]:
                continue
            record = json.loads(line)
            kind = record.get("kind")
            if kind not in TRUTH_KINDS:
                continue
            if kind == "gt_auv" and record.get("stamp") is not None:
                gt.append(record)
                clock.append((record["t"], record["stamp"]))
            elif kind == "dock" and record.get("station_dock_in_auv_base_link_m") is not None:
                dock_rows.append((record["t"], record["station_dock_in_auv_base_link_m"]))
            elif kind == "fused_dock" and record.get("stamp") is not None:
                fused.append((float(record["stamp"]), np.asarray(record["position_m"], dtype=float)))
            elif kind == "tf_static":
                edges.extend(record.get("transforms") or [])
    nose = static_chain(edges, "race_auv/base_link", "race_auv/nose_tip_link")
    nose_offset = np.zeros(3) if nose is None else np.asarray(nose.translation())
    stamps, poses = [], []
    for record in gt:
        world_ned = pose3((record["x"], record["y"], record["z"]),
                          (record["qx"], record["qy"], record["qz"], record["qw"]))
        rotation = NED_TO_ENU @ world_ned.rotation().matrix()
        base = NED_TO_ENU @ (np.asarray(world_ned.translation()) - world_ned.rotation().matrix() @ nose_offset)
        stamps.append(float(record["stamp"]))
        poses.append(gtsam.Pose3(gtsam.Rot3(rotation), base))
    clock.sort()
    times, clock_stamps = [c[0] for c in clock], [c[1] for c in clock]
    dock = [(float(np.interp(t, times, clock_stamps)), np.asarray(p, dtype=float)) for t, p in dock_rows
            if times and times[0] <= t <= times[-1]]
    return Truth(base_link=PoseSeries(stamps, poses), dock_in_base=dock, lab_fuser=sorted(fused, key=lambda r: r[0]))
