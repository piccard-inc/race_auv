"""A synthetic telemetry.jsonl in the Piccard collector's format, for the offline tests: no recorded trial data.

The vehicle (odom = an ENU world here) dives and approaches the station, turns away so the tags leave view, backs
off, turns back for a revisit, then hovers over the station's flat tags. The EKF is the true trajectory plus a
constant position drift; the cameras (the vehicle URDF's extrinsics, as /tf_static records them) see the station's
tags (config/station_layout.yaml) with a small bearing and range noise. Truth is written the way Stonefish records
it: the nose tip in world_ned with the FLU body's quaternion.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import gtsam
import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parent.parent
NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])   # ENU <-> NED, its own inverse
TF_STATIC = [
    {"parent": "race_auv/base_link", "child": "race_auv/nose_tip_link", "xyz": [0.7, 0.0, 0.0], "q": [0, 0, 0, 1]},
    {"parent": "race_auv/nose_tip_link", "child": "race_auv/cam_front", "xyz": [-0.004, 0.0, 0.035],
     "q": [-0.5, 0.5, -0.5, 0.5]},
    {"parent": "race_auv/nose_tip_link", "child": "race_auv/cam_down", "xyz": [-0.305, 0.0, -0.128],
     "q": [0.7071068, -0.7071068, 0.0, 0.0]},
]
DOCK = gtsam.Pose3(gtsam.Rot3.Rz(0.05), np.array([6.0, 0.2, -3.9]))   # the station's dock point, z up
DRIFT_MPS = np.array([0.002, -0.0015, 0.0])
START = 1000.0


def _q(rotation: gtsam.Rot3) -> list:
    q = rotation.toQuaternion()
    return [q.x(), q.y(), q.z(), q.w()]


def _pose(t: float) -> gtsam.Pose3:
    """The true base_link pose at t (seconds from the start), piecewise in time."""
    def lerp(a, b, f):
        return a + (b - a) * min(max(f, 0.0), 1.0)
    if t < 40:      # dive and approach to about 3 m from the tags, facing them
        x, z, yaw = lerp(0.0, 2.8, t / 40), lerp(0.0, -3.7, t / 20), 0.0
    elif t < 55:    # turn away: the tags leave view
        x, z, yaw = 2.8, -3.7, lerp(0.0, math.pi, (t - 40) / 15)
    elif t < 85:    # back off, tags out of view
        x, z, yaw = lerp(2.8, 1.0, (t - 55) / 30), -3.7, math.pi
    elif t < 100:   # turn back: the revisit
        x, z, yaw = 1.0, -3.7, lerp(math.pi, 2 * math.pi, (t - 85) / 15)
    else:           # approach and hover over the flat tags
        x, z, yaw = lerp(1.0, 5.6, (t - 100) / 40), lerp(-3.7, -3.3, (t - 100) / 40), 2 * math.pi
    return gtsam.Pose3(gtsam.Rot3.Rz(yaw), np.array([x, 0.15, z]))


def write(path: Path, duration: float = 150.0, range_scale: float = 1.0, outlier=None, orientations=False,
          seed: int = 7) -> Path:
    """outlier: (time, tag, displacement in the camera frame) applied to that tag's detection nearest the time."""
    rng = np.random.default_rng(seed)
    layout = yaml.safe_load((PACKAGE / "config" / "station_layout.yaml").read_text())["tags"]
    tags = {tag: (DOCK.transformFrom(np.asarray(item["position_m"], float)),
                  DOCK.rotation().compose(gtsam.Rot3.RzRyRx(*item["rpy_rad"]))) for tag, item in layout.items()}
    edges = {e["child"]: e for e in TF_STATIC}
    nose = gtsam.Pose3(gtsam.Rot3(), np.array(edges["race_auv/nose_tip_link"]["xyz"], float))
    cameras = {}
    for name in ("cam_front", "cam_down"):
        e = edges[f"race_auv/{name}"]
        x, y, z, w = e["q"]
        cameras[name] = nose.compose(gtsam.Pose3(gtsam.Rot3.Quaternion(w, x, y, z), np.array(e["xyz"], float)))
    rows = [{"kind": "tf_static", "t": 0.4, "topic": "/tf_static", "stamp": None, "transforms": TF_STATIC},
            {"kind": "gt_tf_static", "t": 0.41, "topic": "/piccard/ground_truth/tf_static", "stamp": None,
             "transforms": []}]
    outlier_done = False
    steps = int(duration * 20)
    for i in range(steps + 1):
        t = i / 20
        stamp, pose = START + t, _pose(t)
        ekf = gtsam.Pose3(pose.rotation(), np.asarray(pose.translation()) + DRIFT_MPS * t +
                          rng.normal(0, 0.0005, 3))
        rows.append({"kind": "odometry", "t": t + 0.5, "topic": "/race_auv/odometry/filtered", "stamp": stamp,
                     "frame_id": "race_auv/odom", "child_frame_id": "race_auv/base_link",
                     **dict(zip("xyz", np.asarray(ekf.translation()).tolist())),
                     **dict(zip(("qx", "qy", "qz", "qw"), _q(ekf.rotation())))})
        if i % 2 == 0:      # truth at 10 Hz: the nose tip in world_ned, the FLU body's quaternion in world_ned
            tip = NED @ np.asarray(pose.compose(nose).translation())
            rows.append({"kind": "gt_auv", "t": t + 0.51, "topic": "/race_auv/stonefish/odometry", "stamp": stamp,
                         "frame_id": "world_ned", "child_frame_id": "race_auv/Odometry",
                         **dict(zip("xyz", tip.tolist())),
                         **dict(zip(("qx", "qy", "qz", "qw"), _q(gtsam.Rot3(NED @ pose.rotation().matrix()))))})
            rows.append({"kind": "dock", "t": t + 0.52, "topic": "/race_auv/stonefish/odometry",
                         "station_dock_in_auv_base_link_m": np.asarray(pose.transformTo(DOCK.translation())).tolist()})
            rows.append({"kind": "alignment", "t": t + 0.53, "topic": None, "controller_cg": {}, "ground_truth_cg": {}})
        if i % 5 == 0:      # images at 4 Hz per camera
            for offset, (name, extrinsic) in enumerate(cameras.items()):
                camera = pose.compose(extrinsic)
                ids, positions, quats = [], [], []
                for tag, (position, rotation) in tags.items():
                    seen = np.asarray(camera.transformTo(position))
                    r = float(np.linalg.norm(seen))
                    if seen[2] < 0.3 or abs(seen[0] / seen[2]) > 0.9 or abs(seen[1] / seen[2]) > 0.7 or r > 9.0:
                        continue
                    ray = seen / r
                    noisy = seen * range_scale + rng.normal(0, 0.001, 3) + ray * rng.normal(0, 0.002 * r)
                    if outlier and not outlier_done and tag == outlier[1] and t >= outlier[0]:
                        noisy, outlier_done = noisy + np.asarray(outlier[2], float), True
                    ids.append(tag)
                    positions.append(noisy.tolist())
                    quats.append(_q(camera.rotation().inverse().compose(rotation)))
                record = {"kind": "detections", "t": t + 0.55 + 0.01 * offset,
                          "topic": f"/{name}/apriltag_detection/detections3d", "stamp": stamp + 0.01 * offset,
                          "frame_id": f"race_auv/{name}", "camera": name, "count": len(ids), "ids": ids,
                          "positions_m": positions}
                if orientations:
                    record["orientations_q"] = quats
                rows.append(record)
                if ids:
                    relative = np.asarray(pose.transformTo(DOCK.translation())) + rng.normal(0, 0.02, 3)
                    rows.append({"kind": "fused_dock", "t": t + 0.56, "topic": "/race_station/dock_point/pose",
                                 "stamp": stamp, "frame_id": "race_auv/base_link",
                                 "position_m": relative.tolist(), "orientation_q": [0, 0, 0, 1]})
    rows.sort(key=lambda r: r["t"])
    Path(path).write_text("".join(json.dumps(r, separators=(",", ":")) + "\n" for r in rows))
    return Path(path)
