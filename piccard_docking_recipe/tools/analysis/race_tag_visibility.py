"""Which station tags each RACE camera can see from a pose, offline (piccard-physical-ai #265, D2).

Used to design and check the drift-and-revisit mission (examples/drift-and-revisit-request.json): its out-of-view
legs must put no station tag inside either camera's field of view, and its returns must put the forward camera's
tags in view at a range the detector reaches. Standard library only; no simulator.

The geometry is the pinned simulation's, as the trials record it:
- Cameras: world_of_stonefish d51d59e vehicles/race_auv.scn, cam_front and cam_down, each 1600 x 1200 with an 82 deg
  horizontal field of view (so 66.2 deg vertical). Their extrinsics are /tf_static's race_auv/base_link ->
  nose_tip_link -> cam_front|cam_down (optical frames: z forward, x right, y down), as every trial's telemetry.jsonl
  records them.
- Station: its ground-truth pose in Stonefish world_ned (4.0, 0.0, 3.95, identity, from /race_station/stonefish/
  odometry) and the ground-truth tf_static race_station/base_link -> dock_point -> apriltag* (the station URDF).
- The vehicle: a pose mission's cg_link pose in the controller's world_ned maps to Stonefish world_ned by
  runtime/mission.py's nominal transform (x_C = -y_W, y_C = x_W + 3.8, yaw_C = yaw_W + 1.571), drift-free. cg_link
  is at base_link's origin; base_link is FLU, so its rotation in world_ned is Rz(yaw_W) Ry(pitch) Rx(roll) Rx(pi).

Two questions, answered separately:
- in_fov: the tag's centre lies inside the camera's field of view widened by FOV_MARGIN_DEG on every side, at any
  range and facing. A pose is out of view when no tag is in_fov on either camera; this ignores range and facing, so
  it can only overstate what a camera sees.
- detectable: in the unwidened field of view, inside the tag's DETECTION_RANGE_M band, and facing the camera within
  FACING_MAX_DEG. Over the 59 local RACE trials (#265 D0) only the forward camera ever detected a tag, and only
  tag36h11 146, 541 and 558; no other tag and nothing on cam_down is ever predicted detectable. Each band is where
  the detector reliably found the tag: over five trials (m3-0-rerun, m3-0-v1.5, pr-ho-n-depth-r1,
  pr-ho-p10-lateral-r1, verify-planner-2), sampled every 0.5 s from ground truth, the bins of true camera range in
  which at least 0.94 of the in-view samples had a detection within 0.5 s. Outside them the rate falls: 146 below
  0.5 m (0.09), 541 from 2.75 m (0.10 to 0.11), 558 below 0.5 m (0.36) and from 3.0 m (0.75, then 0.13). The longest
  detections seen were 7.85 m (146) and 3.52 m (541, 558). A detected tag's -z axis pointed within 39 deg of the
  camera in every trial; FACING_MAX_DEG allows 60.

Checked against the same five trials: no detection came from a tag outside the widened field of view, and no tag
was detected while the pose was out of view (m3-0-rerun: 179 s of out-of-view samples, none detected).
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location(
    "race_mission", HERE.parents[1] / "packages/simulation/race-auv-docking/runtime/mission.py")
mission_module = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
_spec.loader.exec_module(mission_module)

HFOV_DEG = 82.0
RESOLUTION = (1600, 1200)
VFOV_DEG = math.degrees(2.0 * math.atan(math.tan(math.radians(HFOV_DEG / 2.0)) * RESOLUTION[1] / RESOLUTION[0]))
FOV_MARGIN_DEG = 10.0
FACING_MAX_DEG = 60.0
BASE_TO_NOSE_M = (0.7, 0.0, 0.0)
# nose_tip_link -> camera optical frame (/tf_static), quaternions (x, y, z, w).
CAMERAS = {"cam_front": ((-0.004, 0.0, 0.035), (-0.5, 0.500102, -0.5, 0.499898)),
           "cam_down": ((-0.305, 0.0, -0.128), (0.707179, -0.707035, -3.3e-05, -3.3e-05))}
STATION_BASE_NED = ((4.0, 0.0, 3.95), (0.0, 0.0, 0.0, 1.0))
DOCK_IN_STATION = ((-0.41, 0.325, -0.04), (1.0, 0.0, 0.0, 0.0))
TAGS_IN_DOCK = {"tag25h9:2": ((-0.81, 0.0, -0.21), (-0.5, 0.5, -0.5, 0.5)),
                "tag25h9:13": ((-0.26, 0.37, -0.21), (0.0, -0.707107, 0.707107, 0.0)),
                "tag25h9:17": ((-0.26, -0.37, -0.21), (-0.707107, 0.0, 0.0, 0.707107)),
                "tag36h11:146": ((0.43, 0.0, 0.405), (-0.5, 0.5, -0.5, 0.5)),
                "tag36h11:176": ((0.0, 0.25, -0.012), (0.707107, -0.707107, 0.0, 0.0)),
                "tag36h11:185": ((0.0, -0.25, -0.012), (0.707107, -0.707107, 0.0, 0.0)),
                "tag36h11:541": ((0.47, 0.0, 0.16), (-0.5, 0.5, -0.5, 0.5)),
                "tag36h11:558": ((0.47, 0.0, 0.095), (-0.5, 0.5, -0.5, 0.5))}
DETECTION_RANGE_M = {("cam_front", "tag36h11:146"): (0.5, 7.85), ("cam_front", "tag36h11:541"): (0.0, 2.75),
                     ("cam_front", "tag36h11:558"): (0.5, 3.0)}
FRONT_TAGS = ("tag36h11:146", "tag36h11:541", "tag36h11:558")


def quat_matrix(q) -> list[list[float]]:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def apply(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def transpose(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def add(a, b):
    return [a[i] + b[i] for i in range(3)]


def sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def rpy_matrix(roll: float, pitch: float, yaw: float):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw),
                              math.sin(yaw))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


RX_PI = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]


def tag_geometry() -> dict:
    """{tag: (position in world_ned, the unit vector the tag's printed face points along)}: the face is -z of the
    tag frame (a detected tag's -z pointed at the camera in every trial)."""
    station_p, station_r = list(STATION_BASE_NED[0]), quat_matrix(STATION_BASE_NED[1])
    dock_p = add(station_p, apply(station_r, DOCK_IN_STATION[0]))
    dock_r = matmul(station_r, quat_matrix(DOCK_IN_STATION[1]))
    out = {}
    for tag, (xyz, q) in TAGS_IN_DOCK.items():
        rot = matmul(dock_r, quat_matrix(q))
        out[tag] = (add(dock_p, apply(dock_r, xyz)), [-rot[i][2] for i in range(3)])
    return out


def world_from_controller(x_c: float, y_c: float, z_c: float, yaw_c: float) -> tuple:
    """Inverse of mission.controller_from_ground_truth: the nominal Stonefish-world cg_link pose of a commanded
    controller pose."""
    start, offset = mission_module.START_IN_GROUND_TRUTH_M, mission_module.CONTROLLER_YAW_OFFSET_RAD
    c, s = math.cos(offset), math.sin(offset)
    dx, dy = c * x_c + s * y_c, -s * x_c + c * y_c
    yaw = math.atan2(math.sin(yaw_c - offset), math.cos(yaw_c - offset))
    return (dx + start[0], dy + start[1], z_c, yaw)


def view(x_w: float, y_w: float, z_w: float, yaw_w: float, roll: float = 0.0, pitch: float = 0.0) -> dict:
    """Every tag as each camera sees it from a cg_link pose in Stonefish world_ned."""
    return view_from(matmul(rpy_matrix(roll, pitch, yaw_w), RX_PI), [x_w, y_w, z_w])


def view_from(base_r, base_p) -> dict:
    """Every tag as each camera sees it, from base_link's rotation (FLU in world_ned) and position (= cg_link's)."""
    nose_p = add(base_p, apply(base_r, BASE_TO_NOSE_M))
    tags = tag_geometry()
    out = {}
    for camera, (xyz, q) in CAMERAS.items():
        cam_r = matmul(base_r, quat_matrix(q))
        cam_p = add(nose_p, apply(base_r, xyz))
        rows = {}
        for tag, (position, face) in tags.items():
            in_cam = apply(transpose(cam_r), sub(position, cam_p))
            rng = math.sqrt(sum(v * v for v in in_cam))
            if in_cam[2] > 1e-9:
                ax = math.degrees(math.atan2(in_cam[0], in_cam[2]))
                ay = math.degrees(math.atan2(in_cam[1], in_cam[2]))
            else:
                ax = ay = 180.0
            to_cam = sub(cam_p, position)
            facing = math.degrees(math.acos(max(-1.0, min(1.0, sum(face[i] * to_cam[i] for i in range(3)) / rng))))
            inside = abs(ax) <= HFOV_DEG / 2 and abs(ay) <= VFOV_DEG / 2
            band = DETECTION_RANGE_M.get((camera, tag))
            rows[tag] = {"range_m": rng, "azimuth_deg": ax, "elevation_deg": ay, "facing_deg": facing,
                         "in_fov": abs(ax) <= HFOV_DEG / 2 + FOV_MARGIN_DEG and abs(ay) <= VFOV_DEG / 2 + FOV_MARGIN_DEG,
                         "detectable": bool(inside and band is not None and band[0] <= rng < band[1]
                                            and facing <= FACING_MAX_DEG)}
        out[camera] = rows
    return out


def pose_visibility(pose: dict) -> dict:
    """A pose mission pose (cg_link in the controller's world_ned): what is in_fov and detectable on each camera."""
    world = world_from_controller(pose["x_m"], pose["y_m"], pose["z_m"], pose["yaw_rad"])
    seen = view(*world, roll=pose.get("roll_rad", 0.0), pitch=pose.get("pitch_rad", 0.0))
    return {"label": pose.get("label"), "world": world,
            "in_fov": {cam: sorted(t for t, r in rows.items() if r["in_fov"]) for cam, rows in seen.items()},
            "detectable": {cam: sorted(t for t, r in rows.items() if r["detectable"]) for cam, rows in seen.items()},
            "front_ranges_m": {t: round(seen["cam_front"][t]["range_m"], 3) for t in FRONT_TAGS}}


def out_of_view(result: dict) -> bool:
    return not any(result["in_fov"].values())


def check_mission(mission: dict) -> list[dict]:
    return [pose_visibility(pose) for pose in mission_module.validate_mission(mission)["poses"]]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("request", type=Path, help="a race-auv-docking request (or a bare pose mission) JSON")
    args = parser.parse_args(argv)
    value = json.loads(args.request.read_text())
    for result in check_mission(value.get("mission", value)):
        print(json.dumps({**result, "out_of_view": out_of_view(result)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
