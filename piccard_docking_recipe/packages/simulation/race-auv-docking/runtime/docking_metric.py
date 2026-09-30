"""Ground-truth docking geometry and the docking.json summary for race-auv-docking/v1. Pure Python, no ROS.

Frames come from the pinned scenario (world_of_stonefish d51d59e) and URDFs (race_auv da58963, race_station
100bc5a):
- /race_auv/stonefish/odometry is link "Base" of race_auv.scn: the robot origin (nose tip) rolled by pi, i.e.
  the URDF nose_tip_link. The URDF base_link is 0.7 m aft of it and cg_link is base_link rolled by pi.
  auv_dock_point is (-0.305, 0, -0.13) from nose_tip_link.
- /race_station/stonefish/odometry is link "Base" of race_station.scn = URDF race_station/base_link;
  dock_point is (-0.410, 0.325, -0.04) rpy (pi, 0, 0) from it.
The collector checks these constants against the published URDF TF chain and against upstream's
ground_truth_docking node (frame_validation). Ground truth is for scoring only; nothing here feeds the vehicle.
"""
from __future__ import annotations

import math
import statistics

AUV_BASE_TO_DOCK = ((-0.305, 0.0, -0.13), (0.0, 0.0, 0.0))
AUV_BASE_TO_URDF_BASE_LINK = ((-0.7, 0.0, 0.0), (0.0, 0.0, 0.0))
AUV_BASE_TO_CG_LINK = ((-0.7, 0.0, 0.0), (math.pi, 0.0, 0.0))
STATION_BASE_TO_DOCK = ((-0.410, 0.325, -0.04), (math.pi, 0.0, 0.0))
# URDF edges that must reproduce the constants above: (parent, child) -> (xyz, rpy)
URDF_EDGES = {
    ("race_auv/base_link", "race_auv/nose_tip_link"): ((0.7, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("race_auv/base_link", "race_auv/cg_link"): ((0.0, 0.0, 0.0), (math.pi, 0.0, 0.0)),
    ("race_auv/nose_tip_link", "race_auv/auv_dock_point"): AUV_BASE_TO_DOCK,
    ("race_station/base_link", "race_station/dock_point"): STATION_BASE_TO_DOCK,
}
URDF_TOLERANCE = 1e-4
UPSTREAM_TOLERANCE_M = 0.01
SCHEMA = "piccard.race-auv.docking/v1"
CLAIM = ("Synthetic Stonefish simulation; ground-truth geometry from the pinned scenario. No physical docking, "
         "contact-mechanics, perception-accuracy or lab-acceptance claim.")


# ----------------------------------------------------------------------------- rotations
def quat_matrix(x: float, y: float, z: float, w: float) -> list[list[float]]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def rpy_matrix(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def transpose(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def apply(rotation, vector):
    return [sum(rotation[i][k] * vector[k] for k in range(3)) for i in range(3)]


def matrix_rpy(r) -> list[float]:
    pitch = math.asin(max(-1.0, min(1.0, -r[2][0])))
    return [math.atan2(r[2][1], r[2][2]), pitch, math.atan2(r[1][0], r[0][0])]


def rotation_angle(r) -> float:
    return math.acos(max(-1.0, min(1.0, (r[0][0] + r[1][1] + r[2][2] - 1) / 2)))


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(pose, offset):
    """World pose (position, rotation) of a frame fixed at `offset` = (xyz, rpy) in `pose`'s frame."""
    position, rotation = pose
    xyz, rpy = offset
    moved = apply(rotation, xyz)
    return [position[i] + moved[i] for i in range(3)], matmul(rotation, rpy_matrix(*rpy))


def odometry_pose(odometry: dict):
    """(position, rotation) from a flattened odometry row {x, y, z, qx, qy, qz, qw}."""
    return [odometry["x"], odometry["y"], odometry["z"]], quat_matrix(odometry["qx"], odometry["qy"], odometry["qz"], odometry["qw"])


def _norm(vector) -> float:
    return math.sqrt(sum(value * value for value in vector))


# ----------------------------------------------------------------------------- per-sample state
def docking_state(auv: dict, station: dict) -> dict:
    """Dock-point relation from the two ground-truth odometries (world NED)."""
    auv_pose, station_pose = odometry_pose(auv), odometry_pose(station)
    auv_dock, station_dock = compose(auv_pose, AUV_BASE_TO_DOCK), compose(station_pose, STATION_BASE_TO_DOCK)
    delta = [auv_dock[0][i] - station_dock[0][i] for i in range(3)]
    relative_rotation = matmul(transpose(station_dock[1]), auv_dock[1])
    base_link = compose(auv_pose, AUV_BASE_TO_URDF_BASE_LINK)
    to_station_dock = [station_dock[0][i] - base_link[0][i] for i in range(3)]
    return {
        "auv_dock_point_m": auv_dock[0], "station_dock_point_m": station_dock[0],
        "distance_m": _norm(delta),
        "relative_position_in_station_dock_m": apply(transpose(station_dock[1]), delta),
        "relative_rpy_rad": matrix_rpy(relative_rotation),
        "orientation_error_rad": rotation_angle(relative_rotation),
        # the station dock point as the fused AprilTag TF reports it: in the AUV's URDF base_link
        "station_dock_in_auv_base_link_m": apply(transpose(base_link[1]), to_station_dock),
    }


def ground_truth_cg(auv: dict) -> dict:
    """Ground-truth pose of race_auv/cg_link, the frame the controller regulates, in Stonefish world NED."""
    position, rotation = compose(odometry_pose(auv), AUV_BASE_TO_CG_LINK)
    return {"position_m": position, "rpy_rad": matrix_rpy(rotation)}


def base_to_base(auv: dict, station: dict) -> list[float]:
    """Station Base in AUV Base, as upstream's ground_truth_docking node computes it (no link offsets)."""
    (auv_p, auv_r), (station_p, _) = odometry_pose(auv), odometry_pose(station)
    return apply(transpose(auv_r), [station_p[i] - auv_p[i] for i in range(3)])


# ----------------------------------------------------------------------------- validation
def frame_validation(static_edges: dict, upstream_checks: list[dict]) -> dict:
    """Compare the constants with the published URDF TF chain ({"parent->child": {"xyz", "q"}}) and the metric's
    Base-to-Base relation with upstream's ground_truth_docking pose (rows with "difference_m")."""
    edges = []
    for (parent, child), (xyz, rpy) in URDF_EDGES.items():
        observed = static_edges.get(f"{parent}->{child}")
        item = {"edge": f"{parent}->{child}", "expected": {"xyz": list(xyz), "rpy": list(rpy)}, "observed": observed,
                "status": "not_observed"}
        if observed:
            rotation = matmul(transpose(rpy_matrix(*rpy)), quat_matrix(*observed["q"]))
            item["translation_error_m"] = _norm([observed["xyz"][i] - xyz[i] for i in range(3)])
            item["rotation_error_rad"] = rotation_angle(rotation)
            ok = item["translation_error_m"] <= URDF_TOLERANCE and item["rotation_error_rad"] <= URDF_TOLERANCE
            item["status"] = "match" if ok else "mismatch"
        edges.append(item)
    differences = [row["difference_m"] for row in upstream_checks]
    upstream = {"samples": len(differences), "status": "not_observed"}
    if differences:
        upstream.update(median_difference_m=statistics.median(differences), max_difference_m=max(differences),
                        status="consistent" if statistics.median(differences) <= UPSTREAM_TOLERANCE_M else "inconsistent")
    statuses = {item["status"] for item in edges} | {upstream["status"]}
    return {"urdf_edges": edges, "urdf_tolerance": URDF_TOLERANCE, "upstream_ground_truth_node": upstream,
            "upstream_tolerance_m": UPSTREAM_TOLERANCE_M,
            "status": "validated" if statuses <= {"match", "consistent"} else
                      "failed" if statuses & {"mismatch", "inconsistent"} else "incomplete"}


# ----------------------------------------------------------------------------- summary
def _latest(rows: list[dict], kind: str, at: float) -> dict | None:
    candidates = [row for row in rows if row.get("kind") == kind and row["t"] <= at]
    return candidates[-1] if candidates else None


def _window(rows: list[dict], kind: str, start: float, end: float) -> list[dict]:
    return [row for row in rows if row.get("kind") == kind and start <= row["t"] <= end]


def frame_alignment(row: dict) -> dict:
    """The planar rigid transform (yaw about z, then translation) taking controller world_ned to the Stonefish world,
    estimated from one alignment sample of cg_link in both frames."""
    gt, ctrl = row["ground_truth_cg"], row["controller_cg"]
    yaw = wrap(gt["rpy_rad"][2] - ctrl["rpy_rad"][2])
    rotated = apply(rpy_matrix(0.0, 0.0, yaw), ctrl["position_m"])
    return {"yaw_rad": yaw, "origin_m": [gt["position_m"][i] - rotated[i] for i in range(3)]}


def frame_drift(row: dict, alignment: dict) -> dict:
    """Ground truth minus the controller's cg_link mapped through the initial alignment (Stonefish world)."""
    gt, ctrl = row["ground_truth_cg"], row["controller_cg"]
    rotated = apply(rpy_matrix(0.0, 0.0, alignment["yaw_rad"]), ctrl["position_m"])
    position = [gt["position_m"][i] - rotated[i] - alignment["origin_m"][i] for i in range(3)]
    return {"position_m": position, "horizontal_m": math.hypot(position[0], position[1]),
            "yaw_rad": wrap(gt["rpy_rad"][2] - ctrl["rpy_rad"][2] - alignment["yaw_rad"])}


def summarize(rows: list[dict], mission: dict) -> dict:
    """docking.json from collector rows: per commanded pose the dock-point relation at dwell end and over the
    dwell, the controller-frame drift, the trial minimum distance, contacts and the perception comparison."""
    rows = sorted((row for row in rows if isinstance(row.get("t"), (int, float))), key=lambda row: row["t"])
    events = [row for row in rows if row.get("kind") == "event"]
    alignment = [row for row in rows if row.get("kind") == "alignment"]
    origin = frame_alignment(alignment[0]) if alignment else None
    poses = []
    for pose in mission["poses"]:
        start = next((e["t"] for e in events if e.get("event") == "pose_start" and e.get("label") == pose["label"]), None)
        end = next((e["t"] for e in events if e.get("event") == "pose_end" and e.get("label") == pose["label"]), None)
        item = {"label": pose["label"], "commanded": {k: pose[k] for k in pose if k != "label"},
                "start_t": start, "end_t": end, "status": "not_reached", "at_end": None, "dwell": None,
                "controller_frame_drift_at_end": None}
        if start is not None and end is not None:
            last = _latest(rows, "dock", end)
            window = _window(rows, "dock", start, end)
            item["status"] = "observed" if last and window else "no_ground_truth"
            if last:
                item["at_end"] = {"t": last["t"], "distance_m": last["distance_m"],
                                  "relative_position_in_station_dock_m": last["relative_position_in_station_dock_m"],
                                  "relative_rpy_rad": last["relative_rpy_rad"],
                                  "orientation_error_rad": last["orientation_error_rad"],
                                  "auv_speed_mps": _norm(last.get("auv_linear_velocity_mps", [0, 0, 0]))}
                fused = _latest(rows, "fused_dock", end)
                if fused and end - fused["t"] <= 1.0:
                    truth, seen = last["station_dock_in_auv_base_link_m"], fused["position_m"]
                    item["at_end"]["perception"] = {
                        "fused_t": fused["t"], "fused_position_m": seen, "ground_truth_position_m": truth,
                        "error_m": _norm([seen[i] - truth[i] for i in range(3)]),
                        "range_ratio": _norm(seen) / _norm(truth) if _norm(truth) > 1e-9 else None}
            if window:
                distances = [row["distance_m"] for row in window]
                item["dwell"] = {"samples": len(window), "seconds": end - start,
                                 "distance_mean_m": sum(distances) / len(distances),
                                 "distance_max_m": max(distances), "distance_min_m": min(distances),
                                 "orientation_error_max_rad": max(row["orientation_error_rad"] for row in window)}
            aligned = _latest(alignment, "alignment", end)
            if origin and aligned:
                item["controller_frame_drift_at_end"] = frame_drift(aligned, origin)
        poses.append(item)
    dock_rows = [row for row in rows if row.get("kind") == "dock"]
    closest = min(dock_rows, key=lambda row: row["distance_m"]) if dock_rows else None
    contacts = {}
    for row in (row for row in rows if row.get("kind") == "contact"):
        entry = contacts.setdefault(row["contact"], {"events": 0, "first_t": row["t"], "last_t": row["t"],
                                                      "max_normal_force_n": 0.0})
        entry["events"] += 1
        entry["last_t"] = row["t"]
        entry["max_normal_force_n"] = max(entry["max_normal_force_n"], row.get("normal_force_n", 0.0))
    detections = [row for row in rows if row.get("kind") == "detections" and row.get("count", 0) > 0]
    by_camera = {}
    for row in detections:
        camera = by_camera.setdefault(row["camera"], {"messages_with_tags": 0, "first_t": row["t"], "tag_ids": set(),
                                                      "identity_pose_detections": 0})
        camera["messages_with_tags"] += 1
        camera["tag_ids"].update(row.get("ids", []))
        # race_auv_camera_pkg publishes an identity camera-to-tag pose when the pose solve fails
        camera["identity_pose_detections"] += sum(_norm(position) < 1e-9 for position in row.get("positions_m", []))
    fused_rows = [row for row in rows if row.get("kind") == "fused_dock"]
    return {
        "schema": SCHEMA, "claim_boundary": CLAIM, "time_basis": "collector_monotonic_receive_seconds",
        "definitions": {"auv_base_to_dock_point": AUV_BASE_TO_DOCK, "station_base_to_dock_point": STATION_BASE_TO_DOCK,
                        "auv_base_to_urdf_base_link": AUV_BASE_TO_URDF_BASE_LINK,
                        "auv_base_to_cg_link": AUV_BASE_TO_CG_LINK,
                        "distance": "euclidean distance between auv_dock_point and station dock_point, world NED",
                        "orientation_error": "rotation angle of station_dock^T * auv_dock",
                        "controller_frame": "controller world_ned -> Stonefish world as yaw about z then "
                                            "translation, from the first cg_link alignment sample; drift is ground "
                                            "truth minus the controller cg_link mapped through it"},
        "apriltag_tag_size": mission.get("apriltag_tag_size", "lab_configured"),
        "poses": poses,
        "minimum_distance": {"distance_m": closest["distance_m"], "t": closest["t"]} if closest else None,
        "controller_frame_in_ground_truth": origin,
        "contacts": contacts,
        "perception": {"first_tag_detection_t": detections[0]["t"] if detections else None,
                       "cameras": {name: {**value, "tag_ids": sorted(value["tag_ids"])} for name, value in by_camera.items()},
                       "fused_dock_samples": len(fused_rows),
                       "first_fused_dock_t": fused_rows[0]["t"] if fused_rows else None},
    }
