"""piccard.race-auv.pose-mission/v1: an ordered list of desired cg_link poses in world_ned, each held for a
dwell time, published to bhv_direct_control. The runtime checks shape and bounds only, never science.

Poses are in the controller's world_ned (race_auv/world_ned), not the Stonefish world. In the pinned stack
(race_auv da58963) the robot_localization EKF fuses IMU roll/pitch, rates, DVL and depth but not absolute yaw, so
race_auv/odom is base_link's start pose (FLU, dead-reckoned, drifting), and description.launch.py places
race_auv/world_ned at rpy (3.1415, 0, 1.571) from it. With the scenario's start (nose at Stonefish world x -3.1,
heading north; base_link 0.7 m aft at x -3.8) that makes world_ned the Stonefish world rotated +90 deg about z:
    x_C = -y_W,   y_C = x_W + 3.8,   z_C = z_W (depth),   yaw_C = yaw_W + 1.571.
The bounds are the tank interior from the pinned scenario, mapped into that frame (#104). world_of_stonefish d51d59e's
objects/ORL_Tank.obj (world z 5.1, rpy -1.57 0 1.57) is a double shell: an outer box (x_W +/-5.18, y_W +/-3.66, floor
z 5.10) around the inner box the vehicle swims in, whose floor is at z_W 4.487..4.494 (campaigns/tank-floor-v1.json,
tools/campaigns/extract_tank_floor.py). cg_link must lie inside the inner box, and the vehicle's lowest point (its
legs, 0.180 m below cg_link when level) must stay above the shallowest interior floor, so z_C < 4.4868 - 0.1799.
Both bounds are rounded inward to the centimetre; runtime/test_race_runtime.py ties them to the extracted geometry.
The collector measures the actual origin, yaw offset and drift (docking.json controller_frame_in_ground_truth). There
is no stand-off floor, so a pose may put the AUV in contact with the station or a wall; campaign builders keep their
own clearance margins.
"""
from __future__ import annotations

import math

SCHEMA = "piccard.race-auv.pose-mission/v1"
START_IN_GROUND_TRUTH_M = (-3.8, 0.0, 0.0)  # race_auv.scn world_transform x -3.1, base_link 0.7 m aft
CONTROLLER_YAW_OFFSET_RAD = 1.571  # description.launch.py world -> world_ned yaw
TANK_BOUNDS_M = {"x_m": (-3.04, 3.04), "y_m": (-0.77, 8.36), "z_m": (0.0, 4.30)}  # x_C = -y_W, y_C = x_W + 3.8
ANGLE_BOUNDS_RAD = {"roll_rad": (-math.pi / 2, math.pi / 2), "pitch_rad": (-math.pi / 2, math.pi / 2),
                    "yaw_rad": (-math.pi, math.pi)}
DWELL_BOUNDS_S = (1.0, 900.0)
MAX_POSES = 20
POSE_FIELDS = {"label", *TANK_BOUNDS_M, *ANGLE_BOUNDS_RAD, "dwell_s"}
# Detector tag-size convention (#79): the lab's configured sizes are the full texture box (0.15/0.05 m);
# the rendered black-square edge is 0.8 of that. The default reproduces the lab configuration.
TAG_SIZE_CONVENTIONS = ("lab_configured", "black_square_edge")
REQUIRED = {"schema", "mission_id", "question", "frame_id", "child_frame_id", "poses", "seed"}
OPTIONAL = {"apriltag_tag_size"}


def finite(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_mission(value) -> dict:
    """Return the mission with defaults filled in, or raise ValueError naming the first problem."""
    if not isinstance(value, dict) or not REQUIRED <= set(value) or set(value) - REQUIRED - OPTIONAL:
        raise ValueError("pose mission fields mismatch")
    if value["schema"] != SCHEMA:
        raise ValueError("pose mission schema mismatch")
    for key in ("mission_id", "question"):
        if not isinstance(value[key], str) or not value[key].strip():
            raise ValueError(f"{key} required")
    if value["frame_id"] != "world_ned" or value["child_frame_id"] != "cg_link":
        raise ValueError("poses are cg_link in world_ned")
    if value["seed"] is not None:
        raise ValueError("the pinned simulator exposes no seed; seed must be null")
    convention = value.get("apriltag_tag_size", "lab_configured")
    if convention not in TAG_SIZE_CONVENTIONS:
        raise ValueError(f"apriltag_tag_size must be one of {TAG_SIZE_CONVENTIONS}")
    poses = value["poses"]
    if not isinstance(poses, list) or not 1 <= len(poses) <= MAX_POSES:
        raise ValueError(f"mission requires 1..{MAX_POSES} poses")
    labels = set()
    for index, pose in enumerate(poses):
        if not isinstance(pose, dict) or set(pose) != POSE_FIELDS:
            raise ValueError(f"pose {index} fields mismatch")
        if not isinstance(pose["label"], str) or not pose["label"].strip() or pose["label"] in labels:
            raise ValueError(f"pose {index} needs a unique label")
        labels.add(pose["label"])
        for key, (low, high) in {**TANK_BOUNDS_M, **ANGLE_BOUNDS_RAD, "dwell_s": DWELL_BOUNDS_S}.items():
            if not finite(pose[key]) or not low <= pose[key] <= high:
                raise ValueError(f"pose {index} {key} outside [{low}, {high}]")
    return {**value, "apriltag_tag_size": convention}


def controller_from_ground_truth(x_w: float, y_w: float, z_w: float, yaw_w: float = 0.0) -> tuple:
    """Nominal (drift-free) controller world_ned pose of a Stonefish-world cg_link pose."""
    dx, dy = x_w - START_IN_GROUND_TRUTH_M[0], y_w - START_IN_GROUND_TRUTH_M[1]
    c, s = math.cos(CONTROLLER_YAW_OFFSET_RAD), math.sin(CONTROLLER_YAW_OFFSET_RAD)
    return (c * dx - s * dy, s * dx + c * dy, z_w, math.atan2(math.sin(yaw_w + CONTROLLER_YAW_OFFSET_RAD),
                                                            math.cos(yaw_w + CONTROLLER_YAW_OFFSET_RAD)))


def mission_seconds(mission: dict) -> float:
    return float(sum(pose["dwell_s"] for pose in mission["poses"]))
