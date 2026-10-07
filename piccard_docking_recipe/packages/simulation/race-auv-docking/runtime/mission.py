"""The two mission kinds a race-auv-docking request carries, told apart by their "schema" literal. The runtime checks
shape and bounds only, never science.
- piccard.race-auv.pose-mission/v1: an ordered list of desired cg_link poses in world_ned, each held for a dwell time,
  published to bhv_direct_control by the collector.
- piccard.race-auv.planner-mission/v1 (M3, #109): a fallback pose (the dive; the tags are not visible from the
  surface), then runtime/planner_fused_dock.py's staged approach on the tag-fused station dock point. The planner
  parameters are required, with well-formedness bounds only; apriltag_tag_size defaults to black_square_edge.

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

import hashlib
import json
import math

SCHEMA = "piccard.race-auv.pose-mission/v1"
PLANNER_SCHEMA = "piccard.race-auv.planner-mission/v1"
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
PLANNER_REQUIRED = {"schema", "mission_id", "question", "frame_id", "child_frame_id", "seed", "fallback_pose",
                    "planner"}
PLANNER_DEFAULT_TAG_SIZE = "black_square_edge"  # the black-region convention the SOS Lab confirmed (#107, #88)
# Planner parameters: well-formedness bounds only; the values are the preregistration's (piccard-experiments #88).
PLANNER_BOUNDS = {"tick_hz": (1.0, 10.0), "band_m": (0.001, 1.0), "band_rad": (0.001, 0.5), "settle_s": (0.0, 300.0),
                  "vertical_band_m": (0.001, 0.5), "approach_clearance_m": (0.0, 0.1),
                  "final_deadband_m": (0.001, 0.05), "final_along_deadband_m": (0.001, 0.1),
                  "final_along_interval_s": (0.0, 600.0), "final_arrival_m": (0.0, 0.05),
                  "final_reapproach_s": (0.0, 120.0),
                  "speed_cap_mps": (0.01, 1.0), "final_stage_s": (1.0, 900.0), "max_estimate_age_s": (0.1, 10.0),
                  "max_frozen_estimate_s": (0.5, 60.0), "estimate_filter_s": (0.0, 30.0),
                  "heading_window_s": (0.5, 120.0), "setpoint_deadband_m": (0.0, 0.5),
                  "setpoint_deadband_rad": (0.0, 0.5)}
# Vectors: (length, component bounds). tag_pivot_m is station construction (the forward camera's tag centroid in the
# dock frame), checked at trial start against the URDF the fuser loads (tag_pivot.py).
PLANNER_VECTORS = {"tag_pivot_m": (3, (-2.0, 2.0))}
PLANNER_FIELDS = {"standoffs_m", *PLANNER_BOUNDS, *PLANNER_VECTORS}
STANDOFF_BOUNDS_M = (0.0, 8.0)
MAX_STAGES = 8
# The simulator seed (runtime/simulator_seed.py): null draws one. The request schema keeps it null; a native run's
# mission may give one. It reaches the noise generators only through simulator-patches/stonefish_seed_v1.patch.
MAX_SEED = 2 ** 32 - 1


def finite(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_mission(value) -> dict:
    """Return the mission with defaults filled in, or raise ValueError naming the first problem."""
    if isinstance(value, dict) and value.get("schema") == PLANNER_SCHEMA:
        return validate_planner_mission(value)
    if not isinstance(value, dict) or not REQUIRED <= set(value) or set(value) - REQUIRED - OPTIONAL:
        raise ValueError("pose mission fields mismatch")
    if value["schema"] != SCHEMA:
        raise ValueError("pose mission schema mismatch")
    convention = check_header(value, "lab_configured")
    poses = value["poses"]
    if not isinstance(poses, list) or not 1 <= len(poses) <= MAX_POSES:
        raise ValueError(f"mission requires 1..{MAX_POSES} poses")
    labels = set()
    for index, pose in enumerate(poses):
        check_pose(pose, f"pose {index}")
        if pose["label"] in labels:
            raise ValueError(f"pose {index} needs a unique label")
        labels.add(pose["label"])
    return {**value, "apriltag_tag_size": convention}


def check_header(value: dict, default_tag_size: str) -> str:
    """The fields both mission kinds share; returns the tag-size convention with its default."""
    for key in ("mission_id", "question"):
        if not isinstance(value[key], str) or not value[key].strip():
            raise ValueError(f"{key} required")
    if value["frame_id"] != "world_ned" or value["child_frame_id"] != "cg_link":
        raise ValueError("poses are cg_link in world_ned")
    if value["seed"] is not None and not (isinstance(value["seed"], int) and not isinstance(value["seed"], bool)
                                          and 0 <= value["seed"] <= MAX_SEED):
        raise ValueError(f"seed must be null (the runner draws one) or an integer in [0, {MAX_SEED}]")
    convention = value.get("apriltag_tag_size", default_tag_size)
    if convention not in TAG_SIZE_CONVENTIONS:
        raise ValueError(f"apriltag_tag_size must be one of {TAG_SIZE_CONVENTIONS}")
    return convention


def check_pose(pose, name: str) -> None:
    if not isinstance(pose, dict) or set(pose) != POSE_FIELDS:
        raise ValueError(f"{name} fields mismatch")
    if not isinstance(pose["label"], str) or not pose["label"].strip():
        raise ValueError(f"{name} needs a label")
    for key, (low, high) in {**TANK_BOUNDS_M, **ANGLE_BOUNDS_RAD, "dwell_s": DWELL_BOUNDS_S}.items():
        if not finite(pose[key]) or not low <= pose[key] <= high:
            raise ValueError(f"{name} {key} outside [{low}, {high}]")


def validate_planner_mission(value) -> dict:
    """piccard.race-auv.planner-mission/v1, with apriltag_tag_size defaulting to black_square_edge."""
    if not isinstance(value, dict) or not PLANNER_REQUIRED <= set(value) or set(value) - PLANNER_REQUIRED - OPTIONAL:
        raise ValueError("planner mission fields mismatch")
    convention = check_header(value, PLANNER_DEFAULT_TAG_SIZE)
    check_pose(value["fallback_pose"], "fallback_pose")
    planner = value["planner"]
    if not isinstance(planner, dict) or set(planner) != PLANNER_FIELDS:
        raise ValueError(f"planner parameters must be exactly {sorted(PLANNER_FIELDS)}")
    for key, (low, high) in PLANNER_BOUNDS.items():
        if not finite(planner[key]) or not low <= planner[key] <= high:
            raise ValueError(f"planner {key} outside [{low}, {high}]")
    for deadband, band in (("setpoint_deadband_m", "vertical_band_m"), ("setpoint_deadband_m", "band_m"),
                           ("setpoint_deadband_rad", "band_rad")):
        if planner[deadband] > planner[band]:  # protocol v1.3: else a stage can wait forever (planner notes, 6)
            raise ValueError(f"planner {deadband} must not exceed {band}")
    if planner["final_arrival_m"] >= planner["final_along_deadband_m"]:  # protocol v1.5: else hold and approach flap
        raise ValueError("planner final_arrival_m must be less than final_along_deadband_m")
    for key, (length, (low, high)) in PLANNER_VECTORS.items():
        vector = planner[key]
        if (not isinstance(vector, list) or len(vector) != length
                or not all(finite(v) and low <= v <= high for v in vector)):
            raise ValueError(f"planner {key}: {length} numbers within [{low}, {high}]")
    standoffs = planner["standoffs_m"]
    if (not isinstance(standoffs, list) or not 1 <= len(standoffs) <= MAX_STAGES
            or not all(finite(s) and STANDOFF_BOUNDS_M[0] <= s <= STANDOFF_BOUNDS_M[1] for s in standoffs)):
        raise ValueError(f"planner standoffs_m: 1..{MAX_STAGES} stand-offs within {list(STANDOFF_BOUNDS_M)} m")
    if any(a <= b for a, b in zip(standoffs, standoffs[1:])) or standoffs[-1] != 0:
        raise ValueError("planner standoffs_m must decrease strictly and end at 0 (the dock point)")
    return {**value, "apriltag_tag_size": convention}


def is_planner_mission(mission: dict) -> bool:
    return mission.get("schema") == PLANNER_SCHEMA


def parameters_sha256(parameters: dict) -> str:
    """The planner parameters' hash, as trial.json and race_m3_metrics state it: JSON, sorted keys, no spaces."""
    return hashlib.sha256(json.dumps(parameters, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def controller_from_ground_truth(x_w: float, y_w: float, z_w: float, yaw_w: float = 0.0) -> tuple:
    """Nominal (drift-free) controller world_ned pose of a Stonefish-world cg_link pose."""
    dx, dy = x_w - START_IN_GROUND_TRUTH_M[0], y_w - START_IN_GROUND_TRUTH_M[1]
    c, s = math.cos(CONTROLLER_YAW_OFFSET_RAD), math.sin(CONTROLLER_YAW_OFFSET_RAD)
    return (c * dx - s * dy, s * dx + c * dy, z_w, math.atan2(math.sin(yaw_w + CONTROLLER_YAW_OFFSET_RAD),
                                                            math.cos(yaw_w + CONTROLLER_YAW_OFFSET_RAD)))


def mission_seconds(mission: dict) -> float | None:
    """The sum of the dwells; None for a planner mission, whose stages end on settling (the horizon bounds it)."""
    if is_planner_mission(mission):
        return None
    return float(sum(pose["dwell_s"] for pose in mission["poses"]))
