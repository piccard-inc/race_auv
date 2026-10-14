"""The planner's parameters: well-formedness bounds only, and their hash.

The bounds are piccard-physical-ai runtime/mission.py's for a planner mission (protocol v1.5, at 468d65f6); the values
are the preregistration's, in config/docking_planner_v1_5.yaml. The initial set point is the pose the controller holds
when the planner starts (the trials' fallback pose, the dive): cg_link in world_ned, checked for finite values and
angle ranges only. No ROS here.
"""
from __future__ import annotations

import hashlib
import json
import math
from typing import Mapping

PLANNER_BOUNDS = {"tick_hz": (1.0, 10.0), "band_m": (0.001, 1.0), "band_rad": (0.001, 0.5), "settle_s": (0.0, 300.0),
                  "vertical_band_m": (0.001, 0.5), "approach_clearance_m": (0.0, 0.1),
                  "final_deadband_m": (0.001, 0.05), "final_along_deadband_m": (0.001, 0.1),
                  "final_along_interval_s": (0.0, 600.0), "final_arrival_m": (0.0, 0.05),
                  "final_reapproach_s": (0.0, 120.0),
                  "speed_cap_mps": (0.01, 1.0), "final_stage_s": (1.0, 900.0), "max_estimate_age_s": (0.1, 10.0),
                  "max_frozen_estimate_s": (0.5, 60.0), "estimate_filter_s": (0.0, 30.0),
                  "heading_window_s": (0.5, 120.0), "setpoint_deadband_m": (0.0, 0.5),
                  "setpoint_deadband_rad": (0.0, 0.5)}
# Vectors: (length, component bounds). tag_pivot_m is station construction: the forward camera's tag centroid in the
# station dock frame, from the station URDF the fuser loads.
PLANNER_VECTORS = {"tag_pivot_m": (3, (-2.0, 2.0))}
PLANNER_FIELDS = {"standoffs_m", *PLANNER_BOUNDS, *PLANNER_VECTORS}
STANDOFF_BOUNDS_M = (0.0, 8.0)
MAX_STAGES = 8
INITIAL_SETPOINT = "initial_setpoint"
ANGLE_BOUNDS_RAD = {"roll_rad": (-math.pi / 2, math.pi / 2), "pitch_rad": (-math.pi / 2, math.pi / 2),
                    "yaw_rad": (-math.pi, math.pi)}
SETPOINT_FIELDS = ("x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad")


def finite(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_parameters(planner) -> dict:
    """The planner parameters, unchanged, or ValueError naming the first problem."""
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
    return planner


def validate_initial_setpoint(setpoint) -> dict:
    """cg_link in world_ned: six finite numbers, the angles within their ranges."""
    if not isinstance(setpoint, dict) or set(setpoint) != set(SETPOINT_FIELDS):
        raise ValueError(f"{INITIAL_SETPOINT} must be exactly {list(SETPOINT_FIELDS)}")
    for key in SETPOINT_FIELDS:
        low, high = ANGLE_BOUNDS_RAD.get(key, (-math.inf, math.inf))
        if not finite(setpoint[key]) or not low <= setpoint[key] <= high:
            raise ValueError(f"{INITIAL_SETPOINT} {key} must be finite and within [{low}, {high}]")
    return setpoint


def from_ros_parameters(values: Mapping[str, object]) -> tuple[dict, dict]:
    """(planner parameters, initial set point) from the node's parameters by name, validated. ROS holds every value
    as a double and the vectors as arrays; the planner takes lists."""
    expected = PLANNER_FIELDS | {f"{INITIAL_SETPOINT}.{key}" for key in SETPOINT_FIELDS}
    if set(values) != expected:
        raise ValueError(f"parameters must be exactly {sorted(expected)}; got {sorted(values)}")
    planner = {key: list(values[key]) if key in ("standoffs_m", *PLANNER_VECTORS) else values[key]
               for key in PLANNER_FIELDS}
    setpoint = {key: values[f"{INITIAL_SETPOINT}.{key}"] for key in SETPOINT_FIELDS}
    return validate_parameters(planner), validate_initial_setpoint(setpoint)


def load_yaml(path, node: str) -> tuple[dict, dict]:
    """The same from a ROS parameter file (offline use and tests); needs PyYAML."""
    import yaml

    document = yaml.safe_load(open(path, encoding="utf-8").read())
    flat = {}

    def walk(prefix: str, value) -> None:
        if isinstance(value, dict):
            for key, item in value.items():
                walk(f"{prefix}.{key}" if prefix else key, item)
        else:
            flat[prefix] = value

    walk("", document[node]["ros__parameters"])
    return from_ros_parameters(flat)


def canonical(value):
    """Integral floats written as integers: the trial records hold the planner parameters so (3, not 3.0)."""
    if isinstance(value, float) and value.is_integer():
        return int(value)
    if isinstance(value, dict):
        return {key: canonical(item) for key, item in value.items()}
    if isinstance(value, list):
        return [canonical(item) for item in value]
    return value


def parameters_sha256(parameters: dict) -> str:
    """The planner parameters' hash as the trial records state it (planner.parameters_sha256): JSON, sorted keys, no
    spaces, integral numbers as integers. So the v1.5 YAML, all doubles, hashes as the trials' parameters did."""
    return hashlib.sha256(json.dumps(canonical(parameters), sort_keys=True, separators=(",", ":")).encode()).hexdigest()
