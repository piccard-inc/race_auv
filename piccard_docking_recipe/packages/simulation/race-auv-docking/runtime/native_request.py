#!/usr/bin/env python3
"""A race-auv-docking/v1 request for a native trial (run_native_trial.sh): its parts as the runner takes them, and
the race_auv_docking_planner parameter file for a planner mission.

- split: request -> expected-gains.json, mission.json and context.json in a new directory, plus limits.env for the
  runner (horizon and wall timeout). Numbers are written as the platform's submit path writes them: a float with an
  integral value as an integer (2.0 -> 2). The collector hashes the mission's planner parameters as received and the
  planner package hashes them that way too (race_auv_docking_planner.parameters.parameters_sha256), so a request
  written with decimal points still gives the planner and the collector one hash. The values do not change.
- planner-params: a planner mission -> the package's ROS parameter file: every planner parameter as a double (the
  vectors as double arrays), and initial_setpoint, the fallback pose the controller holds when the planner starts.

The mission is checked here by mission.validate_mission; the gains by prepare_candidate at install, against the
workspace's configured limits. No ROS.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from mission import is_planner_mission, validate_mission

PLANNER_NODE = "piccard_planner"
SETPOINT_FIELDS = ("x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad")
LIMITS = {"horizon_s": "HORIZON_SECONDS", "wall_timeout_s": "WALL_TIMEOUT_SECONDS"}


def canonical(value):
    """Integral floats as integers, recursively; nothing else changes."""
    if isinstance(value, float) and value.is_integer():
        return int(value)
    if isinstance(value, dict):
        return {key: canonical(item) for key, item in value.items()}
    if isinstance(value, list):
        return [canonical(item) for item in value]
    return value


def split(request: dict, target: Path) -> dict:
    """The request's parts, checked, in target (which must not exist). Returns the written paths."""
    if not isinstance(request, dict) or not {"gains", "mission", *LIMITS} <= set(request) \
            or set(request) - {"gains", "mission", "context", *LIMITS}:
        raise ValueError(f"a request has gains, mission, {', '.join(LIMITS)} and optionally context")
    request = canonical(request)
    validate_mission(request["mission"])
    if not isinstance(request["gains"], dict):
        raise ValueError("gains must be an object")
    for key in LIMITS:
        if not (isinstance(request[key], int) and not isinstance(request[key], bool) and 0 < request[key] <= 3600):
            raise ValueError(f"{key} must be an integer number of seconds in 1..3600")
    target.mkdir(parents=True)
    written = {}
    for name, value in (("expected-gains.json", request["gains"]), ("mission.json", request["mission"]),
                        ("context.json", request.get("context") or {})):
        (target / name).write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
        written[name] = str(target / name)
    (target / "limits.env").write_text("".join(f"{variable}={request[key]}\n" for key, variable in LIMITS.items()))
    written["limits.env"] = str(target / "limits.env")
    return written


def planner_params(mission: dict) -> dict:
    """The race_auv_docking_planner parameter document for this planner mission."""
    mission = validate_mission(mission)
    if not is_planner_mission(mission):
        raise ValueError("only a planner mission has planner parameters")
    planner = {key: [float(v) for v in value] if isinstance(value, list) else float(value)
               for key, value in sorted(mission["planner"].items())}
    pose = mission["fallback_pose"]
    return {PLANNER_NODE: {"ros__parameters": {**planner,
                                                "initial_setpoint": {key: float(pose[key]) for key in SETPOINT_FIELDS}}}}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="action", required=True)
    one = sub.add_parser("split")
    one.add_argument("--request", type=Path, required=True)
    one.add_argument("--output", type=Path, required=True, help="a new directory")
    two = sub.add_parser("planner-params")
    two.add_argument("--mission", type=Path, required=True)
    two.add_argument("--output", type=Path, required=True, help="a new file")
    args = parser.parse_args(argv)
    if args.action == "split":
        split(json.loads(args.request.read_text()), args.output)
    else:
        if args.output.exists():
            raise SystemExit(f"{args.output} exists; refusing overwrite")
        args.output.write_text(yaml.safe_dump(planner_params(json.loads(args.mission.read_text())), sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
