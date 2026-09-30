#!/usr/bin/env python3
"""Install one race-auv-docking/v1 trial configuration in a fresh disposable container, then verify it.

  install: gains -> control_modes.flight and a new control_modes.docking in config_sim.yaml; a direct_control
           helm state (control mode docking, to/from start and kill) with bhv_direct_control in helm_sim.yaml
           and bhv_params_sim.yaml; optionally a detector config with black-square-edge tag sizes.
  verify:  re-read every installed file and compare against the request before anything launches.
Only the trial container's copies change; the image and the upstream repositories do not.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import yaml

MODES = {"flight": ("u", "v", "z", "roll", "pitch", "yaw"), "docking": ("x", "y", "z", "roll", "pitch", "yaw")}
FIELDS = ("p", "i", "d", "v", "pid_min", "pid_max")
GAIN_CEILING = 10000.0  # well-formedness only: finite, non-negative, below an absurdity ceiling
FREE_LIMIT_AXES = {("docking", "x"), ("docking", "y")}  # no upstream constant; limits come from the request
# The black-square edge of the pinned tag textures is 0.8 of the textured box face (evidence in recipe-v1.json).
BLACK_EDGE_FRACTION = 0.8
HELM_STATE, HELM_BEHAVIOR = "direct_control", "bhv_direct_control"
BHV_PARAMS = {"default_bhv_world_link": "world_ned", "default_bhv_child_link": "cg_link"}


def paths(workspace: Path) -> dict:
    share = workspace / "install/share"
    return {"control": [workspace / "src/race_auv/race_auv_config/mvp_control_config/config_sim.yaml",
                        share / "race_auv_config/mvp_control_config/config_sim.yaml"],
            "helm": [share / "race_auv_config/mvp_mission_config/helm_sim.yaml"],
            "bhv": [share / "race_auv_bringup/config/bhv_params_sim.yaml"],
            "apriltag": share / "race_auv_bringup/config/simulation/apriltag.yaml"}


def finite(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def configured_limits(control: dict) -> dict:
    """(pid_min, pid_max) per flight axis as configured upstream; docking z/roll/pitch/yaw share them."""
    flight = control["control_modes"]["flight"]
    return {axis: (flight[axis]["pid_min"], flight[axis]["pid_max"]) for axis in MODES["flight"]}


def validate(gains, limits: dict) -> dict:
    if not isinstance(gains, dict) or set(gains) != set(MODES):
        raise ValueError("gains must contain exactly the flight and docking modes")
    for mode, axes in MODES.items():
        if not isinstance(gains[mode], dict) or set(gains[mode]) != set(axes):
            raise ValueError(f"{mode} must contain exactly {axes}")
        for axis in axes:
            values = gains[mode][axis]
            if not isinstance(values, dict) or set(values) != set(FIELDS):
                raise ValueError(f"{mode}.{axis}: fields must be {FIELDS}")
            for field in FIELDS:
                if not finite(values[field]):
                    raise ValueError(f"{mode}.{axis}.{field}: finite number required")
            for field in ("p", "i", "d", "v"):
                if not 0 <= values[field] <= GAIN_CEILING:
                    raise ValueError(f"{mode}.{axis}.{field}: must be within [0, {GAIN_CEILING:g}]")
            low, high = values["pid_min"], values["pid_max"]
            if (mode, axis) in FREE_LIMIT_AXES:
                if not -GAIN_CEILING <= low < 0 < high <= GAIN_CEILING:
                    raise ValueError(f"{mode}.{axis}: output limits must satisfy pid_min < 0 < pid_max")
            elif (low, high) != limits[axis]:
                raise ValueError(f"{mode}.{axis}: output limits must stay {limits[axis]} as configured")
    return gains


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode()).hexdigest()


def install_helm(helm: dict) -> dict:
    fsm, behaviors = helm["finite_state_machine"], helm["behaviors"]
    if HELM_STATE in fsm or HELM_BEHAVIOR in behaviors:
        raise ValueError("direct_control already present; refuse to merge into an unexpected helm file")
    for state in ("start", "kill"):
        fsm[state]["transitions"] = list(fsm[state]["transitions"]) + [HELM_STATE]
    fsm[HELM_STATE] = {"control_mode": "docking", "transitions": ["start", "kill"]}
    behaviors[HELM_BEHAVIOR] = {"plugin": "helm/DirectControl", "priority": {HELM_STATE: 1}}
    return helm


def corrected_apriltag(config: dict) -> dict:
    """Scale every tag size, the global list and each camera's override (the detector reads the override)."""
    apriltag = config["apriltag"]
    lists = [apriltag["tags"]] + [camera["tags"] for camera in apriltag["cameras"] if camera.get("tags") is not None]
    for tag in (tag for tags in lists for tag in tags):
        tag["size"] = round(tag["size"] * BLACK_EDGE_FRACTION, 6)
    return config


def install(gains: dict, mission: dict, workspace: Path, output: Path) -> dict:
    files = paths(workspace)
    control = yaml.safe_load(files["control"][-1].read_text())
    validate(gains, configured_limits(control))
    output.mkdir(parents=True, exist_ok=True)
    records = []
    for path in files["control"]:
        data = yaml.safe_load(path.read_text())
        modes = data["control_modes"]
        if "flight" not in modes or "docking" in modes:
            raise ValueError(f"{path}: expected flight and no docking mode")
        modes["flight"], modes["docking"] = gains["flight"], gains["docking"]
        text = yaml.safe_dump(data, sort_keys=False)
        path.write_text(text)
        records.append({"path": str(path), "sha256": sha256_text(text)})
    for path in files["helm"]:
        text = yaml.safe_dump(install_helm(yaml.safe_load(path.read_text())), sort_keys=False)
        path.write_text(text)
        records.append({"path": str(path), "sha256": sha256_text(text)})
    for path in files["bhv"]:
        data = yaml.safe_load(path.read_text()) or {}
        if HELM_BEHAVIOR in data:
            raise ValueError(f"{path}: bhv_direct_control already configured")
        data[HELM_BEHAVIOR] = dict(BHV_PARAMS)
        text = yaml.safe_dump(data, sort_keys=False)
        path.write_text(text)
        records.append({"path": str(path), "sha256": sha256_text(text)})
    apriltag_config = None
    if mission["apriltag_tag_size"] == "black_square_edge":
        text = yaml.safe_dump(corrected_apriltag(yaml.safe_load(files["apriltag"].read_text())), sort_keys=False)
        apriltag_config = output / "apriltag-black-square-edge.yaml"
        apriltag_config.write_text(text)
        records.append({"path": str(apriltag_config), "sha256": sha256_text(text)})
    record = {"schema": "piccard.race-auv.candidate/v1", "gains": gains, "helm_state": HELM_STATE,
              "helm_behavior": {HELM_BEHAVIOR: {"plugin": "helm/DirectControl", "control_mode": "docking"}},
              "apriltag_tag_size": mission["apriltag_tag_size"],
              "apriltag_config": str(apriltag_config) if apriltag_config else None,
              "configurations": records, "scope": "simulation only, no hardware authority"}
    (output / "candidate.json").write_text(json.dumps(record, indent=2) + "\n")
    return record


def verify(gains: dict, mission: dict, workspace: Path, output: Path) -> dict:
    files = paths(workspace)
    for path in files["control"]:
        modes = yaml.safe_load(path.read_text())["control_modes"]
        for mode, axes in MODES.items():
            installed = modes.get(mode, {})
            if set(installed) != set(axes) or any(
                    not math.isclose(float(installed[axis][field]), float(gains[mode][axis][field]),
                                     rel_tol=2e-7, abs_tol=1e-8) for axis in axes for field in FIELDS):
                raise ValueError(f"{path}: installed {mode} gains do not match the request")
    for path in files["helm"]:
        helm = yaml.safe_load(path.read_text())
        state = helm["finite_state_machine"].get(HELM_STATE)
        if state != {"control_mode": "docking", "transitions": ["start", "kill"]} or \
                helm["behaviors"].get(HELM_BEHAVIOR, {}).get("plugin") != "helm/DirectControl":
            raise ValueError(f"{path}: direct_control state or behavior not installed")
    for path in files["bhv"]:
        if yaml.safe_load(path.read_text()).get(HELM_BEHAVIOR) != BHV_PARAMS:
            raise ValueError(f"{path}: bhv_direct_control params not installed")
    for name in ("config.yaml", "helm.yaml"):
        for directory in ("mvp_control_config", "mvp_mission_config"):
            if (workspace / "install/share/race_auv_config" / directory / name).exists():
                raise ValueError("real-vehicle configuration present; refuse run")
    record = json.loads((output / "candidate.json").read_text())
    if record["gains"] != gains or record["apriltag_tag_size"] != mission["apriltag_tag_size"]:
        raise ValueError("candidate record does not match the request")
    result = {"schema": "piccard.race-auv.candidate-install-verification/v1", "status": "verified",
              "configurations": [{"path": item["path"], "sha256": hashlib.sha256(Path(item["path"]).read_bytes()).hexdigest()}
                                 for item in record["configurations"]],
              "real_world_configuration_present": False}
    (output / "candidate-install-verification.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    return result


def main(argv: list[str] | None = None) -> int:
    import mission as mission_module
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", choices=("install", "verify"))
    parser.add_argument("--gains", type=Path, required=True)
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument("--workspace", type=Path, default=Path("/opt/race_ws"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    gains = json.loads(args.gains.read_text())
    mission = mission_module.validate_mission(json.loads(args.mission.read_text()))
    (install if args.action == "install" else verify)(gains, mission, args.workspace, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
