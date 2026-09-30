"""Build-only sanitization of the fetched RACE sources: remove real-world vehicle configuration.

Only the simulation configs stay in the image; their gains are replaced per trial by prepare_candidate.py.
Never prints configuration values.
"""
from pathlib import Path
import json
import shutil
import sys

# Real-vehicle and hardware configuration (controller gains, helm, sensors, cameras, modems, C2, GNSS).
REAL_WORLD_FILES = (
    "race_auv/race_auv_config/mvp_control_config/config.yaml",
    "race_auv/race_auv_config/mvp_mission_config/helm.yaml",
    "race_auv/race_auv_bringup/config/apriltag.yaml",
    "race_auv/race_auv_bringup/config/explore_cam_apriltag.yaml",
    "race_auv/race_auv_bringup/config/bhv_params.yaml",
    "race_auv/race_auv_bringup/config/mvp_control.yaml",
    "race_auv/race_auv_bringup/config/mvp_mission.yaml",
    "race_auv/race_auv_bringup/config/navsat.yaml",
    "race_auv/race_auv_bringup/config/localization/robot_localization.yaml",
    "race_auv/race_auv_bringup/config/localization/robot_initialization.yaml",
    "race_auv/race_auv_bringup/config/c2/mvp_c2.yaml",
    "race_auv/race_auv_bringup/config/c2/mvp_gui_params.yaml",
)
REAL_WORLD_DIRS = (
    "race_auv/race_auv_bringup/config/sensors",
    "race_auv/race_auv_bringup/config/camera",
    "race_auv/race_auv_bringup/config/evologics",
)
# The simulation configs the trial runtime installs into or launches with; missing any is a hard stop.
REQUIRED_SIM_FILES = (
    "race_auv/race_auv_config/mvp_control_config/config_sim.yaml",
    "race_auv/race_auv_config/mvp_mission_config/helm_sim.yaml",
    "race_auv/race_auv_bringup/config/bhv_params_sim.yaml",
    "race_auv/race_auv_bringup/config/mvp_control_sim.yaml",
    "race_auv/race_auv_bringup/config/mvp_mission_sim.yaml",
    "race_auv/race_auv_bringup/config/sim_params.yaml",
    "race_auv/race_auv_bringup/config/simulation/apriltag.yaml",
    "world_of_stonefish/world/race_auv_test.scn",
)


def sanitize(src: Path) -> dict:
    missing = [name for name in REQUIRED_SIM_FILES if not (src / name).is_file()]
    if missing:
        raise SystemExit(f"expected simulation files missing; refuse guessed layout: {missing}")
    removed = []
    for name in REAL_WORLD_FILES:
        path = src / name
        if path.is_file():
            path.unlink()
            removed.append(name)
    for name in REAL_WORLD_DIRS:
        path = src / name
        if path.is_dir():
            shutil.rmtree(path)
            removed.append(name + "/")
    record = {"schema": "piccard.race-auv.sanitization/v1",
              "status": "simulation-only sources; real-vehicle configuration removed at build time",
              "removed": removed, "kept_simulation_files": list(REQUIRED_SIM_FILES),
              "real_world_gain_file_present": False}
    (src / "piccard-sanitization.json").write_text(json.dumps(record, indent=2) + "\n")
    return record


if __name__ == "__main__":
    sanitize(Path(sys.argv[1]))
