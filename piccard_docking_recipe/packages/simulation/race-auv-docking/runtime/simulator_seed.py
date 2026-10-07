#!/usr/bin/env python3
"""The simulator seed of one trial: chosen before the launch, recorded from what the simulator printed.

Stonefish 7d52673 seeds its sensor-noise generators (sensors/Sensor.cpp, comms/USBL.cpp) from std::random_device when
the library loads: a new seed each run, never printed, not settable. In the RACE vehicle the noise it drives is the
pressure sensor's (2.0 Pa) and the DVL's (velocity 0.01 m/s, altitude 0.03 m); the IMU's noise is set to zero. simulator-patches/stonefish_seed_v1.patch seeds
them from STONEFISH_SEED when it is set (Sensor: the seed, USBL: the seed + 1, mod 2^32) and prints each seed used.

- choose: the runner calls it before the launch. The seed is the mission's `seed` when that is an integer, else one
  drawn here. It is written to simulator-seed.json in the trial output, and printed for the runner to export as
  STONEFISH_SEED.
- record: the collector's trial.json `simulator_seed`, from simulator-seed.json and the simulator's lines in
  launch.log. With a patched simulator it is {"value": <seed>, "support": "stonefish_seed_v1", ...}. With the pinned,
  unpatched simulator nothing is printed and the value is null with support "not_exposed_by_pinned_runner": the
  exported seed was not used. A printed seed that differs from the exported one is recorded as printed and flagged.

A seed fixes the sequence of sensor-noise draws, not the trajectory. The EKF, the controller, the AprilTag fuser and
the renders run asynchronously, so the order in which the draws are taken, and with it the run, depends on timing.
No run before this file (both arms of the M3 report among them) recorded a seed: the pinned simulator exposes none.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import secrets

from mission import MAX_SEED

PATCH = "stonefish_seed_v1"
UNSUPPORTED = "not_exposed_by_pinned_runner"
SEED_FILE = "simulator-seed.json"
ENVIRONMENT = "STONEFISH_SEED"
# One line per generator; ros2 launch prefixes the simulator's output with its process name.
LINE = re.compile(r"^\[stonefish_simulator-\d+\] stonefish_seed_v1: (Sensor|USBL) noise seed (\d+) from "
                  r"(STONEFISH_SEED|random_device)$", re.MULTILINE)


def valid_seed(value) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and 0 <= value <= MAX_SEED


def choose(mission: dict, draw=secrets.randbelow) -> dict:
    """{"exported": seed, "origin": "request" | "drawn"}: the mission's seed when given, else a fresh draw."""
    seed = mission.get("seed")
    if seed is None:
        return {"exported": draw(MAX_SEED + 1), "origin": "drawn"}
    if not valid_seed(seed):
        raise ValueError(f"seed must be null or an integer in [0, {MAX_SEED}]")
    return {"exported": seed, "origin": "request"}


def record(chosen: dict | None, launch_log: str) -> dict:
    """trial.json's simulator_seed. `chosen` is simulator-seed.json (None if the runner wrote none)."""
    printed = {generator: (int(seed), source) for generator, seed, source in LINE.findall(launch_log)}
    base = {"exported": chosen["exported"] if chosen else None, "origin": chosen["origin"] if chosen else None,
            "environment": ENVIRONMENT}
    if "Sensor" not in printed:
        return {"value": None, "support": UNSUPPORTED, **base, "applied": False}
    seed, source = printed["Sensor"]
    applied = chosen is not None and source == ENVIRONMENT and seed == chosen["exported"]
    return {"value": seed, "support": PATCH, **base, "source": source, "applied": applied,
            "generators": {generator: seed for generator, (seed, _) in sorted(printed.items())}}


def read_record(output: Path) -> dict:
    """record() from a trial output directory."""
    seed_file, log = output / SEED_FILE, output / "launch.log"
    chosen = json.loads(seed_file.read_text()) if seed_file.exists() else None
    return record(chosen, log.read_text(errors="replace") if log.exists() else "")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mission-profile-json", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    chosen = choose(json.loads(args.mission_profile_json.read_text()))
    target = args.output / SEED_FILE
    if target.exists():
        raise SystemExit(f"{target} exists; refusing overwrite")
    target.write_text(json.dumps(chosen, sort_keys=True) + "\n")
    print(chosen["exported"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
