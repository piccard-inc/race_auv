#!/usr/bin/env python3
"""M2 Phase R-A docking metrics for race-auv-docking/v1 trial directories (piccard-physical-ai #96).

The fixed metrics of the preregistration (piccard-experiments #87, "M2 preregistration: Phase R-A v0.1", as amended
by v0.2 of the same day), from a trial's trial.json, docking.json and telemetry.jsonl:
- per pose, from ground truth in the station dock-point frame: overshoot along the approach axis past the commanded
  stand-off (m, and % of the stand-off step), minimum dock-point distance, and the dwell-end stand-off / lateral /
  vertical / distance / orientation errors, the position errors decomposed into controller error and frame drift;
- per step (v0.2 settling, the default): consecutive poses with the same command are one step; on each axis the step
  changes by more than the band, the controller's own tracking error (its cg_link value minus the commanded pose, in its world_ned) settles
  when it stays within +/-0.05 m (+/-0.05 rad for yaw) of its final value, the mean of the last 10 s, for at least
  the last 10 s; settling time is the start of that final in-band run, from the step start; the final value is the
  steady-state error, reported on every axis. The first step (the dive from the surface) is not judged;
- v0.1 settling, kept as an option for the record (--settling-definition v0.1): per pose, +/-0.05 m of the commanded
  dock point on each station dock-point axis and +/-0.05 rad, from ground truth, entered for good at least 3 s
  before the pose end;
- per trial: thruster saturation fraction per thruster, front-camera tag visibility, fused range ratio, the
  controller-frame drift at each pose end, contacts, route completion and the exclusion rule;
- contacts in two classes (amendment v0.3): force-bearing events (normal force > 0, the meshes touch) and near-misses
  (normal force exactly 0: manifold points inside Bullet's contact margin, about 2 cm, no load; #100), each with its
  count, span, seconds with events and the ground-truth dock-point distance while they occur. A pass fails on either
  class. The maximum normal force is a lower bound: the monitors keep one manifold point per step (history 1);
- R-B contact metrics: the first force-bearing contact (time, speed and alignment), its persistence through the
  final dwell, the near-miss statistics and the maximum normal force (a lower bound);
- with --matrix: the R-A1 selection rule (pass, gates, ranking) over the trials given.

Frames and constants: the commanded pose (cg_link in the controller's world_ned) is mapped to the Stonefish world
through the controller frame the collector measured (docking.json controller_frame_in_ground_truth; the nominal
frame if it is missing), then to the AUV dock point with the runtime's own offsets (runtime/docking_metric.py).
Definitions are Piccard's, not lab-approved. Standard library only.

    race_m2_metrics.py DIR [DIR ...] --output-dir OUT [--matrix] [--selection-mission staged]
                       [--settling-definition v0.2|v0.1]
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[2]
RUNTIME = ROOT / "packages/simulation/race-auv-docking/runtime"
SCHEMA = "piccard.race-auv.m2-metrics/v1"
MATRIX_SCHEMA = "piccard.race-auv.m2-matrix/v1"
SETTLE_POSITION_M = 0.05
SETTLE_ANGLE_RAD = 0.05
SETTLE_DWELL_S = 3.0  # v0.1 only
STEADY_TAIL_S = 10.0  # v0.2: the final value is the mean of, and the band must hold over, the last 10 s of the step
SETTLING_DEFINITIONS = ("v0.2", "v0.1")
# v0.2 settling axes: the controller's cg_link tracking error in its world_ned. Docking x is lateral to the station,
# docking y the approach axis (error positive = short of the command, farther from the station, like stand-off error).
CONTROLLER_AXES = {"x": SETTLE_POSITION_M, "y": SETTLE_POSITION_M, "z": SETTLE_POSITION_M, "yaw": SETTLE_ANGLE_RAD}
GATED_AXES = ("x", "y")  # the knobs are the docking x/y gains; the non-settling gate applies to steps of these axes
COMMAND_FIELDS = ("x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad")
STEP_MIN_M = 0.05
PASS_MIN_DISTANCE_M = 0.9
SATURATION_GATE = 0.15
PERSIST_BIN_S, PERSIST_FRACTION = 1.0, 0.9
CONTACT_CLASSES = ("force_bearing", "near_miss")  # v0.3: normal force > 0, or exactly 0 (inside the contact margin)
FORCE_NOTE = ("lower bound: the scenario's contact monitors keep one manifold point per simulation step (history 1), "
              "so a loaded point can be dropped for an unloaded one")
FUSED_MATCH_S = 0.2
NOMINAL_FRAME = {"origin_m": [-3.8, 0.0, 0.0], "yaw_rad": -1.571}
CG_TO_NOSE = ((0.7, 0.0, 0.0), (math.pi, 0.0, 0.0))  # inverse of docking_metric.AUV_BASE_TO_CG_LINK
# Thruster force model of the pinned controller (race_auv da58963 race_auv_config/mvp_control_config/config_sim.yaml):
# command c on control/thruster/<name> produces force poly(c); allocation clamps each force to its limits.
THRUSTER_POLY = (0.0, 20.2169, 10.0265, 92.5762, -3.4585, -73.7285)
THRUSTER_LIMITS_N = {"sway_stern": 10.0, "heave_bow": 10.0, "heave_stern_port": 13.0, "heave_stern_stbd": 13.0,
                     "surge_port": 20.0, "surge_stbd": 20.0}
# The R-A1 ranking poses: settling on the step into the 1.5 m stand-off, dwell-end errors at the hold that follows
# (v0.2 section 4; for staged, approach_1p5m and hold_1p5m are one step, timed from approach_1p5m's start).
RANK_POSES = {"staged": ("approach_1p5m", "hold_1p5m"), "1step": ("approach", "hold_1p5m")}
KINDS = {"gt_auv", "gt_station", "dock", "thruster", "detections", "fused_dock", "contact", "value"}
CLAIM = ("Synthetic Stonefish simulation scored by ground truth; Piccard's metric definitions, not lab-approved; no "
         "physical docking, contact-mechanics or perception-accuracy claim.")


def _docking_metric():
    spec = importlib.util.spec_from_file_location("race_docking_metric", RUNTIME / "docking_metric.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


dm = _docking_metric()


def finite(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def norm(vector) -> float:
    return math.sqrt(sum(v * v for v in vector))


def rounded(value, digits=4):
    return round(value, digits) if finite(value) else value


# ----------------------------------------------------------------------------- loading
def trial_root(path: Path) -> Path:
    """A trial's output directory, or a downloaded artifact directory that holds it under output/."""
    for candidate in (path, path / "output"):
        if (candidate / "trial.json").is_file() and (candidate / "telemetry.jsonl").is_file():
            return candidate
    raise FileNotFoundError(f"{path}: no trial.json and telemetry.jsonl (or under output/)")


def load_trial(path: Path) -> dict:
    root = trial_root(path)
    trial = json.loads((root / "trial.json").read_text(encoding="utf-8"))
    docking = json.loads((root / "docking.json").read_text(encoding="utf-8")) if (root / "docking.json").is_file() else {}
    rows = []
    with (root / "telemetry.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            if row.get("kind") in KINDS and finite(row.get("t")):
                rows.append(row)
    rows.sort(key=lambda row: row["t"])
    context = trial.get("comparison_context") or {}
    label = context.get("trial_id") or trial.get("candidate_id") or path.name
    return {"path": str(path), "label": label, "trial": trial, "docking": docking, "rows": rows, "context": context}


def mission_forms(trial: dict) -> dict:
    """The mission's sha256 in each rendering a job context may have bound. "runtime" is trial.json's own hash of the
    bytes the runtime received; the submit path renders integral floats as integers (0.0 as 0), so it differs from
    the builder's. "builder" is the builder's numeric form (build_phase_r_campaign.py): the same mission with every
    pose coordinate and angle a float, serialized as the runtime does."""
    mission = trial.get("mission") or {}
    builder = {**mission, "poses": [{**pose, **{key: float(pose[key]) for key in COMMAND_FIELDS if finite(pose.get(key))}}
                                    for pose in mission.get("poses") or []]}
    text = json.dumps(builder, indent=2, sort_keys=True) + "\n"
    return {"runtime": trial.get("mission_sha256"), "builder": hashlib.sha256(text.encode()).hexdigest()}


def mission_binding(trial: dict, context: dict) -> tuple:
    """(matches, form) of the context's mission_sha256 against the trial's mission; (None, None) without one."""
    bound = context.get("mission_sha256")
    if not bound:
        return None, None
    form = next((name for name, digest in mission_forms(trial).items() if digest == bound), None)
    return form is not None, form


# ----------------------------------------------------------------------------- geometry
def commanded_dock(pose: dict, frame: dict):
    """The AUV dock-point pose (Stonefish world) the commanded cg_link pose puts it at."""
    yaw = frame["yaw_rad"]
    position = [a + b for a, b in zip(dm.apply(dm.rpy_matrix(0.0, 0.0, yaw), [pose["x_m"], pose["y_m"], pose["z_m"]]),
                                      frame["origin_m"])]
    rotation = dm.rpy_matrix(pose["roll_rad"], pose["pitch_rad"], pose["yaw_rad"] + yaw)
    return dm.compose(dm.compose((position, rotation), CG_TO_NOSE), dm.AUV_BASE_TO_DOCK)


def samples(rows: list[dict], command) -> list[dict]:
    """Per ground-truth AUV sample: dock-point distance, errors against the commanded dock point in the station
    dock-point frame (stand-off error positive = farther from the station than commanded), speed."""
    station, result = None, []
    for row in rows:
        if row["kind"] == "gt_station":
            station = dm.compose(dm.odometry_pose(row), dm.STATION_BASE_TO_DOCK)
        elif row["kind"] == "gt_auv" and station is not None:
            auv = dm.compose(dm.odometry_pose(row), dm.AUV_BASE_TO_DOCK)
            to_station = dm.transpose(station[1])
            rel = dm.apply(to_station, [auv[0][i] - station[0][i] for i in range(3)])
            rel_cmd = dm.apply(to_station, [command[0][i] - station[0][i] for i in range(3)])
            result.append({"t": row["t"], "distance_m": norm([auv[0][i] - station[0][i] for i in range(3)]),
                           "standoff_m": -rel[0], "commanded_standoff_m": -rel_cmd[0],
                           "standoff_error_m": rel_cmd[0] - rel[0], "lateral_error_m": rel[1] - rel_cmd[1],
                           "vertical_error_m": rel[2] - rel_cmd[2],
                           "orientation_error_rad": dm.rotation_angle(dm.matmul(dm.transpose(command[1]), auv[1])),
                           "relative_position_in_station_dock_m": rel, "commanded_relative_m": rel_cmd,
                           "orientation_error_to_station_rad": dm.rotation_angle(dm.matmul(to_station, auv[1])),
                           "speed_mps": norm(row.get("linear_mps") or [0.0, 0.0, 0.0])})
    return result


def in_band(sample: dict) -> bool:
    return (abs(sample["standoff_error_m"]) <= SETTLE_POSITION_M and abs(sample["lateral_error_m"]) <= SETTLE_POSITION_M
            and abs(sample["vertical_error_m"]) <= SETTLE_POSITION_M
            and sample["orientation_error_rad"] <= SETTLE_ANGLE_RAD)


def settling(window: list[dict], start: float) -> dict:
    last_out = max((i for i, sample in enumerate(window) if not in_band(sample)), default=-1)
    if last_out + 1 < len(window) and window[-1]["t"] - window[last_out + 1]["t"] >= SETTLE_DWELL_S:
        return {"settled": True, "settling_time_s": window[last_out + 1]["t"] - start}
    return {"settled": False, "settling_time_s": None,
            "reason": "never inside the band" if last_out == len(window) - 1 else "in band for less than 3 s at pose end"}


def command(pose: dict) -> tuple:
    return tuple(pose[key] for key in COMMAND_FIELDS)


def axis_error(axis: str, value: dict, pose: dict) -> float:
    if axis == "x":
        return value["x"] - pose["x_m"]
    if axis == "y":
        return pose["y_m"] - value["y"]
    if axis == "z":
        return value["z"] - pose["z_m"]
    return dm.wrap(value["yaw"] - pose["yaw_rad"])


def steady_settling(series: list[tuple[float, float]], start: float, end: float, band: float) -> dict:
    """v0.2 definition 1 on one axis: series is (t, tracking error) over the step."""
    tail = [error for t, error in series if t >= end - STEADY_TAIL_S]
    if not tail:
        return {"settled": False, "settling_time_s": None, "steady_state_error": None,
                "reason": f"no controller samples in the last {STEADY_TAIL_S:g} s"}
    final = sum(tail) / len(tail)
    entered = None
    for t, error in series:
        if abs(error - final) > band:
            entered = None
        elif entered is None:
            entered = t
    settled = all(abs(error - final) <= band for error in tail)
    return {"settled": settled, "settling_time_s": rounded(entered - start, 3) if settled else None,
            "steady_state_error": rounded(final),
            **({} if settled else {"reason": f"not within +/-{band:g} of its final value over the last "
                                             f"{STEADY_TAIL_S:g} s"})}


def step_settling(rows: list[dict], poses: list[dict], docked: dict) -> list[dict]:
    """v0.2: consecutive poses with the same command are one step, judged on the axes it changes."""
    groups = []
    for pose in poses:
        if groups and command(groups[-1][-1]) == command(pose):
            groups[-1].append(pose)
        else:
            groups.append([pose])
    values = [r for r in rows if r["kind"] == "value" and all(finite(r.get(k)) for k in ("x", "y", "z", "yaw"))]
    result, previous = [], None
    for index, group in enumerate(groups):
        target = group[0]
        # an axis is stepped when its command moves by more than its band (M1's 1 mm lateral change is not a step)
        stepped = [] if previous is None else [
            axis for axis, field in (("x", "x_m"), ("y", "y_m"), ("z", "z_m"), ("yaw", "yaw_rad"))
            if abs(target[field] - previous[field]) > CONTROLLER_AXES[axis]]
        previous = target
        start = docked.get(target["label"], {}).get("start_t")
        end = docked.get(group[-1]["label"], {}).get("end_t")
        item = {"index": index, "poses": [pose["label"] for pose in group], "stepped_axes": stepped,
                "gated": any(axis in GATED_AXES for axis in stepped), "start_t": start, "end_t": end}
        if index == 0:
            result.append({**item, "judged": False, "settled": None, "settling_time_s": None,
                           "reason": "first step (the dive from the surface): not judged (v0.2 section 3)"})
            continue
        if start is None or end is None:
            result.append({**item, "judged": False, "settled": None, "settling_time_s": None, "reason": "not reached"})
            continue
        window = [r for r in values if start <= r["t"] <= end]
        axes = {axis: steady_settling([(r["t"], axis_error(axis, r, target)) for r in window], start, end, band)
                for axis, band in CONTROLLER_AXES.items()}
        judged = [axes[axis] for axis in stepped]
        settled = bool(judged) and all(a["settled"] for a in judged)
        result.append({**item, "judged": bool(judged), "settled": settled if judged else None,
                       "settling_time_s": max(a["settling_time_s"] for a in judged) if settled else None,
                       "axes": axes})
    return result


def decomposition(pose: dict, docked: dict, last: dict, station_rotation) -> dict | None:
    """Dwell-end ground-truth position error (docking.json) split into controller error and frame drift, all in the
    station dock-point frame as (stand-off, lateral, vertical). Drift is ground truth minus the controller's cg_link
    mapped through the initial alignment, so ground truth = controller + drift by construction."""
    rel = (docked.get("at_end") or {}).get("relative_position_in_station_dock_m")
    drift = (docked.get("controller_frame_drift_at_end") or {}).get("position_m")
    if not rel or not drift or station_rotation is None:
        return None
    rel_cmd = last["commanded_relative_m"]
    drift_rel = dm.apply(dm.transpose(station_rotation), drift)
    total = [rel_cmd[0] - rel[0], rel[1] - rel_cmd[1], rel[2] - rel_cmd[2]]
    frame = [-drift_rel[0], drift_rel[1], drift_rel[2]]
    return {"axes": ["standoff_m", "lateral_m", "vertical_m"], "ground_truth": [rounded(v) for v in total],
            "controller": [rounded(a - b) for a, b in zip(total, frame)], "frame_drift": [rounded(v) for v in frame]}


def pose_metrics(rows: list[dict], pose: dict, docked: dict, frame: dict, previous_standoff) -> dict:
    start, end = docked.get("start_t"), docked.get("end_t")
    item = {"label": pose["label"], "start_t": start, "end_t": end, "status": "not_reached"}
    if start is None or end is None:
        return item
    ground_truth = [r for r in rows if r["kind"] in ("gt_station", "gt_auv") and r["t"] <= end]  # time order
    station = next((r for r in reversed(ground_truth) if r["kind"] == "gt_station"), None)
    station_rotation = dm.compose(dm.odometry_pose(station), dm.STATION_BASE_TO_DOCK)[1] if station else None
    window = [s for s in samples(ground_truth, commanded_dock(pose, frame)) if start <= s["t"]]
    if not window:
        return {**item, "status": "no_ground_truth"}
    commanded = window[0]["commanded_standoff_m"]
    step = previous_standoff - commanded if finite(previous_standoff) else None
    overshoot = max(0.0, -min(s["standoff_error_m"] for s in window))
    last = window[-1]
    closest = min(window, key=lambda s: s["distance_m"])
    at_end = docked.get("at_end") or {}
    drift = docked.get("controller_frame_drift_at_end") or {}
    return {**item, "status": "observed", "samples": len(window), "commanded_standoff_m": rounded(commanded),
            "standoff_step_m": rounded(step),
            "overshoot_m": rounded(overshoot),
            "overshoot_percent_of_step": rounded(100.0 * overshoot / step, 2) if step and step >= STEP_MIN_M else None,
            "minimum_distance_m": rounded(closest["distance_m"]), "minimum_distance_t": rounded(closest["t"], 3),
            "v0_1_settling": {key: rounded(value, 3) if key == "settling_time_s" else value
                              for key, value in settling(window, start).items()},
            "dwell_end": {"t": rounded(last["t"], 3), "distance_m": rounded(last["distance_m"]),
                          "standoff_error_m": rounded(last["standoff_error_m"]),
                          "lateral_error_m": rounded(last["lateral_error_m"]),
                          "vertical_error_m": rounded(last["vertical_error_m"]),
                          "distance_error_m": rounded(norm([last["standoff_error_m"], last["lateral_error_m"],
                                                           last["vertical_error_m"]])),
                          "orientation_error_rad": rounded(last["orientation_error_rad"]),
                          "speed_mps": rounded(last["speed_mps"]),
                          "fused_range_ratio": rounded((at_end.get("perception") or {}).get("range_ratio")),
                          "relative_position_in_station_dock_m": [rounded(v) for v in
                                                                  last["relative_position_in_station_dock_m"]],
                          "decomposition": decomposition(pose, docked, last, station_rotation)},
            "controller_frame_drift_at_end": {"horizontal_m": rounded(drift.get("horizontal_m")),
                                              "yaw_rad": rounded(drift.get("yaw_rad"))} if drift else None}


# ----------------------------------------------------------------------------- per trial
def thruster_force(command: float) -> float:
    return sum(k * command ** i for i, k in enumerate(THRUSTER_POLY))


def saturation(rows: list[dict], start, end) -> dict:
    counts = {}
    for row in rows:
        if row["kind"] != "thruster" or not start <= row["t"] <= end or not finite(row.get("data")):
            continue
        name = (row.get("topic") or "").rsplit("/", 1)[-1]
        if name not in THRUSTER_LIMITS_N:
            continue
        total, hits = counts.get(name, (0, 0))
        at_limit = abs(thruster_force(row["data"])) >= THRUSTER_LIMITS_N[name] * (1 - 1e-4)
        counts[name] = (total + 1, hits + at_limit)
    return {name: {"samples": total, "at_limit_fraction": rounded(hits / total)} for name, (total, hits) in sorted(counts.items())}


def tag_visibility(rows: list[dict], start, end) -> dict:
    """Front camera: distinct source images the detector processed, and those with at least one tag."""
    images, with_tags = set(), set()
    for row in rows:
        if row["kind"] == "detections" and row.get("camera") == "cam_front" and start <= row["t"] <= end \
                and finite(row.get("stamp")):
            images.add(row["stamp"])
            if row.get("count", 0) > 0:
                with_tags.add(row["stamp"])
    return {"images": len(images), "images_with_tags": len(with_tags),
            "fraction": rounded(len(with_tags) / len(images)) if images else None,
            "basis": "distinct source-image stamps in the detector output (it republishes on a timer)"}


def fused_range(rows: list[dict], start, end) -> dict:
    """|fused station dock point| / |ground-truth station dock point|, both in the AUV base_link."""
    ratios, truth = [], None
    for row in rows:
        if row["kind"] == "dock":
            truth = row
        elif row["kind"] == "fused_dock" and start <= row["t"] <= end and truth and row["t"] - truth["t"] <= FUSED_MATCH_S:
            actual = norm(truth.get("station_dock_in_auv_base_link_m") or [0.0])
            if actual > 1e-9:
                ratios.append(norm(row["position_m"]) / actual)
    ratios.sort()
    return {"samples": len(ratios), "median": rounded(statistics.median(ratios)) if ratios else None,
            "p10": rounded(ratios[int(0.1 * len(ratios))]) if ratios else None,
            "p90": rounded(ratios[int(0.9 * len(ratios))]) if ratios else None}


def contact_class(row: dict) -> str:
    return "force_bearing" if (row.get("normal_force_n") or 0.0) > 0.0 else "near_miss"


def contact_events(rows: list[dict]) -> list[dict]:
    """Contact rows in time order, each with the latest ground-truth dock-point distance at or before it."""
    distance, events = None, []
    for row in rows:
        if row["kind"] == "dock" and finite(row.get("distance_m")):
            distance = row["distance_m"]
        elif row["kind"] == "contact":
            events.append({**row, "class": contact_class(row), "dock_point_distance_m": distance})
    return events


def event_summary(events: list[dict]) -> dict | None:
    if not events:
        return None
    distances = [e["dock_point_distance_m"] for e in events if finite(e["dock_point_distance_m"])]
    return {"events": len(events), "first_t": rounded(events[0]["t"], 3), "last_t": rounded(events[-1]["t"], 3),
            "seconds_with_events": len({int(e["t"] // PERSIST_BIN_S) for e in events}),
            "min_dock_point_distance_m": rounded(min(distances)) if distances else None,
            "max_normal_force_n": rounded(max(e.get("normal_force_n") or 0.0 for e in events))}


def contacts(rows: list[dict]) -> dict:
    """Per contact pair: all events, and the force-bearing and near-miss classes (v0.3)."""
    by_pair = {}
    for event in contact_events(rows):
        by_pair.setdefault(event.get("contact"), []).append(event)
    return {pair: {**event_summary(events),
                   **{name: event_summary([e for e in events if e["class"] == name]) for name in CONTACT_CLASSES}}
            for pair, events in by_pair.items()}


def contact_metrics(rows: list[dict], docked_poses: list[dict]) -> dict:
    """R-B: the first force-bearing AUV-station contact, its persistence through the final pose's dwell, and the
    near-miss statistics (v0.3)."""
    events = [e for e in contact_events(rows) if e.get("contact") == "auv_station"]
    if not events:
        return {"contact": False}
    final = docked_poses[-1] if docked_poses else {}
    loaded = [e for e in events if e["class"] == "force_bearing"]
    result = {"contact": True, "events": len(events), "near_miss": event_summary([e for e in events if e["class"] == "near_miss"]),
              "force_bearing": None, "max_normal_force_n": rounded(max(e.get("normal_force_n") or 0.0 for e in events)),
              "max_normal_force_note": FORCE_NOTE}
    if not loaded:
        return result
    first = loaded[0]
    # alignment at first contact is against the station dock point itself, so the commanded pose does not enter
    state = samples([r for r in rows if r["kind"] in ("gt_station", "gt_auv") and r["t"] <= first["t"]],
                    ((0.0, 0.0, 0.0), dm.rpy_matrix(0.0, 0.0, 0.0)))
    at = state[-1] if state else None
    persistence = None
    if final.get("start_t") is not None and final.get("end_t") is not None and first["t"] < final["end_t"]:
        # from first force-bearing contact (or the final pose's start, if it came earlier) to the end of its dwell
        start, end = max(final["start_t"], first["t"]), final["end_t"]
        bins = max(1, int((end - start) // PERSIST_BIN_S))
        touched = {int((e["t"] - start) // PERSIST_BIN_S) for e in loaded if start <= e["t"] < start + bins * PERSIST_BIN_S}
        persistence = {"pose": final.get("label"), "from_t": rounded(start, 3), "bins": bins,
                       "bins_with_contact_fraction": rounded(len(touched) / bins),
                       "persists": len(touched) / bins >= PERSIST_FRACTION, "bin_s": PERSIST_BIN_S}
    result["force_bearing"] = {
        **event_summary(loaded), "first_contact_t": rounded(first["t"], 3),
        "first_contact_after_final_pose_start_s": rounded(first["t"] - final["start_t"], 3)
        if final.get("start_t") is not None else None,
        "speed_at_first_contact_mps": rounded(at["speed_mps"]) if at else None,
        "alignment_at_first_contact": {
            "relative_position_in_station_dock_m": [rounded(v) for v in at["relative_position_in_station_dock_m"]],
            "distance_m": rounded(at["distance_m"]),
            "orientation_error_rad": rounded(at["orientation_error_to_station_rad"])} if at else None,
        "persistence": persistence}
    return result


def analyze(path: Path) -> dict:
    loaded = load_trial(path)
    trial, docking, rows, context = loaded["trial"], loaded["docking"], loaded["rows"], loaded["context"]
    mission = trial.get("mission") or {}
    frame = docking.get("controller_frame_in_ground_truth") or NOMINAL_FRAME
    docked = {p.get("label"): p for p in docking.get("poses") or []}
    start, end = trial.get("mission_start_t"), trial.get("mission_end_t")
    window = (start, end) if finite(start) and finite(end) else (-math.inf, math.inf)
    first_station = next((r for r in rows if r["kind"] == "gt_station"), None)
    first_auv = next((r for r in rows if r["kind"] == "gt_auv"), None)
    previous = None
    if first_station and first_auv:  # the start stand-off, for the first pose's step
        station = dm.compose(dm.odometry_pose(first_station), dm.STATION_BASE_TO_DOCK)
        auv = dm.compose(dm.odometry_pose(first_auv), dm.AUV_BASE_TO_DOCK)
        previous = -dm.apply(dm.transpose(station[1]), [auv[0][i] - station[0][i] for i in range(3)])[0]
    poses = []
    for pose in mission.get("poses") or []:
        item = pose_metrics(rows, pose, docked.get(pose["label"], {}), frame, previous)
        poses.append(item)
        if finite(item.get("commanded_standoff_m")):
            previous = item["commanded_standoff_m"]
    audits = [trial.get(name) or {} for name in ("consumption_audit_before_mission", "consumption_audit_at_end")]
    audit_problems = sorted({p for audit in audits for p in audit.get("problems") or []})
    route_completed = (trial.get("status") == "completed" and trial.get("stop_reason") == "mission_complete"
                       and bool(poses) and all(p["status"] == "observed" for p in poses))
    exclusions = []
    if audit_problems:
        exclusions.append("audit problem: " + ", ".join(audit_problems))
    if "nonfinite" in str(trial.get("stop_reason", "")):
        exclusions.append(f"non-finite critical stream stop ({trial.get('stop_reason')})")
    if not route_completed:
        exclusions.append(f"route incomplete ({trial.get('status')}, {trial.get('stop_reason')})")
    touched = contacts(rows)
    matches, form = mission_binding(trial, context)
    return {
        "schema": SCHEMA, "claim_boundary": CLAIM, "label": loaded["label"], "path": loaded["path"],
        "context": {key: context.get(key) for key in ("trial_id", "campaign_id", "stage", "gain_set", "mission_key",
                                                        "role", "repeat", "mission_sha256")},
        "mission_id": mission.get("mission_id"), "mission_sha256": trial.get("mission_sha256"),
        "mission_matches_context": matches, "mission_hash_form": form,
        "apriltag_tag_size": mission.get("apriltag_tag_size"),
        "status": trial.get("status"), "stop_reason": trial.get("stop_reason"),
        "time_basis": "collector_monotonic_receive_seconds",
        "controller_frame": {"source": "docking.json" if docking.get("controller_frame_in_ground_truth") else "nominal",
                             **{key: frame[key] for key in ("origin_m", "yaw_rad")}},
        "poses": poses,
        "steps": step_settling(rows, mission.get("poses") or [], docked),
        "minimum_distance_m": rounded(min((p["minimum_distance_m"] for p in poses if p["status"] == "observed"),
                                          default=None)),
        "thruster_saturation": saturation(rows, *window),
        "tag_visibility_cam_front": tag_visibility(rows, *window),
        "fused_range_ratio": fused_range(rows, *window),
        "contacts": touched,
        "contact_metrics": contact_metrics(rows, poses),
        "route_completed": route_completed,
        "audit_problems": audit_problems,
        "excluded": bool(exclusions), "exclusion_reasons": exclusions,
        "definitions": {"settling": f"v0.2 (steps): per stepped axis, the controller's tracking error within "
                                    f"+/-{SETTLE_POSITION_M} m / +/-{SETTLE_ANGLE_RAD} rad of its final value (mean of "
                                    f"the last {STEADY_TAIL_S:g} s) over at least the last {STEADY_TAIL_S:g} s; time "
                                    "from the step start to the start of the final in-band run; same-command poses "
                                    "are one step; the first step (dive) is not judged",
                        "settling_v0_1": f"(poses, v0_1_settling) +/-{SETTLE_POSITION_M} m on each station dock-point "
                                         f"axis against the commanded dock point and +/-{SETTLE_ANGLE_RAD} rad, from "
                                         f"ground truth, entered for good at least {SETTLE_DWELL_S:g} s before the pose "
                                         "end",
                        "error_reference": "dock points as runtime/docking_metric.py (the same offsets as "
                                           "contact-margin.json). The stand-off, lateral, vertical and distance "
                                           "*errors* are against the COMMANDED dock point: the commanded cg_link pose "
                                           "mapped through the measured controller frame (docking.json "
                                           "controller_frame_in_ground_truth, including its vertical origin, the "
                                           "controller's depth offset at alignment). distance_m, minimum_distance_m "
                                           "and relative_position_in_station_dock_m (station dock frame, Rx(pi) of the "
                                           "station: +z is shallower) are against the STATION dock point. The two "
                                           "differ by the frame offset: in M2 R-B the dock points were 2.1-2.2 cm apart "
                                           "vertically while the vertical errors were +0.004 (N) / -0.016 m (p 10)",
                        "decomposition": "dwell-end ground-truth error = controller error + frame drift, station "
                                         "dock-point frame (stand-off, lateral, vertical)",
                        "overshoot": "how far the dock point came closer to the station than the commanded stand-off",
                        "saturation": "fraction of mission-window thruster commands whose modelled force is at the "
                                      "thruster's limit (pinned config_sim.yaml polynomial and limits)",
                        "exclusion": "audit problem, non-finite critical stream stop or route incompletion: re-run once",
                        "contact_classes": "force_bearing: contact events with normal force > 0 (the meshes touch); "
                                           "near_miss: events with normal force exactly 0 (manifold points inside "
                                           "Bullet's contact margin, about 2 cm, no load); dock-point distances are ground "
                                           "truth at the events (amendment v0.3, #100)",
                        "max_normal_force": FORCE_NOTE},
    }


# ----------------------------------------------------------------------------- selection
def settling_view(report: dict, step_pose: str, definition: str) -> tuple[list[str], object]:
    """(non-settling poses for the gate, settling time on the step pose) under a settling definition."""
    if definition == "v0.1":  # every pose, ground-truth band
        poses = {p["label"]: p for p in report["poses"]}
        return ([p["label"] for p in report["poses"] if not (p.get("v0_1_settling") or {}).get("settled")],
                ((poses.get(step_pose) or {}).get("v0_1_settling") or {}).get("settling_time_s"))
    steps = report["steps"]
    non_settling = [s["poses"][0] for s in steps if s["gated"] and s["judged"] and not s["settled"]]
    non_settling += [s["poses"][0] for s in steps if s["gated"] and not s["judged"] and s["index"] > 0]
    step = next((s for s in steps if s["poses"][0] == step_pose), {})
    return non_settling, step.get("settling_time_s")


def selection(reports: list[dict], mission_key: str, definition: str = "v0.2") -> dict:
    """The R-A1 rule: pass = no AUV-station contact and minimum distance >= 0.9 m on every pose; gates = saturation
    <= 0.15 on every thruster, no non-settling step of docking x or y (v0.2; every pose under v0.1), route completed;
    passes ranked by settling on the step into the 1.5 m stand-off, then dwell-end distance error and lateral error
    at the hold."""
    step_pose, hold_pose = RANK_POSES.get(mission_key, RANK_POSES["staged"])
    candidates, excluded, rows = [], [], []
    for report in reports:
        context = report["context"]
        if context.get("mission_key") not in (None, mission_key) or context.get("stage") not in (None, "a1"):
            continue
        if report["excluded"]:
            excluded.append({"label": report["label"], "reasons": report["exclusion_reasons"]})
            continue
        poses = {p["label"]: p for p in report["poses"]}
        station = report["contacts"].get("auv_station") or {}
        station_contact = bool(station)  # v0.3: either class fails the pass
        min_ok = all(p.get("minimum_distance_m") is not None and p["minimum_distance_m"] >= PASS_MIN_DISTANCE_M
                     for p in report["poses"])
        saturation_ok = all(v["at_limit_fraction"] <= SATURATION_GATE for v in report["thruster_saturation"].values())
        non_settling, settling_time = settling_view(report, step_pose, definition)
        hold = poses.get(hold_pose, {})
        row = {"label": report["label"], "gain_set": context.get("gain_set"), "repeat": context.get("repeat"),
               "pass": not station_contact and min_ok, "station_contact": station_contact,
               "station_contact_class": ("force_bearing" if station.get("force_bearing") else "near_miss")
               if station_contact else None,
               "minimum_distance_m": report["minimum_distance_m"],
               "gates": {"saturation_ok": saturation_ok, "non_settling_poses": non_settling,
                         "route_completed": report["route_completed"]},
               "rank_keys": {"settling_time_s": settling_time,
                             "dwell_end_distance_error_m": (hold.get("dwell_end") or {}).get("distance_error_m"),
                             "dwell_end_abs_lateral_error_m": abs((hold.get("dwell_end") or {}).get("lateral_error_m")
                                                                  or 0.0) if hold.get("dwell_end") else None}}
        row["eligible"] = row["pass"] and saturation_ok and not non_settling and report["route_completed"]
        rows.append(row)
        candidates.append(row)
    ranked = sorted((r for r in candidates if r["eligible"]),
                    key=lambda r: tuple(r["rank_keys"][k] if finite(r["rank_keys"][k]) else math.inf
                                        for k in ("settling_time_s", "dwell_end_distance_error_m",
                                                  "dwell_end_abs_lateral_error_m")))
    for rank, row in enumerate(ranked, 1):
        row["rank"] = rank
    nominal = [r for r in candidates if r["gain_set"] == "n"]
    spread = None
    if len(nominal) >= 2:  # how far two identical N trials differ on each ranking key
        spread = {}
        for key in ("settling_time_s", "dwell_end_distance_error_m", "dwell_end_abs_lateral_error_m"):
            values = [r["rank_keys"][key] for r in nominal]
            spread[key] = rounded(max(values) - min(values)) if all(finite(v) for v in values) else None
    best = ranked[0] if ranked else None
    gate = "no non-settling step of docking x or y" if definition == "v0.2" else "no non-settling pose"
    return {"mission_key": mission_key, "step_pose": step_pose, "hold_pose": hold_pose,
            "settling_definition": definition,
            "rule": {"pass": f"no AUV-station contact of either class (force-bearing or near-miss) and minimum "
                             f"dock-point distance >= {PASS_MIN_DISTANCE_M} m on every pose",
                     "gates": f"saturation <= {SATURATION_GATE} on every thruster, {gate}, route completed",
                     "ranking": "settling time on the step pose, then dwell-end distance error, then |lateral error| "
                                "at the hold pose"},
            "trials": rows, "excluded_rerun_once": excluded,
            "best": {"label": best["label"], "gain_set": best["gain_set"]} if best else None,
            "nominal_repeat_spread": spread}


# ----------------------------------------------------------------------------- output
def fmt(value, digits=3) -> str:
    return "—" if value is None else (f"{value:.{digits}f}" if isinstance(value, float) else str(value))


def trial_markdown(report: dict) -> str:
    binding = {None: "no context hash", True: f"matches the context ({report['mission_hash_form']} form)",
               False: "DOES NOT match the context"}[report["mission_matches_context"]]
    lines = [f"# {report['label']}", "", f"{report['status']} / {report['stop_reason']}; mission "
             f"`{report['mission_id']}` ({binding}); excluded: {report['excluded']} "
             f"{report['exclusion_reasons'] or ''}", "",
             "Ground truth, station dock-point frame. Dwell-end errors as ground truth = controller + frame drift "
             "(stand-off / lateral), m.", "",
             "| Pose | Stand-off cmd m | Step m | Overshoot m | % step | Min distance m | End distance m "
             "| End dist err m | End stand-off err | End lateral err | End vertical m | End orient rad | Drift m "
             "| v0.1 settling s |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for p in report["poses"]:
        end = p.get("dwell_end") or {}
        drift = p.get("controller_frame_drift_at_end") or {}
        split = end.get("decomposition")

        def parts(i):
            return (f"{fmt(split['ground_truth'][i])} = {fmt(split['controller'][i])} + {fmt(split['frame_drift'][i])}"
                    if split else fmt(end.get(("standoff_error_m", "lateral_error_m")[i])))
        lines.append(f"| {p['label']} | {fmt(p.get('commanded_standoff_m'))} | {fmt(p.get('standoff_step_m'))} "
                     f"| {fmt(p.get('overshoot_m'))} | {fmt(p.get('overshoot_percent_of_step'), 1)} "
                     f"| {fmt(p.get('minimum_distance_m'))} | {fmt(end.get('distance_m'))} "
                     f"| {fmt(end.get('distance_error_m'))} | {parts(0)} | {parts(1)} "
                     f"| {fmt(end.get('vertical_error_m'))} | {fmt(end.get('orientation_error_rad'))} "
                     f"| {fmt(drift.get('horizontal_m'))} "
                     f"| {fmt((p.get('v0_1_settling') or {}).get('settling_time_s'), 1)} |")
    lines += ["", "Steps, v0.2 settling on the controller's tracking error (steady-state error per axis: x lateral, "
              "y short of the command, z, yaw).", "",
              "| Step | Poses | Stepped axes | Settled | Settling s | Steady x m | Steady y m | Steady z m | Steady yaw rad |",
              "|---|---|---|---|---|---|---|---|---|"]
    for step in report["steps"]:
        axes = step.get("axes") or {}
        lines.append(f"| {step['index']} | {', '.join(step['poses'])} | {', '.join(step['stepped_axes']) or '—'} "
                     f"| {fmt(step['settled']) if step['judged'] else 'not judged'} "
                     f"| {fmt(step['settling_time_s'], 1)} "
                     + "".join(f"| {fmt((axes.get(a) or {}).get('steady_state_error'))} " for a in CONTROLLER_AXES) + "|")
    sat = ", ".join(f"{k} {v['at_limit_fraction']:.3f}" for k, v in report["thruster_saturation"].items())
    tags = report["tag_visibility_cam_front"]
    lines += ["", f"Saturation (at-limit fraction): {sat}", "",
              f"cam_front tag visibility: {fmt(tags['fraction'])} ({tags['images_with_tags']}/{tags['images']} images); "
              f"fused range ratio median {fmt(report['fused_range_ratio']['median'])}", ""]
    if report["contacts"]:
        lines += ["| Contact pair | Class | Events | First–last t s | Seconds with events | Min dock-point distance m "
                  "| Max normal force N (lower bound) |", "|---|---|---|---|---|---|---|"]
        for pair, entry in sorted(report["contacts"].items()):
            for name in CONTACT_CLASSES:
                c = entry.get(name)
                if c:
                    lines.append(f"| {pair} | {name} | {c['events']} | {fmt(c['first_t'], 1)}–{fmt(c['last_t'], 1)} "
                                 f"| {c['seconds_with_events']} | {fmt(c['min_dock_point_distance_m'])} "
                                 f"| {fmt(c['max_normal_force_n'], 1)} |")
        lines.append("")
    else:
        lines += ["Contacts: none.", ""]
    return "\n".join(lines) + "\n"


def matrix_markdown(result: dict) -> str:
    lines = [f"# Phase R-A selection on `{result['mission_key']}` (settling {result['settling_definition']})", "",
             f"Rule: pass = {result['rule']['pass']}; gates = "
             f"{result['rule']['gates']}; ranking = {result['rule']['ranking']}.", "",
             "| Trial | Set | Pass | Station contact | Min distance m | Saturation ok | Non-settling | Route | Settling s "
             "| End dist err m | |Lateral| m | Rank |", "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in result["trials"]:
        k = r["rank_keys"]
        lines.append(f"| {r['label']} | {r['gain_set']} | {r['pass']} | {r.get('station_contact_class') or '—'} "
                     f"| {fmt(r['minimum_distance_m'])} "
                     f"| {r['gates']['saturation_ok']} | {', '.join(r['gates']['non_settling_poses']) or '—'} "
                     f"| {r['gates']['route_completed']} | {fmt(k['settling_time_s'], 1)} "
                     f"| {fmt(k['dwell_end_distance_error_m'])} | {fmt(k['dwell_end_abs_lateral_error_m'])} "
                     f"| {r.get('rank', '—')} |")
    lines += ["", f"Best: {result['best']['label'] + ' (' + result['best']['gain_set'] + ')' if result['best'] else 'none'}; "
              f"N repeat spread: {json.dumps(result['nominal_repeat_spread'])}; excluded (re-run once): "
              f"{json.dumps(result['excluded_rerun_once'])}", ""]
    return "\n".join(lines) + "\n"


def write(output: Path, stem: str, payload: dict, markdown: str) -> None:
    (output / f"{stem}.json").write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n")
    (output / f"{stem}.md").write_text(markdown)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("trials", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--matrix", action="store_true", help="also evaluate the R-A1 selection rule")
    parser.add_argument("--selection-mission", default="staged", choices=sorted(RANK_POSES))
    parser.add_argument("--settling-definition", default="v0.2", choices=SETTLING_DEFINITIONS,
                        help="v0.2 (amendment of 2026-09-28, default) or the v0.1 ground-truth band, for the record")
    args = parser.parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    try:
        reports = [analyze(path) for path in args.trials]
    except (FileNotFoundError, ValueError, KeyError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    for report in reports:
        write(args.output_dir, f"{report['label']}.metrics", report, trial_markdown(report))
    if args.matrix:
        result = {"schema": MATRIX_SCHEMA, "claim_boundary": CLAIM,
                  **selection(reports, args.selection_mission, args.settling_definition)}
        write(args.output_dir, "matrix", result, matrix_markdown(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
