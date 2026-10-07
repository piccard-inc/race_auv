#!/usr/bin/env python3
"""M3 dock-criterion metrics for race-auv-docking trial directories (piccard-physical-ai #108, protocol
piccard-experiments #88).

From a trial's trial.json, docking.json and telemetry.jsonl, scored by ground truth (the collector's dock rows, which
use the dock-point offsets of runtime/docking_metric.py):
- gap: the AUV dock point's distance to the station dock point, as a 1 s series over the mission, its minimum, and the
  first time it enters <= 0.02 m and <= 0.01 m (closure at each threshold), also as the time from the mission start;
- speed at closure: the AUV dock point's speed relative to the station dock point, a central finite difference of the
  dock rows' relative position over +/-0.5 s, at each closure time, and its maximum over the final approach (from the
  last crossing of the 0.3 m stand-off to closure);
- hold: over hold_s (default 30 s) after each closure, the fraction of dock rows within the threshold, the maximum and
  mean gap; docked means closed and every dock row of the whole hold within the threshold, the hold observed (inside
  the mission window, no dock-row gap over 1 s);
- load: AUV-station contact events during each hold, in the v0.3 classes, with the maximum normal force (a lower bound,
  history 1) and the fraction of 0.5 s bins holding a force-bearing event;
- coupling (with --meshes, the pinned world_of_stonefish collision meshes): whole-mesh and couplink separation at
  the minimum gap, at closure and each second of the hold, placed as contact_margin_check_v1_1.py places them
  (imported, not copied);
- vertical: z_W(AUV dock point) - z_W(station dock point) (world z down: negative = AUV shallower) at the minimum gap,
  at closure and over the hold;
- planner provenance: the trial's planner block and the consumption audits' planner_inputs result, with its timeline
  (handover, stage starts, stale intervals, set-point steps);
- dock criterion with speed: per threshold, speed_ok (speed at closure <= --speed-limit-mps, default 0.1) and
  docked_at_speed (docked and speed_ok);
- dock criterion at clearance (protocol v1.3, with --clearance-tol-m > 0): per threshold, the same closure, speed,
  hold, load and coupling with the horizontal gap within the threshold and the vertical offset within
  --clearance-tol-m of the AUV dock point --clearance-m above the station's; the 3D-gap closure stays alongside;
- first full hold (protocol v1.5's criterion, piccard-inc/piccard-experiments#88): per threshold, for the 3D and the
  at-clearance closures, the first hold_s window inside the final stage in which the criterion holds continuously
  (docked_any_window), and the first such window entered at <= the speed limit (docked_any_window_at_speed: the
  criterion is existential); criterion.scored names the definition the trial is scored by (first_full_hold from
  protocol v1.5, first_closure before), and both are reported;
- the minimum horizontal gap, and with --meshes the closest AUV-station part pair there and at the minimum gap;
- provenance: the job context's role and protocol_version, and whether the trial's mission is the one the context
  bound (runtime or builder hash form; the builder form knows planner missions);
- tags: the forward camera's tag layout (trial.json, else the planner's tag pivot record) and per-camera detections;
- perception: the fused station dock point minus ground truth at full fuser rate (range, lateral, vertical, yaw),
  binned by true range and by the forward camera's tag set; the series goes to LABEL.m3-perception.jsonl.
With --matrix, a table over the trials. Definitions are Piccard's, not lab-approved. Standard library only, except
numpy for --meshes.

    race_m3_metrics.py DIR [DIR ...] --output-dir OUT [--hold-s 30] [--speed-limit-mps 0.1] [--meshes DIR] [--matrix]
                       [--clearance-m 0.02 --clearance-tol-m 0.005]
"""
from __future__ import annotations

import argparse
import bisect
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import statistics
import sys

HERE = Path(__file__).resolve().parent
SCHEMA = "piccard.race-auv.m3-metrics/v1"
MATRIX_SCHEMA = "piccard.race-auv.m3-matrix/v1"
THRESHOLDS_M = (0.02, 0.01)
HOLD_S = 30.0
SPEED_HALF_WINDOW_S = 0.5
FINAL_APPROACH_STANDOFF_M = 0.3
MAX_ROW_GAP_S = 1.0
SERIES_STEP_S = 1.0
LOAD_BIN_S = 0.5
COUPLING_STEP_S = 1.0
SPEED_LIMIT_MPS = 0.1  # the lab's dock criterion: entered at <= 0.1 m/s
TAG_FRESH_S = 0.5  # the fuser's detection_max_age: a tag counts as in the solve this long after its detection
TRUTH_MATCH_S = 0.1  # a fused sample is compared with the ground-truth dock row nearest in time, if this close
RANGE_BIN_EDGES_M = (0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0)
CLAIM = ("Synthetic Stonefish simulation scored by ground truth; Piccard's M3 dock-criterion definitions "
         "(piccard-inc/piccard-experiments#88), not lab-approved; mesh geometry and Bullet contact reporting only, no "
         "physical docking, contact-mechanics or perception-accuracy claim: the fused-minus-truth numbers describe this "
         "simulation's rendered tags and the lab's pinned fuser, not the lab's hardware. closure_at_clearance scores "
         "protocol v1.3's redefinition, the AUV dock point held at the approach clearance above the station dock point "
         "(the simulated controller cannot descend onto the dock), Piccard's and not the lab's. A trial whose "
         "controller_variant is not upstream ran Piccard's modification of the lab's controller "
         "(piccard-inc/piccard-experiments#88 arm M3-C: mvp_control keeping the x/y integrals across set-point "
         "changes), gains unchanged: its own arm, saying nothing about the lab's controller. From protocol v1.5 a "
         "trial is scored docked on a full hold (a hold_s window inside the final stage in which the criterion holds "
         "continuously), and docked at speed if any such window was entered at <= the speed limit, preregistered on "
         "piccard-inc/piccard-experiments#88; earlier trials stay scored on their first closure.")
FIRST_FULL_HOLD_FROM = (1, 5)  # protocol_version from which docked is the first full hold
STATUS_PLAIN = {"completed": "completed the mission", "budget_censored": "ran to the horizon without completing",
                "failed": "failed"}
STOP_REASON_PLAIN = {
    "mission_complete": "the mission completed", "fixed_wall_horizon": "the fixed horizon was reached",
    "not_started": "the trial did not start", "readiness_timeout": "the simulator was not ready in time",
    "wall_timeout": "the wall-clock timeout was reached", "helm_left_direct_control": "the helm left direct_control",
    "tag_pivot_mismatch": "the mission's tag pivot differs from the station's construction",
    "tag_pivot_underivable": "the tag pivot could not be derived from the fuser's files",
    "planner_parameters_mismatch": "the planner reported other parameters than the mission's",
    "planner_state_stale": "the planner's state stopped arriving",
    "planner_never_commanded": "the planner never commanded a set point",
    "planner_setpoint_not_applied": "the controller did not apply the planner's set point",
    "planner_start_rejected": "the planner refused to start"}


def load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


m2 = load("race_m2_metrics", HERE / "race_m2_metrics.py")
finite, rounded = m2.finite, m2.rounded
PLANNER_EXAMPLE = m2.ROOT / "packages/simulation/race-auv-docking/examples/m3-planner-request.json"
PLANNER_SCHEMA = "piccard.race-auv.planner-mission/v1"


def plain(mapping: dict, value) -> str | None:
    """A plain-word rendering of a status or stop reason; unknown codes are spelled out."""
    if value is None:
        return None
    text = str(value)
    if text.startswith("consumption_audit"):
        return "the consumption audit found a problem (" + text.split(":", 1)[-1].replace(",", ", ") + ")"
    if text.startswith("signal_"):
        return f"stopped by signal {text[len('signal_'):]}"
    return mapping.get(text, text.replace("_", " "))


def label_of(threshold: float) -> str:
    return f"{threshold * 100:g}cm"


# ----------------------------------------------------------------------------- dock rows
def dock_rows(rows: list[dict], start: float, end: float) -> list[dict]:
    """Mission-window dock rows as {t, gap, rel (station dock frame), horizontal (|along, lateral| of rel), vertical
    (z_W AUV - z_W station)}."""
    result = []
    for row in rows:
        if row["kind"] != "dock" or not start <= row["t"] <= end or not finite(row.get("distance_m")):
            continue
        rel = row.get("relative_position_in_station_dock_m")
        auv, station = row.get("auv_dock_point_m"), row.get("station_dock_point_m")
        rel = rel if rel and all(map(finite, rel)) else None
        result.append({"t": row["t"], "gap": row["distance_m"], "rel": rel, "label": row.get("label"),
                       "horizontal": math.hypot(rel[0], rel[1]) if rel else None,
                       "vertical": auv[2] - station[2] if auv and station else None})
    return result


def speed_at(rows: list[dict], times: list[float], t: float) -> dict | None:
    """Central difference of the relative position between the dock rows at or before t - h and at or after t + h
    (clamped to the series), as speed and its station dock-frame velocity."""
    i = max(bisect.bisect_right(times, t - SPEED_HALF_WINDOW_S) - 1, 0)
    j = min(bisect.bisect_left(times, t + SPEED_HALF_WINDOW_S), len(rows) - 1)
    a, b = rows[i], rows[j]
    if b["t"] <= a["t"] or a["rel"] is None or b["rel"] is None:
        return None
    velocity = [(b["rel"][k] - a["rel"][k]) / (b["t"] - a["t"]) for k in range(3)]
    return {"speed_mps": math.sqrt(sum(v * v for v in velocity)), "velocity_in_station_dock_mps": velocity,
            "span_s": b["t"] - a["t"]}


def standoff(row: dict):
    return -row["rel"][0] if row["rel"] is not None else None


def final_approach(rows: list[dict], times: list[float], closure_index: int) -> dict:
    """From the last crossing of the stand-off threshold before closure (the first row after the last row farther
    out; the first mission row if none was) to closure: the maximum speed and where it occurred."""
    begin = 0
    for index in range(closure_index, -1, -1):
        value = standoff(rows[index])
        if value is not None and value > FINAL_APPROACH_STANDOFF_M:
            begin = index + 1
            break
    speeds = [(rows[k]["t"], speed_at(rows, times, rows[k]["t"])) for k in range(begin, closure_index + 1)]
    speeds = [(t, s["speed_mps"]) for t, s in speeds if s]
    top = max(speeds, key=lambda item: item[1], default=None)
    return {"from_t": rounded(rows[begin]["t"], 3), "crossed_standoff": begin > 0,
            "max_speed_mps": rounded(top[1]) if top else None, "max_speed_t": rounded(top[0], 3) if top else None}


def observed(rows: list[dict], start: float, end: float, mission_end: float) -> bool:
    """The window lies inside the mission and its dock rows leave no gap longer than MAX_ROW_GAP_S."""
    if end > mission_end:
        return False
    times = [start] + [r["t"] for r in rows] + [end]
    return all(b - a <= MAX_ROW_GAP_S for a, b in zip(times, times[1:]))


def stats(values: list[float]) -> dict | None:
    if not values:
        return None
    return {"min": rounded(min(values)), "median": rounded(statistics.median(values)), "max": rounded(max(values)),
            "mean": rounded(statistics.fmean(values))}


# ----------------------------------------------------------------------------- load and coupling
def load_during(events: list[dict], start: float, end: float) -> dict:
    inside = [e for e in events if e.get("contact") == "auv_station" and start <= e["t"] <= end]
    loaded = [e for e in inside if e["class"] == "force_bearing"]
    bins = max(1, math.ceil((end - start) / LOAD_BIN_S - 1e-9))  # 30.000000000000004 s is 60 bins
    return {"events": len(inside), "force_bearing_events": len(loaded),
            "near_miss_events": len(inside) - len(loaded),
            "max_normal_force_n": rounded(max((e.get("normal_force_n") or 0.0 for e in inside), default=0.0)),
            "force_bearing_bin_fraction": rounded(len({int((e["t"] - start) // LOAD_BIN_S) for e in loaded}) / bins)}


class Coupling:
    """Mesh separation as contact_margin_check_v1_1.py places the meshes (the served tool, imported unchanged)."""

    def __init__(self, meshes: Path):
        self.tool = load("contact_margin_check_v1_1", HERE / "contact_margin_check_v1_1.py")
        self.scene = self.tool.Scene(meshes)
        self.auv_parts = {name: self.tool.Mesh([self.tool.load_obj(meshes / name)]) for name in self.tool.AUV_PARTS}
        self.station_parts = {name: self.tool.Mesh([self.tool.load_obj(meshes / name)])
                              for name in self.tool.STATION_PARTS}
        self.provenance = {
            "tool": {"file": "contact_margin_check_v1_1.py",
                     "sha256": hashlib.sha256((HERE / "contact_margin_check_v1_1.py").read_bytes()).hexdigest()},
            "meshes": {name: self.tool.sha256(meshes / name)
                       for name in (*self.tool.AUV_PARTS, *self.tool.STATION_PARTS)},
            "sources": self.tool.SOURCES}
        self.trials = {}

    def at(self, root: Path, t: float, parts: bool = False) -> dict:
        trial = self.trials.setdefault(root, self.tool.Trial(root))
        whole, whole_cross, gap = self.scene.at(trial, t)
        couplink, couplink_cross, _ = self.scene.at(trial, t, couplinks=True)
        return {"mesh_mm": self.tool.mm(whole), "couplink_mm": self.tool.mm(couplink),
                "edges_cross": bool(whole_cross or couplink_cross), "pose_gap_s": rounded(float(gap), 3),
                **({"parts": self.parts_at(trial, t)} if parts else {})}

    def parts_at(self, trial, t: float) -> dict:
        """Every AUV part's separation (mm; crossing edges read 0) from every station part, and the closest pair: on
        M3-0 v1.2 the legs met the station frame at its rail entrance."""
        ra, ta, _ = trial.pose("gt_auv", t)
        rs, ts, _ = trial.pose("gt_station", t)
        station = {name: mesh.placed(rs, ts, self.tool.rz(self.tool.STATION_PART_YAW))
                   for name, mesh in self.station_parts.items()}
        pairs = {}
        for auv_name, mesh in self.auv_parts.items():
            placed = mesh.placed(ra, ta)
            for station_name, other in station.items():
                distance, _ = self.tool.separation(placed, other)
                pairs[f"{Path(auv_name).stem}~{Path(station_name).stem}"] = self.tool.mm(distance)
        measured = {pair: value for pair, value in pairs.items() if value is not None}
        closest = min(measured, key=measured.get, default=None)
        return {"pairs_mm": pairs, "closest_pair": closest, "closest_mm": measured.get(closest)}

    def window(self, root: Path, closure_t: float, end: float) -> dict:
        samples, t = [], closure_t
        while t <= end + 1e-9:
            samples.append(self.at(root, t))
            t += COUPLING_STEP_S
        values = {key: [s[key] for s in samples if s[key] is not None] for key in ("mesh_mm", "couplink_mm")}
        return {"at_closure": samples[0], "samples": len(samples),
                **{f"hold_{key}": stats(values[key]) for key in values},
                "hold_samples_touching": sum(s["couplink_mm"] is not None and s["couplink_mm"] < 0.1 for s in samples),
                "max_pose_gap_s": rounded(max(s["pose_gap_s"] for s in samples), 3)}


# ----------------------------------------------------------------------------- planner provenance
def canonical_sha256(value) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def planner_provenance(trial: dict) -> dict:
    """trial.json planner (null or absent: no planner node) and each audit's planner_inputs."""
    planner = trial.get("planner") or None
    audits = {name: (trial.get(f"consumption_audit_{name}") or {}).get("planner_inputs")
              for name in ("before_mission", "at_end")}
    result = {"present": bool(planner and planner.get("node")), "node": (planner or {}).get("node"),
              "parameters_sha256": (planner or {}).get("parameters_sha256"), "parameters_hash_verified": None,
              "stages_reached": [stage.get("standoff_m") for stage in (planner or {}).get("stages") or []],
              "complete": (planner or {}).get("complete"),
              "planner_inputs": audits, "reads_only_fuser_tf_and_odometry": None, "problems": []}
    if not result["present"]:
        if any(audits.values()):
            result["problems"].append("audit reports planner_inputs but trial.json has no planner")
        return result
    if isinstance(planner.get("parameters"), dict):
        result["parameters_hash_verified"] = canonical_sha256(planner["parameters"]) == result["parameters_sha256"]
        if not result["parameters_hash_verified"]:
            result["problems"].append("planner parameters_sha256 does not match its parameters")
    verdicts = [(audit or {}).get("reads_only_fuser_tf_and_odometry") for audit in audits.values()]
    if any(verdict is None for verdict in verdicts):
        result["problems"].append("a consumption audit has no planner_inputs verdict")
    else:
        result["reads_only_fuser_tf_and_odometry"] = all(verdicts)
        if not all(verdicts):
            result["problems"].append("planner read outside the fuser TF and odometry")
    return result


# ----------------------------------------------------------------------------- mission binding
def cast_like(template, value):
    """value with template's float-ness: the submit path re-renders integral floats as integers (2.0 as 2)."""
    if isinstance(template, float) and finite(value):
        return float(value)
    if isinstance(template, list) and isinstance(value, list) and template:
        return [cast_like(template[min(k, len(template) - 1)], v) for k, v in enumerate(value)]
    return value


def builder_mission(mission: dict) -> dict:
    """The mission as build_phase_r_campaign.py rendered it, whose bytes its context's mission_sha256 hashes: every
    pose and fallback-pose coordinate and angle a float (as race_m2_metrics' builder form), and each planner value a
    float where the builder's source, the planner example, has one."""
    floats = lambda pose: {**pose, **{k: float(pose[k]) for k in m2.COMMAND_FIELDS if finite(pose.get(k))}}
    result = {**mission, "poses": [floats(pose) for pose in mission.get("poses") or []]} if "poses" in mission \
        else dict(mission)
    if mission.get("schema") == PLANNER_SCHEMA:
        template = json.loads(PLANNER_EXAMPLE.read_text())["mission"]["planner"]
        result["fallback_pose"] = floats(mission.get("fallback_pose") or {})
        result["planner"] = {k: cast_like(template.get(k), v) for k, v in (mission.get("planner") or {}).items()}
    return result


def mission_binding(trial: dict, context: dict) -> tuple:
    """(matches, form) of the context's mission_sha256: "runtime" (trial.json's hash of the bytes the runtime got) or
    "builder" (the campaign builder's rendering); (None, None) when the context binds no mission."""
    bound = context.get("mission_sha256")
    if not bound:
        return None, None
    mission = trial.get("mission") or {}
    forms = {"runtime": trial.get("mission_sha256"),
             "builder": hashlib.sha256((json.dumps(builder_mission(mission), indent=2, sort_keys=True) + "\n")
                                       .encode()).hexdigest()}
    form = next((name for name, digest in forms.items() if digest == bound), None)
    return form is not None, form


# ----------------------------------------------------------------------------- planner timeline and tags
def telemetry_rows(path: Path, kind: str) -> list[dict]:
    """Rows of one kind that race_m2_metrics.load_trial does not keep (it filters by KINDS)."""
    rows = []
    with (m2.trial_root(path) / "telemetry.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            if f'"{kind}"' in line[:48]:
                row = json.loads(line)
                if row.get("kind") == kind and finite(row.get("t")):
                    rows.append(row)
    return rows


def setpoint_steps(planner: dict | None, planner_rows: list[dict]) -> dict:
    """The planner's discrete set-point steps as [t, x, y, z, yaw] (collector time; world_ned, cg_link): trial.json's
    record when the collector kept one, else each planner state row whose setpoint_updates changed."""
    if planner and planner.get("setpoint_steps") is not None:
        return {"source": "trial.json", "steps": planner["setpoint_steps"]}
    steps, previous = [], None
    for row in planner_rows:
        state = row.get("planner") or {}
        updates, command = state.get("setpoint_updates"), state.get("held") or state.get("command")
        if updates is not None and command and updates != previous:
            if previous is not None or any(updates.values()):
                steps.append([rounded(row["t"], 3)] + [rounded(command.get(k), 4) for k in ("x_m", "y_m", "z_m", "yaw_rad")])
            previous = updates
    return {"source": "telemetry" if planner_rows else None, "steps": steps}


def collinearity_m(points: list[list[float]]) -> float | None:
    """The largest distance of the points from the line through their two farthest-apart members."""
    if len(points) < 3:
        return None
    a, b = max(((p, q) for p in points for q in points), key=lambda pq: math.dist(*pq))
    axis = [b[k] - a[k] for k in range(3)]
    norm = math.sqrt(sum(v * v for v in axis))

    def off(point):
        d = [point[k] - a[k] for k in range(3)]
        cross = [d[1] * axis[2] - d[2] * axis[1], d[2] * axis[0] - d[0] * axis[2], d[0] * axis[1] - d[1] * axis[0]]
        return math.sqrt(sum(v * v for v in cross)) / norm
    return rounded(max(off(p) for p in points), 4)


def tag_record(trial: dict, detections: list[dict], start: float, end: float) -> dict:
    """The forward camera's tag layout (trial.json tags; else the planner's tag pivot record) with its collinearity,
    and per camera the mission-window detection rows that held each tag id."""
    layout = trial.get("tags") or None
    source = "trial.json tags" if layout else None
    if not layout:
        derived = ((trial.get("planner") or {}).get("tag_pivot") or {}).get("derived")
        if derived:
            layout = {"cameras": {derived["camera"]: {"tags": [{"tag": k, "position_in_dock_m": v}
                                                               for k, v in derived["tags"].items()]}}}
            source = "trial.json planner.tag_pivot"
    counts = {}
    for row in detections:
        if start <= row["t"] <= end:
            camera = counts.setdefault(row.get("camera"), {"rows": 0, "ids": {}})
            camera["rows"] += 1
            for tag in row.get("ids") or []:
                camera["ids"][tag] = camera["ids"].get(tag, 0) + 1
    forward = ((layout or {}).get("cameras") or {}).get("cam_front") or {}
    return {"source": source, "layout": layout, "detections_in_mission": counts,
            "forward_collinearity_m": collinearity_m([t["position_in_dock_m"] for t in forward.get("tags") or []])}


# ----------------------------------------------------------------------------- perception
def yaw_of(q) -> float:
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def perception(rows: list[dict], start: float, end: float) -> tuple[dict, list[list]]:
    """The fused station dock point (race_auv/base_link) minus ground truth at every fuser sample in the mission.
    Returns the summary (by true-range bin and by forward tag set: n, mean, sd) and the series."""
    truth = [r for r in rows if r["kind"] == "dock" and r.get("station_dock_in_auv_base_link_m")
             and r.get("relative_rpy_rad")]
    times = [r["t"] for r in truth]
    forward = [r for r in rows if r["kind"] == "detections" and r.get("camera") == "cam_front"]
    forward_t = [r["t"] for r in forward]
    series = []
    for row in rows:
        if row["kind"] != "fused_dock" or not start <= row["t"] <= end or not truth:
            continue
        k = bisect.bisect_left(times, row["t"])
        near = min((i for i in (k - 1, k) if 0 <= i < len(truth)), key=lambda i: abs(times[i] - row["t"]))
        if abs(times[near] - row["t"]) > TRUTH_MATCH_S:
            continue
        fused, true = row["position_m"], truth[near]["station_dock_in_auv_base_link_m"]
        true_range = math.sqrt(sum(v * v for v in true))
        horizontal = math.hypot(true[0], true[1]) or 1.0
        left = (-true[1] / horizontal, true[0] / horizontal)
        d = [fused[i] - true[i] for i in range(3)]
        i = bisect.bisect_right(forward_t, row["t"]) - 1
        tags = sorted({tag for r in forward[max(0, i - 10):i + 1] if row["t"] - r["t"] <= TAG_FRESH_S
                       for tag in r.get("ids") or []})
        series.append([rounded(row["t"], 3), rounded(true_range, 4),
                       rounded(math.sqrt(sum(v * v for v in fused)) - true_range, 5),
                       rounded(d[0] * left[0] + d[1] * left[1], 5), rounded(d[2], 5),
                       rounded(m2.dm.wrap(yaw_of(row["orientation_q"]) + truth[near]["relative_rpy_rad"][2]), 5),
                       ",".join(tags)])
    names = ("range_m", "lateral_m", "vertical_m", "yaw_rad")

    def summary(group: list[list]) -> dict:
        return {"n": len(group), **{name: {"mean": rounded(statistics.fmean(s[2 + k] for s in group), 5),
                                           "sd": rounded(statistics.pstdev(s[2 + k] for s in group), 5)}
                                    for k, name in enumerate(names)}}
    by_range, by_tags = {}, {}
    for sample in series:
        edge = next((e for e in RANGE_BIN_EDGES_M if sample[1] <= e), None)
        low = max([0.0] + [e for e in RANGE_BIN_EDGES_M if e < sample[1]])
        by_range.setdefault(f"{low:g}-{edge:g} m" if edge else f">{RANGE_BIN_EDGES_M[-1]:g} m", []).append(sample)
        by_tags.setdefault(sample[6] or "none", []).append(sample)
    order = lambda key: float(key.split("-")[0].lstrip(">").split(" ")[0])
    return ({"samples": len(series), "by_true_range": {k: summary(by_range[k]) for k in sorted(by_range, key=order)},
             "by_forward_tags": {k: summary(v) for k, v in sorted(by_tags.items())}}, series)


# ----------------------------------------------------------------------------- per trial
def criterion(threshold: float, clearance: tuple | None):
    """(distance, meets) for the 3D gap at threshold, or with clearance (clearance_m, tolerance_m) protocol v1.3's
    hover: the horizontal gap within threshold and the vertical offset within tolerance_m of -clearance_m (world z
    down: the AUV dock point above)."""
    if clearance is None:
        return (lambda row: row["gap"]), (lambda row: row["gap"] <= threshold)
    return ((lambda row: row["horizontal"]),
            (lambda row: row["horizontal"] is not None and row["horizontal"] <= threshold and finite(row["vertical"])
             and abs(row["vertical"] + clearance[0]) <= clearance[1]))


def final_stage(trial: dict, rows: list[dict]) -> dict:
    """Where the final stage starts: a planner mission's last stage (trial.json planner.stages), else a pose mission's
    last pose (the first mission dock row carrying its label); None when neither is recorded."""
    stages = (trial.get("planner") or {}).get("stages") or []
    if stages and finite(stages[-1].get("start_t")):
        return {"start_t": stages[-1]["start_t"], "label": stages[-1].get("label"), "source": "planner.stages"}
    poses = (trial.get("mission") or {}).get("poses") or []
    label = poses[-1].get("label") if poses else None
    first = next((row for row in rows if label is not None and row["label"] == label), None)
    if first is not None:
        return {"start_t": first["t"], "label": label, "source": "the first dock row of the mission's last pose"}
    return {"start_t": None, "label": None, "source": None}


def full_holds(rows: list[dict], times: list[float], meets, distance, stage: dict, hold_s: float,
               mission_start: float, mission_end: float, events: list[dict], coupling: Coupling | None, root: Path,
               speed_limit_mps: float):
    """Protocol v1.5: every hold_s window inside the final stage in which every dock row meets the criterion and the
    window is observed, in time order, each entered at a run's first row (or the first row after a dock-row gap)."""
    start = next((k for k, row in enumerate(rows) if row["t"] >= stage["start_t"]), None)
    for k in range(start if start is not None else len(rows), len(rows)):
        entry = meets(rows[k]) and (k == start or not meets(rows[k - 1])
                                    or rows[k]["t"] - rows[k - 1]["t"] > MAX_ROW_GAP_S)
        if not entry:
            continue
        t = rows[k]["t"]
        end = t + hold_s
        if end > mission_end:
            return
        window = rows[k:bisect.bisect_right(times, end)]
        if not (all(meets(row) for row in window) and observed(window, t, end, mission_end)):
            continue
        speed = speed_at(rows, times, t)
        distances = [distance(row) for row in window]
        verticals = [row["vertical"] for row in window if finite(row["vertical"])]
        yield {"computed": True, "found": True, "entry_t": rounded(t, 3),
               "time_to_entry_s": rounded(t - mission_start, 3) if finite(mission_start) else None,
               "time_into_final_stage_s": rounded(t - stage["start_t"], 3),
               "gap_at_entry_m": rounded(distance(rows[k])),
               "speed_at_entry_mps": rounded(speed["speed_mps"]) if speed else None,
               "velocity_at_entry_in_station_dock_mps": [rounded(v) for v in speed["velocity_in_station_dock_mps"]]
               if speed else None,
               "speed_ok": speed["speed_mps"] <= speed_limit_mps if speed else None,
               "window": {"from_t": rounded(t, 3), "to_t": rounded(end, 3), "samples": len(window),
                          "max_gap_m": rounded(max(distances)), "mean_gap_m": rounded(statistics.fmean(distances))},
               "vertical_offset_m": {"at_entry": rounded(rows[k]["vertical"]), "window": stats(verticals)},
               "load": load_during(events, t, end),
               "coupling": coupling.window(root, t, end) if coupling else None}


def first_full_hold(*args) -> tuple[dict, dict | None]:
    """(the first full window, or {found: false}; the first full window entered at <= the speed limit, or None when
    none was or its speed is unmeasurable). The preregistered criterion is existential: docked at speed if any full
    window was entered slowly enough."""
    stage = args[4]
    if stage["start_t"] is None:
        return ({"computed": False, "found": False,
                 "reason": "no final stage recorded (no planner stages, no pose labels on the dock rows)"}, None)
    windows = full_holds(*args)
    first = next(windows, None)
    if first is None:
        return {"computed": True, "found": False}, None
    at_speed = first if first["speed_ok"] else next((window for window in windows if window["speed_ok"]), None)
    return first, at_speed


def closure(rows: list[dict], times: list[float], threshold: float, hold_s: float, mission_start: float,
            mission_end: float, events: list[dict], coupling: Coupling | None, root: Path,
            speed_limit_mps: float = SPEED_LIMIT_MPS, clearance: tuple | None = None,
            stage: dict | None = None) -> dict:
    """The 3D-gap closure at threshold; with clearance (clearance_m, tolerance_m) protocol v1.3's hover at clearance
    instead, the *gap* fields then being the horizontal gap. Alongside, protocol v1.5's first full hold in the final
    stage (stage: final_stage())."""
    distance, meets = criterion(threshold, clearance)
    stage = stage or {"start_t": None}
    full, at_speed = first_full_hold(rows, times, meets, distance, stage, hold_s, mission_start, mission_end, events,
                                     coupling, root, speed_limit_mps)
    anywhere = {"first_full_hold": full, "first_full_hold_at_speed": at_speed, "docked_any_window": full["found"],
                "docked_any_window_at_speed": at_speed is not None}
    index = next((k for k, row in enumerate(rows) if meets(row)), None)
    if index is None:
        return {"closed": False, "docked": False, "speed_ok": None, "docked_at_speed": False, **anywhere}
    t = rows[index]["t"]
    end = t + hold_s
    hold = [row for row in rows if t <= row["t"] <= end]
    within = [row for row in hold if meets(row)]
    distances = [distance(row) for row in hold if distance(row) is not None]
    seen = observed(hold, t, end, mission_end)
    speed = speed_at(rows, times, t)
    verticals = [row["vertical"] for row in hold if finite(row["vertical"])]
    result = {
        "closed": True, "closure_t": rounded(t, 3),
        "time_to_closure_s": rounded(t - mission_start, 3) if finite(mission_start) else None,
        "gap_at_closure_m": rounded(distance(rows[index])),
        "speed_at_closure_mps": rounded(speed["speed_mps"]) if speed else None,
        "velocity_at_closure_in_station_dock_mps": [rounded(v) for v in speed["velocity_in_station_dock_mps"]]
        if speed else None,
        "final_approach": final_approach(rows, times, index),
        "hold": {"from_t": rounded(t, 3), "to_t": rounded(end, 3), "observed": seen, "samples": len(hold),
                 "fraction_within": rounded(len(within) / len(hold)), "max_gap_m": rounded(max(distances)),
                 "mean_gap_m": rounded(statistics.fmean(distances))},
        "docked": seen and len(within) == len(hold),
        "speed_ok": speed["speed_mps"] <= speed_limit_mps if speed else None,
        "vertical_offset_m": {"at_closure": rounded(rows[index]["vertical"]), "hold": stats(verticals)},
        "load": load_during(events, t, min(end, mission_end)),
        "coupling": coupling.window(root, t, min(end, mission_end)) if coupling else None}
    result["docked_at_speed"] = bool(result["docked"] and result["speed_ok"])
    return {**result, **anywhere}


def scored_criterion(context: dict, stage: dict) -> dict:
    """Which definition of docked the trial is scored by: the first full hold from protocol v1.5 (preregistered on
    piccard-inc/piccard-experiments#88, for trials run after it), the first closure before it or without a version."""
    version = context.get("protocol_version")
    try:
        number = tuple(int(part) for part in str(version).lstrip("v").split("."))
    except ValueError:
        number = ()
    full = bool(number) and number >= FIRST_FULL_HOLD_FROM
    return {"scored": "first_full_hold" if full else "first_closure", "protocol_version": version,
            "final_stage": {key: rounded(value, 3) if key == "start_t" and value is not None else value
                            for key, value in stage.items()}}


def gap_series(rows: list[dict]) -> list[list[float]]:
    """The first dock row of each SERIES_STEP_S bin, as [t, gap]."""
    series, seen = [], set()
    for row in rows:
        key = int(row["t"] // SERIES_STEP_S)
        if key not in seen:
            seen.add(key)
            series.append([rounded(row["t"], 3), rounded(row["gap"], 5)])
    return series


def analyze(path: Path, hold_s: float = HOLD_S, coupling: Coupling | None = None,
            speed_limit_mps: float = SPEED_LIMIT_MPS, series_out: list | None = None, clearance_m: float = 0.0,
            clearance_tol_m: float = 0.0) -> dict:
    """The trial's report; series_out, when given, receives the full-rate perception series. The hover-at-clearance
    criterion is scored only with clearance_tol_m > 0."""
    loaded = m2.load_trial(path)
    trial, rows, context = loaded["trial"], loaded["rows"], loaded["context"]
    start, end = trial.get("mission_start_t"), trial.get("mission_end_t")
    start = start if finite(start) else -math.inf
    end = end if finite(end) else math.inf
    docks = dock_rows(rows, start, end)
    times = [row["t"] for row in docks]
    events = m2.contact_events(rows)
    minimum = min(docks, key=lambda row: row["gap"], default=None)
    horizontal = min((row for row in docks if row["horizontal"] is not None), key=lambda row: row["horizontal"],
                     default=None)
    root = m2.trial_root(path)
    stage = final_stage(trial, docks)
    closures = {label_of(threshold): closure(docks, times, threshold, hold_s, start, end, events, coupling, root,
                                             speed_limit_mps, stage=stage) for threshold in THRESHOLDS_M}
    at_clearance = None
    if clearance_tol_m > 0:
        at_clearance = {label_of(threshold): closure(docks, times, threshold, hold_s, start, end, events, coupling,
                                                     root, speed_limit_mps, (clearance_m, clearance_tol_m), stage)
                        for threshold in THRESHOLDS_M}
    provenance = planner_provenance(trial)
    planner = trial.get("planner") or {}
    provenance["timeline"] = {
        "handover_t": planner.get("handover_t"),
        "stages": [{key: stage.get(key) for key in ("label", "standoff_m", "start_t")} for stage in planner.get("stages") or []],
        "stale_intervals": planner.get("stale_intervals"),
        "setpoint_steps": setpoint_steps(planner or None, telemetry_rows(path, "planner") if planner else []),
        "heading_rehold": planner.get("heading_rehold"), "approach_clearance_m": planner.get("approach_clearance_m"),
        "final_along": planner.get("final_along")}
    provenance["tag_pivot"] = planner.get("tag_pivot")
    detections = [row for row in rows if row["kind"] == "detections"]
    perception_summary, series = perception(rows, start, end)
    if series_out is not None:
        series_out.extend(series)
    audits = [trial.get(name) or {} for name in ("consumption_audit_before_mission", "consumption_audit_at_end")]
    audit_problems = sorted({p for audit in audits for p in audit.get("problems") or []})
    exclusions = []
    if audit_problems:
        exclusions.append("audit problem: " + ", ".join(audit_problems))
    if provenance["problems"]:
        exclusions.append("planner provenance: " + ", ".join(provenance["problems"]))
    if "nonfinite" in str(trial.get("stop_reason", "")):
        exclusions.append(f"non-finite critical stream stop ({trial.get('stop_reason')})")
    if not (trial.get("status") == "completed" and trial.get("stop_reason") == "mission_complete"):
        exclusions.append(f"trial incomplete ({trial.get('status')}, {trial.get('stop_reason')})")
    if not docks:
        exclusions.append("no ground-truth dock rows in the mission window")
    mission = trial.get("mission") or {}
    matches, form = mission_binding(trial, context)
    return {
        "schema": SCHEMA, "claim_boundary": CLAIM, "label": loaded["label"], "path": loaded["path"],
        "context": {key: context.get(key) for key in ("trial_id", "campaign_id", "stage", "arm", "role", "gain_set",
                                                        "mission_key", "repeat", "mission_sha256", "protocol",
                                                        "protocol_version", "controller_variant")},
        "controller_variant": trial.get("controller_variant"),
        "mvp_control_patch": (trial.get("source_pins") or {}).get("mvp_control_patch"),
        "mission_schema": mission.get("schema"), "mission_id": mission.get("mission_id"),
        "mission_sha256": trial.get("mission_sha256"), "mission_matches_context": matches, "mission_hash_form": form,
        "apriltag_tag_size": mission.get("apriltag_tag_size"), "simulator_seed": trial.get("simulator_seed"),
        "status": trial.get("status"), "stop_reason": trial.get("stop_reason"),
        "status_plain": plain(STATUS_PLAIN, trial.get("status")),
        "stop_reason_plain": plain(STOP_REASON_PLAIN, trial.get("stop_reason")),
        "time_basis": {"clock": "collector_monotonic_receive_seconds",
                       "mission_start_t": trial.get("mission_start_t"), "mission_end_t": trial.get("mission_end_t")},
        "hold_s": hold_s, "speed_limit_mps": speed_limit_mps, "criterion": scored_criterion(context, stage),
        "gap": {"minimum_m": rounded(minimum["gap"]) if minimum else None,
                "minimum_t": rounded(minimum["t"], 3) if minimum else None,
                "vertical_offset_at_minimum_m": rounded(minimum["vertical"]) if minimum else None,
                "coupling_at_minimum": coupling.at(root, minimum["t"], parts=True) if coupling and minimum else None,
                "minimum_horizontal_m": rounded(horizontal["horizontal"]) if horizontal else None,
                "minimum_horizontal_t": rounded(horizontal["t"], 3) if horizontal else None,
                "vertical_offset_at_minimum_horizontal_m": rounded(horizontal["vertical"]) if horizontal else None,
                "coupling_at_minimum_horizontal": coupling.at(root, horizontal["t"], parts=True)
                if coupling and horizontal else None,
                "series_1s": gap_series(docks)},
        "closure": closures,
        "clearance": {"clearance_m": clearance_m, "tolerance_m": clearance_tol_m, "computed": at_clearance is not None,
                      "planner_approach_clearance_m": planner.get("approach_clearance_m"),
                      **({} if at_clearance is not None else
                         {"reason": "no --clearance-tol-m > 0 given: the hover-at-clearance criterion is not scored"})},
        "closure_at_clearance": at_clearance,
        "coupling_provenance": coupling.provenance if coupling else
        {"computed": False, "reason": "no --meshes given (world_of_stonefish d51d59e collision meshes)"},
        "planner": provenance,
        "tags": tag_record(trial, detections, start, end),
        "perception": perception_summary,
        "audit_problems": audit_problems,
        "excluded": bool(exclusions), "exclusion_reasons": exclusions,
        "definitions": {
            "gap": "ground-truth distance between the AUV dock point and the station dock point, from the collector's "
                   "dock rows (runtime/docking_metric.py offsets, the same as race_m2_metrics and contact-margin "
                   f"v1.1), inside the mission window; series_1s is the first dock row of each {SERIES_STEP_S:g} s; "
                   "minimum_horizontal_m is the smallest horizontal gap (along and lateral in the station dock frame)",
            "closure": f"the first dock row with gap <= the threshold ({', '.join(f'{t:g} m' for t in THRESHOLDS_M)}); "
                       "time_to_closure_s is from the mission start (trial.json mission_start_t: the dive's start)",
            "closure_at_clearance": "protocol v1.3's criterion (piccard-inc/piccard-experiments#88): the simulated "
                                    "controller cannot descend onto the dock without a depth step, so the reachable "
                                    "docked state is the hover at the planner's approach clearance. Per threshold, the "
                                    "first dock row whose horizontal gap (along and lateral of the relative position "
                                    "in the station dock frame) is <= the threshold and whose vertical_offset_m is "
                                    "within clearance.tolerance_m of -clearance.clearance_m (the AUV dock point "
                                    "above); speed, hold (every hold row meeting both), load and coupling as for "
                                    "closure, and its gap_at_closure_m, max_gap_m and mean_gap_m are the horizontal "
                                    "gap. Scored only with --clearance-tol-m > 0 (clearance.computed); "
                                    "clearance.planner_approach_clearance_m is the trial's planner value. closure "
                                    "(the 3D gap) is reported alongside, unchanged",
            "speed": f"|d/dt| of the AUV dock point's position relative to the station dock point (station dock "
                     f"frame), a central difference between the dock rows at or before t - {SPEED_HALF_WINDOW_S:g} s "
                     f"and at or after t + {SPEED_HALF_WINDOW_S:g} s (clamped to the series)",
            "final_approach": f"from the first dock row after the last row with stand-off (-x of the relative "
                              f"position in the station dock frame) > {FINAL_APPROACH_STANDOFF_M:g} m before closure, "
                              "to closure; the mission start if the stand-off never exceeded it",
            "hold": f"the dock rows from closure to closure + hold_s; docked = closed, every hold row within the "
                    f"threshold, and the hold observed: inside the mission window with no dock-row gap over "
                    f"{MAX_ROW_GAP_S:g} s. docked alone is gap and hold only; speed is scored separately",
            "first_full_hold": "protocol v1.5's criterion (piccard-inc/piccard-experiments#88, for trials run after "
                               "its preregistration): per threshold, for closure and closure_at_clearance alike, the "
                               "first hold_s window inside the final stage in which every dock row meets the "
                               "criterion and the window is observed (no dock-row gap over "
                               f"{MAX_ROW_GAP_S:g} s, inside the mission). A window starts at a run's first row or "
                               "the first row after a dock-row gap. docked_any_window: a full window exists "
                               "(first_full_hold, with its entry speed and speed_ok). The criterion is existential, "
                               "so docked_any_window_at_speed is true when any full window was entered at <= the "
                               "speed limit: first_full_hold_at_speed is the first such window (null when none was, "
                               "or its speed is unmeasurable). The final stage is "
                               "a planner mission's last stage (trial.json planner.stages) or a pose mission's last "
                               "pose (the first dock row carrying its label); without either, first_full_hold is "
                               "not computed",
            "criterion": "criterion.scored names the definition the trial is scored by: first_full_hold (docked = "
                         "docked_any_window, at speed = docked_any_window_at_speed) from protocol_version v1.5, "
                         "first_closure (docked, docked_at_speed) before it or without a protocol_version. Both are "
                         "always reported; criterion.final_stage says where the final stage starts and from what",
            "dock_criterion": "the lab's criterion is gap closed, held, and entered at <= speed_limit_mps: speed_ok = "
                              "speed_at_closure_mps <= speed_limit_mps (null without a speed); docked_at_speed = docked "
                              "and speed_ok. speed_limit_mps is this run's --speed-limit-mps (default "
                              f"{SPEED_LIMIT_MPS:g}); the planner's speed_cap_mps caps the commanded set point, not this",
            "time_fields": "every *_t field (closure_t, minimum_t, hold from_t/to_t, final_approach from_t and "
                           "max_speed_t, series_1s times, minimum_horizontal_t, the same fields of "
                           "closure_at_clearance, planner timeline handover_t, stages start_t, stale_intervals "
                           "from_t/to_t, setpoint_steps t, final_along received_t, perception series t) is collector "
                           "time "
                           "(time_basis.clock: seconds on the collector's monotonic clock at receipt); mission time is "
                           "t - time_basis.mission_start_t, and time_to_closure_s is already mission time. The mission "
                           "starts with the dive (the fallback pose) and ends at time_basis.mission_end_t",
            "frames_and_signs": "relative_position_in_station_dock_m (dock rows): the AUV dock point in the station "
                                "dock frame, x along the approach into the station (stand-off = -x), y left, z UP "
                                "(+z: AUV shallower). vertical_offset_m here: z_W(AUV) - z_W(station) with world z "
                                "DOWN (-: AUV shallower). The same position therefore reads +0.0229 m as a relative "
                                "z and -0.0229 m as vertical_offset_m. Distances in m, angles in rad, speeds in m/s",
            "true_range": "|station dock point in race_auv/base_link| from the dock rows "
                          "(station_dock_in_auv_base_link_m): the camera-side range the fuser sees, measured from "
                          "the AUV's base_link origin, 0.395 m aft of the AUV dock point and 0.13 m above it. It is "
                          "not the gap",
            "planner_frame": "the planner's error is in L, the level station frame at the planner's station estimate: "
                             "x along the held heading (the first stage's window median, held from its end; from "
                             "protocol v1.2 re-held once at the final stage's start from the stage before it: "
                             "planner.timeline.heading_rehold {t on the planner's clock, received_t, samples, "
                             "applied, previous, heading}), z up; error = [along, lateral, vertical] of the AUV dock "
                             "point from the stage target (stand-off s behind the station dock point on L's x axis; "
                             "from protocol v1.3 approach_clearance_m above it on L's z axis, in every stage, "
                             "planner.timeline.approach_clearance_m), then the heading error (+: the station's axis "
                             "lies to the AUV's left)",
            "planner_stage": "stage is an index into the mission's planner.standoffs_m (0 = the first stand-off). "
                             "A stage other than the last ends after settle_s with the fused error inside the band: "
                             "band_m along, vertical_band_m vertical (protocol v1.2; band_m before it), band_rad in "
                             "heading, band_m + s * band_rad lateral at stand-off s. In such a stage, once the "
                             "vehicle has been at its set point for settle_s (cg_link within band_m of it on x and y, "
                             "and from protocol v1.3 within vertical_band_m on z; band_m on z before), every axis "
                             "outside its band steps to the goal if the goal differs from the held value by more "
                             "than setpoint_deadband_m (_rad for yaw); v1.2 stepped whatever the difference. The last "
                             "stage lasts final_stage_s; from v1.2 its x/y follow the live goal past final_deadband_m, "
                             "walked, with yaw and depth held; from v1.4 per world axis, the along axis (the one "
                             "closer to the approach) also gated on the fused along error past final_along_deadband_m "
                             "and final_along_interval_s since its last change. From v1.5 the along axis approaches "
                             "(re-targeted only on a stale command: the goal past final_along_deadband_m from the held "
                             "value, final_along_interval_s since its last change), arrives once within "
                             "final_arrival_m of the goal on the approach's side (the along set point re-issued at the "
                             "goal, which zeroes the approach's integral windup), holds (following the goal past "
                             "final_deadband_m) and approaches again after final_reapproach_s past "
                             "final_along_deadband_m: planner.timeline.final_along {phase, side, arrivals [{t, error_m, "
                             "side, change_m, received_t}], reapproaches [{t, error_m, side, received_t}]}, t on the "
                             "planner's clock",
            "setpoint_steps": "the planner's discrete changes of its held set point as [t, x, y, z, yaw] of cg_link in "
                              "race_auv/world_ned (m, rad): from trial.json planner.setpoint_steps, else each planner "
                              "state row whose setpoint_updates changed (source says which). From protocol v1.1 the "
                              "x/y command walks to a stage start's held value at speed_cap_mps; the entry is where "
                              "the walk lands",
            "tags": "the forward camera's configured tags and their positions in the station dock frame (m) from "
                    "the station URDF the fuser loads; forward_collinearity_m is their largest distance from the line "
                    "through the two farthest apart (0 m: collinear, so the fuser's joint solve on tag centres leaves "
                    "rotation about that line free); detections_in_mission counts, per camera, the detection rows "
                    "holding each tag id",
            "perception": "at every fused_dock sample in the mission, the fused station dock point minus ground truth "
                          f"(the dock row nearest in time, within {TRUTH_MATCH_S:g} s), in race_auv/base_link: "
                          "range_m = |fused| - |true|; lateral_m along the horizontal left normal of the true line of "
                          "sight; vertical_m along base_link z (up); yaw_rad = fused dock-frame heading in base_link "
                          "minus true (-relative_rpy_rad yaw), wrapped. Summaries are n, mean and sd per true-range bin "
                          "and per forward tag set (cam_front ids detected within the fuser's "
                          f"{TAG_FRESH_S:g} s detection age). Series in LABEL.m3-perception.jsonl as "
                          "[t, true_range_m, range_m, lateral_m, vertical_m, yaw_rad, forward_tags]",
            "provenance": "context.role and context.protocol_version are the job context's (the campaign builder "
                          "writes them); mission_matches_context tells whether the context's mission_sha256 is this "
                          "trial's mission, in the runtime form (trial.json's hash of the bytes the runtime received) "
                          "or the builder form (the builder's rendering: the submit path re-renders integral floats "
                          "as integers, so the builder form casts back every value the builder wrote as a float)",
            "controller_variant": "trial.json controller_variant: upstream (the lab's mvp_control as pinned) or "
                                  "Piccard's variant (keep_xy_integral, piccard-inc/piccard-experiments#88 arm M3-C: "
                                  "its image's mvp_control patch, mvp_control_patch from trial.json source_pins); null "
                                  "for trials recorded before the field existed, which ran upstream. "
                                  "context.controller_variant is the builder's",
            "simulator_seed": "trial.json simulator_seed, as recorded: value null with support "
                              "not_exposed_by_pinned_runner when the simulator exposed no seed (every run before "
                              "stonefish_seed_v1, both arms of the M3 report among them); null for a record without "
                              "the field. A seed fixes the sensor-noise sequence, not the trajectory",
            "plain_words": "status_plain and stop_reason_plain restate status and stop_reason in words; "
                           "budget_censored means the trial ran to the horizon without completing",
            "load": f"AUV-station contact events during the hold in the v0.3 classes (force_bearing: normal force > 0; "
                    f"near_miss: exactly 0); force_bearing_bin_fraction is the share of {LOAD_BIN_S:g} s bins with a "
                    f"force-bearing event; max_normal_force_n: {m2.FORCE_NOTE}",
            "coupling": f"whole-mesh and couplink-to-couplink separation (mm) at the minimum gap, at closure and each "
                        f"{COUPLING_STEP_S:g} s of the hold, placed and measured by contact_margin_check_v1_1.py "
                        "(ground-truth poses interpolated to the time); touching means a couplink separation below "
                        "0.1 mm; exactly coincident dock points interpenetrate the couplinks by 2.6 mm "
                        "(piccard-inc/piccard-physical-ai#100). At the minimum gap and the minimum horizontal gap, "
                        "parts: every AUV part against every station part (pairs_mm, crossing edges read 0) and the "
                        "closest pair",
            "vertical_offset": "z_W(AUV dock point) - z_W(station dock point) from the dock rows; world z points down, "
                               "so negative means the AUV dock point is shallower (contact-margin v1.1 "
                               "vertical_offset_mm, in m)",
            "planner": "trial.json planner: {node, parameters, parameters_sha256 = sha256 of the parameters as JSON "
                       "with sorted keys and no spaces}, null or absent when no planner node ran; each consumption "
                       "audit's planner_inputs.reads_only_fuser_tf_and_odometry; a problem excludes the trial",
            "exclusion": "audit problem, planner provenance problem, non-finite critical stream stop, incomplete trial "
                         "or no dock rows: re-run once"},
    }


# ----------------------------------------------------------------------------- matrix
def matrix(reports: list[dict]) -> dict:
    rows = []
    for report in reports:
        row = {"label": report["label"], "stage": report["context"].get("stage"), "arm": report["context"].get("arm"),
               "role": report["context"].get("role"), "protocol_version": report["context"].get("protocol_version"),
               "controller_variant": report["controller_variant"], "criterion": report["criterion"]["scored"],
               "gain_set": report["context"].get("gain_set"), "excluded": report["excluded"],
               "status_plain": report["status_plain"], "mission_matches_context": report["mission_matches_context"],
               "minimum_gap_m": report["gap"]["minimum_m"],
               "vertical_offset_at_minimum_m": report["gap"]["vertical_offset_at_minimum_m"],
               "planner_present": report["planner"]["present"],
               "planner_reads_only_fuser_tf_and_odometry": report["planner"]["reads_only_fuser_tf_and_odometry"]}
        for name, item in report["closure"].items():
            row[name] = {"closed": item["closed"], "docked": item["docked"], "speed_ok": item["speed_ok"],
                         "docked_at_speed": item["docked_at_speed"],
                         **({"closure_t": item["closure_t"], "time_to_closure_s": item["time_to_closure_s"],
                             "speed_at_closure_mps": item["speed_at_closure_mps"],
                             "final_approach_max_speed_mps": item["final_approach"]["max_speed_mps"],
                             "hold_fraction_within": item["hold"]["fraction_within"],
                             "hold_max_gap_m": item["hold"]["max_gap_m"],
                             "force_bearing_events": item["load"]["force_bearing_events"],
                             "hold_couplink_min_mm": ((item["coupling"] or {}).get("hold_couplink_mm") or {}).get("min")}
                            if item["closed"] else {}),
                         "any_window": any_window(item)}
            if report["closure_at_clearance"]:
                at = report["closure_at_clearance"][name]
                row[name]["at_clearance"] = {
                    "closed": at["closed"], "docked": at["docked"], "speed_ok": at["speed_ok"],
                    "docked_at_speed": at["docked_at_speed"],
                    **({"closure_t": at["closure_t"], "speed_at_closure_mps": at["speed_at_closure_mps"],
                        "hold_fraction_within": at["hold"]["fraction_within"]} if at["closed"] else {}),
                    "any_window": any_window(at)}
        row["clearance"] = {key: report["clearance"][key] for key in ("clearance_m", "tolerance_m", "computed")}
        rows.append(row)
    speed_limits = sorted({report["speed_limit_mps"] for report in reports})
    names = list(map(label_of, THRESHOLDS_M))
    included = [r for r in rows if not r["excluded"]]
    scored = [r for r in included if r["clearance"]["computed"]]

    def counts(trials, pick):
        return {**{key: {name: sum(1 for r in trials if pick(r[name])[key]) for name in names}
                   for key in ("docked", "docked_at_speed")},
                **{out: {name: sum(1 for r in trials if pick(r[name])["any_window"][key]) for name in names}
                   for out, key in (("docked_any_window", "docked"),
                                    ("docked_any_window_at_speed", "docked_at_speed"))},
                **{f"scored_{key}": {name: sum(1 for r in trials if scored_docked(r, pick(r[name]), key))
                                     for name in names} for key in ("docked", "docked_at_speed")}}

    return {"schema": MATRIX_SCHEMA, "claim_boundary": CLAIM, "trials": rows,
            "speed_limit_mps": speed_limits[0] if len(speed_limits) == 1 else speed_limits,
            **counts(included, lambda item: item), "included": len(included),
            "at_clearance": {"included": len(scored), **counts(scored, lambda item: item["at_clearance"])}}


def any_window(item: dict) -> dict:
    full = item["first_full_hold"]
    at_speed = item["first_full_hold_at_speed"] or {}
    return {"computed": full["computed"], "docked": item["docked_any_window"],
            "docked_at_speed": item["docked_any_window_at_speed"], "entry_t": full.get("entry_t"),
            "speed_at_entry_mps": full.get("speed_at_entry_mps"), "at_speed_entry_t": at_speed.get("entry_t"),
            "at_speed_speed_at_entry_mps": at_speed.get("speed_at_entry_mps")}


def scored_docked(row: dict, item: dict, key: str) -> bool:
    """docked (or docked_at_speed) by the definition the trial is scored by (criterion)."""
    return bool(item["any_window"][key] if row["criterion"] == "first_full_hold" else item[key])


def fmt(value, digits=3) -> str:
    if value is None:
        return "-"
    return f"{value:.{digits}f}" if isinstance(value, float) else str(value)


def matrix_markdown(result: dict) -> str:
    lines = ["# RACE M3 dock-criterion matrix", "", result["claim_boundary"], "",
             "| trial | excl. | min gap m | vert. at min m | planner | " +
             " | ".join(f"{n} closed t | {n} speed m/s | {n} hold frac | {n} docked | {n} at speed"
                        for n in map(label_of, THRESHOLDS_M)) + " |",
             "|---|---|---|---|---|" + "---|" * (5 * len(THRESHOLDS_M))]
    for row in result["trials"]:
        cells = [row["label"], "yes" if row["excluded"] else "", fmt(row["minimum_gap_m"], 4),
                 fmt(row["vertical_offset_at_minimum_m"], 4), "yes" if row["planner_present"] else "no"]
        for name in map(label_of, THRESHOLDS_M):
            item = row[name]
            cells += [fmt(item.get("closure_t"), 1), fmt(item.get("speed_at_closure_mps")),
                      fmt(item.get("hold_fraction_within")), "yes" if item["docked"] else "no",
                      "yes" if item["docked_at_speed"] else "no"]
        lines.append("| " + " | ".join(cells) + " |")
    lines += ["", "Docked (gap and hold), included trials: "
              + ", ".join(f"{k}: {v}" for k, v in result["docked"].items()) + f" of {result['included']}",
              f"Docked at <= {result['speed_limit_mps']} m/s, included trials: "
              + ", ".join(f"{k}: {v}" for k, v in result["docked_at_speed"].items()) + f" of {result['included']}", ""]
    scored = [row for row in result["trials"] if row["clearance"]["computed"]]
    if scored:
        names = list(map(label_of, THRESHOLDS_M))
        lines += ["## At the approach clearance (protocol v1.3)", "",
                  "| trial | clearance m | tol. m | " + " | ".join(f"{n} closed t | {n} docked | {n} at speed"
                                                           for n in names) + " |",
                  "|---|---|---|" + "---|" * (3 * len(names))]
        for row in scored:
            cells = [row["label"], fmt(row["clearance"]["clearance_m"]), fmt(row["clearance"]["tolerance_m"])]
            for name in names:
                item = row[name]["at_clearance"]
                cells += [fmt(item.get("closure_t"), 1), "yes" if item["docked"] else "no",
                          "yes" if item["docked_at_speed"] else "no"]
            lines.append("| " + " | ".join(cells) + " |")
        at = result["at_clearance"]
        lines += ["", "Docked at clearance (horizontal gap and hold), included trials: "
                  + ", ".join(f"{k}: {v}" for k, v in at["docked"].items()) + f" of {at['included']}",
                  f"Docked at clearance at <= {result['speed_limit_mps']} m/s, included trials: "
                  + ", ".join(f"{k}: {v}" for k, v in at["docked_at_speed"].items()) + f" of {at['included']}", ""]
    names = list(map(label_of, THRESHOLDS_M))
    clearance = any(row["clearance"]["computed"] for row in result["trials"])
    kinds = [("3D", lambda row, name: row[name])] + (
        [("clearance", lambda row, name: row[name].get("at_clearance"))] if clearance else [])
    lines += ["## First full hold in the final stage (protocol v1.5)", "",
              "The first 30 s window inside the final stage in which the criterion holds continuously (entry t), "
              "and the first such window entered at <= the speed limit (at-speed entry t; the criterion is "
              "existential). A trial is scored by the definition in its scored-by column: first_full_hold from "
              "protocol v1.5, first_closure before it.", "",
              "| trial | scored by | " + " | ".join(f"{n} {k} entry t | {n} {k} docked | {n} {k} at-speed entry t | "
                                                   f"{n} {k} at speed" for n in names for k, _ in kinds) + " |",
              "|---|---|" + "---|" * (4 * len(names) * len(kinds))]
    for row in result["trials"]:
        cells = [row["label"], row["criterion"]]
        for name in names:
            for _, pick in kinds:
                item = pick(row, name)
                window = item["any_window"] if item else None
                if not window or not window["computed"]:
                    cells += ["n/a", "-", "-", "-"]
                    continue
                cells += [fmt(window["entry_t"], 1), "yes" if window["docked"] else "no",
                          fmt(window["at_speed_entry_t"], 1), "yes" if window["docked_at_speed"] else "no"]
        lines.append("| " + " | ".join(cells) + " |")
    def listed(counts: dict) -> str:
        return ", ".join(f"{k}: {v}" for k, v in counts.items())

    at = result["at_clearance"]
    lines += ["", "Docked by the definition each trial is scored by, included trials: "
              + listed(result["scored_docked"]) + f" of {result['included']}"
              + (f" (3D); at clearance: {listed(at['scored_docked'])} of {at['included']}" if clearance else ""),
              f"The same at <= {result['speed_limit_mps']} m/s: " + listed(result["scored_docked_at_speed"])
              + (f" (3D); at clearance: {listed(at['scored_docked_at_speed'])}" if clearance else ""), ""]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("trials", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--hold-s", type=float, default=HOLD_S)
    parser.add_argument("--speed-limit-mps", type=float, default=SPEED_LIMIT_MPS,
                        help="the dock criterion's entry speed (default: the lab's 0.1 m/s)")
    parser.add_argument("--meshes", type=Path, help="directory with the six collision meshes contact_margin_check_v1_1.py "
                                                    "reads (world_of_stonefish d51d59e data/)")
    parser.add_argument("--matrix", action="store_true")
    parser.add_argument("--clearance-m", type=float, default=0.0,
                        help="protocol v1.3: the AUV dock point's height above the station dock point for "
                             "closure_at_clearance (the planner's approach_clearance_m)")
    parser.add_argument("--clearance-tol-m", type=float, default=0.0,
                        help="the vertical tolerance around it; closure_at_clearance is scored only when > 0")
    args = parser.parse_args(argv)
    clearance = (args.clearance_m, args.clearance_tol_m)
    if not all(finite(value) and value >= 0 for value in clearance):
        parser.error("--clearance-m and --clearance-tol-m must be finite and non-negative")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    try:
        coupling = Coupling(args.meshes) if args.meshes else None
        reports, series = [], []
        for path in args.trials:
            series.append([])
            reports.append(analyze(path, args.hold_s, coupling, args.speed_limit_mps, series[-1], args.clearance_m,
                                   args.clearance_tol_m))
    except (FileNotFoundError, ValueError, KeyError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    for report, samples in zip(reports, series):
        (args.output_dir / f"{report['label']}.m3.json").write_text(
            json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n")
        (args.output_dir / f"{report['label']}.m3-perception.jsonl").write_text(
            "".join(json.dumps(sample, separators=(",", ":"), allow_nan=False) + "\n" for sample in samples))
    if args.matrix:
        result = matrix(reports)
        (args.output_dir / "m3-matrix.json").write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False)
                                                        + "\n")
        (args.output_dir / "m3-matrix.md").write_text(matrix_markdown(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
