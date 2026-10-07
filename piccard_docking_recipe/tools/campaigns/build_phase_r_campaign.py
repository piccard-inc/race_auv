#!/usr/bin/env python3
"""M2 Phase R-A campaign builder for race-auv-docking/v1 (piccard-physical-ai #96).

Implements the preregistration on piccard-experiments #87 ("M2 preregistration: Phase R-A v0.1", 2026-09-28, as
amended by v0.2 of the same day): the five pose missions, the docking x/y knobs, the stages and the one re-run an
excluded trial gets; and the follow-ups of piccard-physical-ai #102 (stage a0b; M-depth-hold v2 and its stage
holdout2; stage a0c). It writes the jobs file the CTO's submit client consumes:
    {"recipe", "horizon_s", "wall_timeout_s", "jobs": [{"trial_id", "gains", "mission", "context", "media"?}]}
Same inputs give the same bytes. Standard library only; nothing here calls the API.

    build_phase_r_campaign.py --stage a0|a1|a2 [--out jobs.json]
    build_phase_r_campaign.py --stage holdout|b --selected <knob> [--out jobs.json]
    build_phase_r_campaign.py --stage a0b|holdout2|a0c --selected <knob> [--out jobs.json]  # #102
    build_phase_r_campaign.py --stage m3-0|m3-a|m3-b [--out jobs.json]  # M3, #109
    build_phase_r_campaign.py --stage <stage> [--selected <knob>] --rerun <trial_id>   # <trial_id>-rerun1 alone
    build_phase_r_campaign.py --write-missions    # regenerate the mission files from the definitions below

Mission poses are authored in the controller frame from the station dock point exactly as M1 (the M1 example
request's context.pose_derivation): the station dock point is at Stonefish world (3.59, 0.325, 3.91); the AUV dock
point is 0.395 m ahead of and 0.13 m below cg_link for a level AUV heading north; world_ned is x_C = -y_W,
y_C = x_W + 3.8, yaw_C = yaw_W + 1.571. For a stand-off s (dock point to dock point along the approach axis, x_W)
that is cg_link at x_C = -0.325, y_C = 6.995 - s, z_C = 3.78, yaw_C = 1.571; lateral, depth and yaw offsets add
to x_C, z_C and yaw_C (yaw turns about cg_link, which the controller regulates).

Floor clearance (#102): every pose of a mission a stage builds must keep the vehicle at least 0.3 m above the tank's
interior floor, or the build fails. The geometry comes from campaigns/tank-floor-v1.json, which
extract_tank_floor.py writes from the pinned world_of_stonefish: the interior floor triangles (world), the vehicle's
horizontal extent and its lowest point below the Base origin (cg_link is 0.7 m behind it at the same height). The
clearance of a pose is the smallest floor height under the vehicle's footprint, widened by 0.3 m for tracking and
drift, minus the depth of its lowest point, for a level vehicle at the commanded pose. The CLI prints it.
M-depth-hold v1 (+0.5 m) fails this: its deeper pose rested on the floor in both M2 holdout runs; it stays in the
record for the stage that ran it (holdout) and is not used by new stages.

M3 (piccard-experiments #88, piccard-physical-ai #109): the planner mission (piccard.race-auv.planner-mission/v1) is
the example request's (examples/m3-planner-request.json: M1's dive as the fallback pose, the planner values proposed
on #88) with black_square_edge tags, at the M2 selection p10. Stage m3-0 is the unscored dev run; m3-a interleaves two
planner runs with two runs of M2's precomputed contact mission at the same stand-offs (black_square_edge); m3-b runs
the planner once each with the lab's tag sizes, speed caps 0.05 and 0.2 m/s, and N gains. Planner stages end on
settling, so every M3 jobs file gets the fixed horizon M3_HORIZON_S. The floor check covers the fallback pose and the
nominal pose of every stand-off. --controller-variant keep_xy_integral emits the same M3 jobs for Piccard's controller
variant (M3-C): the variant recipe, context.arm suffixed _keepxyint, context.controller_variant and kxi trial ids.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "packages/simulation/race-auv-docking"
CAMPAIGN_DIR = PACKAGE / "campaigns/phase-r-a-20260928"
MISSION_DIR = CAMPAIGN_DIR / "missions"
EXAMPLE_REQUEST = PACKAGE / "examples/m1-smoke-request.json"
PLANNER_EXAMPLE = PACKAGE / "examples/m3-planner-request.json"
TANK_FLOOR = PACKAGE / "campaigns/tank-floor-v1.json"
FLOOR_CLEARANCE_MIN_M = 0.3
FOOTPRINT_MARGIN_M = 0.3  # horizontal allowance around the commanded footprint (tracking error, frame drift)
FOOTPRINT_STEP_M = 0.1

CAMPAIGN_ID = "phase-r-a-20260928"
RECIPE = "race-auv-docking/v1"
PROTOCOL = "piccard-experiments#87 M2 preregistration: Phase R-A v0.1 (2026-09-28), amendment v0.2 (2026-09-28)"
STATION_DOCK_W = (3.59, 0.325, 3.91)
CG_BEHIND_DOCK_M, CG_ABOVE_DOCK_M = 0.395, 0.13
START_STANDOFF_M = 7.0  # the M1 dive pose (y_C 0.0): the start position, 7 m in front of the station
HORIZON_MARGIN_S = 30  # M1: 300 s of dwells, horizon 330 s
WALL_MARGIN_S = 150  # M1: horizon 330 s, wall timeout 480 s (startup and readiness included)
DERIVATION = ("cg_link in controller world_ned from the station dock point (Stonefish world 3.59, 0.325, 3.91) as "
              "the M1 example: AUV dock point = station dock point minus the stand-off along x_W; cg_link 0.395 m "
              "behind and 0.13 m above it for a level AUV heading north; x_C = -y_W, y_C = x_W + 3.8, "
              "yaw_C = yaw_W + 1.571; so x_C = -0.325 + lateral, y_C = 6.995 - stand-off, z_C = 3.78 + depth offset, "
              "yaw_C = 1.571 + yaw offset (a yaw step turns about cg_link)")

# Docking x/y knobs (the protocol's R-A1 probes); flight and docking z/roll/pitch/yaw never change.
KNOBS = {"n": {}, "p2p5": {"p": 2.5}, "p10": {"p": 10.0}, "i0": {"i": 0.0}, "i0p5": {"i": 0.5},
         "v10": {"v": 10.0}, "v20": {"v": 20.0}}
PROBES = ("p2p5", "p10", "i0", "i0p5", "v10", "v20")
STAGE_TOKENS = {"holdout": "ho", "holdout2": "ho2", "m3-0": "m30", "m3-a": "m3a", "m3-b": "m3b"}  # pr-ho-<knob>-...
M3_CAMPAIGN_ID = "m3-planner"
M3_PROTOCOL = ("piccard-inc/piccard-experiments#88 M3 preregistration: planner on the tag-fused dock point, v1.5 "
               "(the planner values of examples/m3-planner-request.json; the set point walks at speed_cap_mps in every "
               "stage, the final stage follows the live estimate in x/y, vertical_band_m before it, every stage's "
               "vertical target approach_clearance_m above the station dock point, refinement held to the deadbands, "
               "final-stage re-targets per axis; the along axis approaches on command staleness, is re-issued at the "
               "goal on arrival and then holds on it; docked is the first full hold in the final stage)")
M3_PROTOCOL_VERSION = "v1.5"  # every M3 job context carries it; race_m3_metrics reports it per trial
M3_GAINS = "p10"  # the M2 selection (#87)
# Controller arms (piccard-inc/piccard-experiments#88, M3-C): the recipe image's lab controller, and Piccard's
# keep_xy_integral variant image (Dockerfile.controller-variant). A variant jobs file names its recipe, suffixes
# context.arm, records context.controller_variant and puts a token in the trial ids; the default jobs are unchanged.
# M3-C was retired after its M3-0 on protocol v1.4 (no set-point change clears the windup under it); kept on record.
CONTROLLER_VARIANTS = {"upstream": None,
                       "keep_xy_integral": {"recipe": "race-auv-docking-keepxyint/v1", "arm_suffix": "keepxyint",
                                            "token": "kxi"}}
M3_HORIZON_S = 1800  # v1.1; planner stages end on settling, so the horizon only bounds the trial
FOLLOW_UP = "piccard-physical-ai#102"
SELECTED_STAGES = ("holdout", "b", "a0b", "holdout2", "a0c")  # stages that take --selected
# Missions kept for the record only: the stages that ran them rebuild them, nothing new may use them.
RECORDED_ONLY = {"depth": ("holdout", "M-depth-hold v1: the +0.5 m pose rests on the tank floor (#102); use depth2")}


def pose(label: str, dwell_s: float, standoff_m: float, lateral_m: float = 0.0, depth_m: float = 0.0,
         yaw_rad: float = 0.0) -> dict:
    """A pose-mission pose from a stand-off and offsets (see the module docstring)."""
    return {"label": label, "x_m": round(-STATION_DOCK_W[1] + lateral_m, 3),
            "y_m": round(STATION_DOCK_W[0] - standoff_m - CG_BEHIND_DOCK_M + 3.8, 3),
            "z_m": round(STATION_DOCK_W[2] - CG_ABOVE_DOCK_M + depth_m, 3),
            "roll_rad": 0.0, "pitch_rad": 0.0, "yaw_rad": round(1.571 + yaw_rad, 3), "dwell_s": dwell_s}


def dive() -> dict:
    return {**pose("dive", 40, START_STANDOFF_M), "y_m": 0.0}  # exactly M1's dive pose


STAGED = [dive(), pose("approach_3m", 120, 3.0), pose("approach_1p5m", 120, 1.5)]
MISSIONS = {
    # key: (file, mission_id, question, poses, role)
    "1step": ("m-approach-1step-v1.json", "phase-r-a-m-approach-1step-v1",
              "Phase R-A reference (the M1 mission): one step from the 7 m dive to the 1.5 m stand-off, dwell 200 s, "
              "hold 60 s; how far does the dock point overshoot and how long does it take to settle?",
              [dive(), {**pose("approach", 200, 1.5), "x_m": -0.326}, {**pose("hold_1p5m", 60, 1.5), "x_m": -0.326}],
              "reference; R-A0 isolation pairs and the staged-vs-single comparison"),
    "staged": ("m-approach-staged-v1.json", "phase-r-a-m-approach-staged-v1",
               "Phase R-A selection mission: 7 m -> 3.0 m (dwell 120 s) -> 1.5 m (dwell 120 s), hold 60 s; does "
               "staging keep the dock point clear of the station and how does each docking x/y knob change "
               "overshoot and settling?",
               STAGED + [pose("hold_1p5m", 60, 1.5)], "R-A1 probes and selection; R-A2"),
    "lateral": ("m-lateral-yaw-v1.json", "phase-r-a-m-lateral-yaw-v1",
                "Phase R-A held-out mission: at the 1.5 m stand-off, lateral steps x_C +0.5 m and -0.5 m and yaw "
                "steps +0.3 rad and -0.3 rad (dwell 90 s each), return, hold 60 s; does the selected set's "
                "ranking against N transfer?",
                STAGED + [pose("lateral_plus", 90, 1.5, lateral_m=0.5), pose("lateral_minus", 90, 1.5, lateral_m=-0.5),
                          pose("lateral_center", 90, 1.5), pose("yaw_plus", 90, 1.5, yaw_rad=0.3),
                          pose("yaw_minus", 90, 1.5, yaw_rad=-0.3), pose("return", 90, 1.5),
                          pose("hold_1p5m", 60, 1.5)],
                "R-A1 holdout (never used for selection)"),
    "depth": ("m-depth-hold-v1.json", "phase-r-a-m-depth-hold-v1",
              "Phase R-A held-out mission: at the 1.5 m stand-off, depth steps +0.5 m and -0.5 m (dwell 90 s each) "
              "and a 300 s hold; does the selected set's ranking against N transfer?",
              STAGED + [pose("depth_plus", 90, 1.5, depth_m=0.5), pose("depth_minus", 90, 1.5, depth_m=-0.5),
                        pose("hold_1p5m", 300, 1.5)],
              "R-A1 holdout (never used for selection)"),
    # v2 (#102): the deeper step is +0.2 m, not v1's +0.5 m, which put the vehicle on the tank floor in both M2 holdout
    # runs; +0.2 m keeps it at least 0.3 m above the floor (FLOOR_CLEARANCE_MIN_M). The -0.5 m step and the 300 s
    # hold are unchanged.
    "depth2": ("m-depth-hold-v2.json", "phase-r-a-m-depth-hold-v2",
               "Phase R-A held-out mission, v2 (piccard-physical-ai#102): at the 1.5 m stand-off, depth steps +0.2 m "
               "and -0.5 m (dwell 90 s each) and a 300 s hold; v1's +0.5 m step rested on the tank floor. Does the "
               "selected set's ranking against N transfer?",
               STAGED + [pose("depth_plus", 90, 1.5, depth_m=0.2), pose("depth_minus", 90, 1.5, depth_m=-0.5),
                         pose("hold_1p5m", 300, 1.5)],
               "R-A1 holdout rerun (never used for selection)"),
    "contact": ("m-contact-v1.json", "phase-r-a-m-contact-v1",
                "Phase R-B contact-seeking terminal pose: staged approach to the 0.3 m stand-off (dwell 120 s), "
                "then dock point on dock point (stand-off 0, dwell 120 s); when, how fast and how aligned is first "
                "contact, and does contact persist?",
                STAGED + [pose("approach_0p3m", 120, 0.3), pose("contact", 120, 0.0)],
                "R-B only (contact is the goal)"),
}


def mission_document(key: str, tag_size: str = "lab_configured") -> dict:
    _, mission_id, question, poses, _ = MISSIONS[key]
    return {"schema": "piccard.race-auv.pose-mission/v1", "mission_id": mission_id, "question": question,
            "frame_id": "world_ned", "child_frame_id": "cg_link", "seed": None, "apriltag_tag_size": tag_size,
            "poses": copy.deepcopy(poses)}


def mission_bytes(mission: dict) -> bytes:
    """The runtime's serialization. The context's mission_sha256 is the sha256 of these bytes. trial.json's differs
    when the submit path re-renders numbers (it writes 0.0 as 0); race_m2_metrics accepts either form."""
    return (json.dumps(mission, indent=2, sort_keys=True) + "\n").encode()


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def load_mission(key: str) -> tuple[dict, str]:
    """A committed mission file, checked against its definition (the files are generated, never hand-edited)."""
    path = MISSION_DIR / MISSIONS[key][0]
    data = path.read_bytes()
    if data != mission_bytes(mission_document(key)):
        raise SystemExit(f"{path} differs from its definition; run --write-missions")
    return json.loads(data), sha256(data)


def floor_z(triangles, x: float, y: float):
    """Height (world z, down) of the interior floor under (x, y); None outside it."""
    for a, b, c in triangles:
        den = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(den) < 1e-12:
            continue
        u = ((b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])) / den
        v = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / den
        if u >= -1e-9 and v >= -1e-9 and u + v <= 1 + 1e-9:
            return u * a[2] + v * b[2] + (1 - u - v) * c[2]
    return None


def floor_clearance(pose: dict, geometry: dict | None = None) -> float:
    """Metres between the floor and the vehicle's lowest point, over its footprint widened by FOOTPRINT_MARGIN_M at the
    commanded pose (level vehicle). The pose is cg_link in the controller frame: x_W = y_C - 3.8, y_W = -x_C,
    z_W = z_C, yaw_W = yaw_C - 1.571. Widened samples beyond the interior (at a wall) are skipped; SystemExit if the
    footprint itself leaves the interior floor."""
    geometry = geometry or json.loads(TANK_FLOOR.read_text())
    vehicle, triangles = geometry["vehicle"], geometry["interior_floor_triangles_world_m"]
    yaw = pose["yaw_rad"] - 1.571
    heading, side = (math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw))
    base = (pose["y_m"] - 3.8 + vehicle["cg_link_behind_base_m"] * heading[0],
            -pose["x_m"] + vehicle["cg_link_behind_base_m"] * heading[1])
    (fx0, fx1), (fy0, fy1) = vehicle["footprint_base_m"]["x"], vehicle["footprint_base_m"]["y"]
    x0, x1, y0, y1 = fx0 - FOOTPRINT_MARGIN_M, fx1 + FOOTPRINT_MARGIN_M, fy0 - FOOTPRINT_MARGIN_M, fy1 + FOOTPRINT_MARGIN_M
    nx, ny = max(1, math.ceil((x1 - x0) / FOOTPRINT_STEP_M)), max(1, math.ceil((y1 - y0) / FOOTPRINT_STEP_M))
    lowest = pose["z_m"] + vehicle["lowest_point_below_base_m"]
    clearance = math.inf
    for i in range(nx + 1):
        for j in range(ny + 1):
            fx, fy = x0 + (x1 - x0) * i / nx, y0 + (y1 - y0) * j / ny
            wx, wy = base[0] + fx * heading[0] + fy * side[0], base[1] + fx * heading[1] + fy * side[1]
            z = floor_z(triangles, wx, wy)
            if z is None:
                if fx0 <= fx <= fx1 and fy0 <= fy <= fy1:
                    raise SystemExit(f"pose {pose['label']}: the vehicle's footprint leaves the tank interior at "
                                     f"world ({wx:.2f}, {wy:.2f})")
                continue  # the widened band reaches a wall
            clearance = min(clearance, z - lowest)
    return clearance


def check_floor(mission: dict, geometry: dict | None = None) -> list[tuple[str, float]]:
    """(label, clearance) per pose; SystemExit if any pose is closer than FLOOR_CLEARANCE_MIN_M to the floor."""
    geometry = geometry or json.loads(TANK_FLOOR.read_text())
    rows = [(p["label"], floor_clearance(p, geometry)) for p in floor_poses(mission)]
    low = [(label, value) for label, value in rows if value < FLOOR_CLEARANCE_MIN_M]
    if low:
        raise SystemExit(f"{mission['mission_id']}: floor clearance below {FLOOR_CLEARANCE_MIN_M} m at "
                         + ", ".join(f"{label} {value:.3f} m" for label, value in low))
    return rows


def gains_for(knob: str) -> dict:
    """The M1 example gains (flight = upstream config_sim.yaml, docking z/roll/pitch/yaw = flight, docking x/y = N)
    with the knob applied to docking x and y only."""
    gains = json.loads(EXAMPLE_REQUEST.read_text())["gains"]
    for axis in ("x", "y"):
        gains["docking"][axis].update(KNOBS[knob])
    return gains


def is_planner(mission: dict) -> bool:
    return mission["schema"] == "piccard.race-auv.planner-mission/v1"


def horizon(mission: dict) -> int:
    if is_planner(mission):
        return M3_HORIZON_S
    return int(sum(p["dwell_s"] for p in mission["poses"])) + HORIZON_MARGIN_S


def floor_poses(mission: dict) -> list[dict]:
    """The poses the floor check covers: a pose mission's poses; a planner mission's fallback pose and the nominal
    pose of each stand-off (the M2 derivation, which the planner's goal equals when the fused estimate is exact)."""
    if not is_planner(mission):
        return mission["poses"]
    return [mission["fallback_pose"]] + [pose(f"standoff_{s:g}m", 1, s) for s in mission["planner"]["standoffs_m"]]


def planner_mission(tag_size: str = "black_square_edge", **changes) -> dict:
    """The M3 planner mission: the example request's, with its tag size and any planner values changed."""
    mission = json.loads(PLANNER_EXAMPLE.read_text())["mission"]
    mission["apriltag_tag_size"] = tag_size
    mission["planner"].update(changes)
    mission["mission_id"] += "".join(f"-{key.replace('_', '-')}-{value:g}" for key, value in sorted(changes.items()))
    mission["mission_id"] += "-lab-tags" if tag_size == "lab_configured" else ""
    return mission


def control_mission() -> dict:
    """M3-A's control: M2's precomputed contact mission (the same stand-offs as the planner) with black_square_edge."""
    mission = mission_document("contact", "black_square_edge")
    mission["mission_id"] += "-black-square-edge"
    return mission


def job(stage: str, knob: str, key: str, repeat: int, role: str, variant: str = "", media: dict | None = None,
        mission: dict | None = None, arm: str | None = None) -> dict:
    mission, mission_sha = (mission, sha256(mission_bytes(mission))) if mission else load_mission(key)
    trial_id = f"pr-{STAGE_TOKENS.get(stage, stage)}-{knob}-{key}{'-' + variant if variant else ''}-r{repeat}"
    campaign, protocol = (M3_CAMPAIGN_ID, M3_PROTOCOL) if arm else (CAMPAIGN_ID, PROTOCOL)
    context = {"schema": "piccard.race-auv.phase-r-a-context/v1", "trial_id": trial_id, "campaign_id": campaign,
               "protocol": protocol, **({"protocol_version": M3_PROTOCOL_VERSION} if arm else {}),
               "stage": stage, "gain_set": knob, "docking_xy_overrides": KNOBS[knob], "mission_key": key,
               "mission_sha256": mission_sha, "role": role, "repeat": repeat, "pose_derivation": DERIVATION,
               "epistemic_class": "synthetic_simulation_development", **({"arm": arm} if arm else {})}
    item = {"trial_id": trial_id, "gains": gains_for(knob), "mission": mission, "context": context}
    if media:
        item["media"] = media
        context["media"] = media
    return item


def stage_a0() -> list[dict]:
    """8 trials: three display-video pairs and one onboard pair of M-approach-1step at N (not scored)."""
    jobs = []
    for repeat in (1, 2, 3):  # interleaved, so host drift hits both halves of a pair alike
        jobs.append(job("a0", "n", "1step", repeat, "isolation_display_without_video", "novideo"))
        jobs.append(job("a0", "n", "1step", repeat, "isolation_display_with_video", "video", {"display_video": True}))
    jobs.append(job("a0", "n", "1step", 1, "onboard_check_without_onboard", "noonboard"))
    jobs.append(job("a0", "n", "1step", 1, "onboard_check_with_onboard", "onboard", {"onboard_camera_video": True}))
    return jobs


def stage_a1() -> list[dict]:
    """9 trials: N, the six single-knob probes and an N repeat on M-approach-staged; N once on M-approach-1step."""
    jobs = [job("a1", "n", "staged", 1, "nominal")]
    jobs += [job("a1", knob, "staged", 1, "single_knob_probe") for knob in PROBES]
    jobs.append(job("a1", "n", "staged", 2, "nominal_repeat"))
    jobs.append(job("a1", "n", "1step", 1, "staged_vs_single_step"))
    return jobs


def stage_holdout(selected: str) -> list[dict]:
    if selected == "n":
        raise SystemExit("N ranked best in R-A1: there is no selected set to hold out (protocol: N stays)")
    return [job("holdout", knob, key, 1, "nominal_heldout" if knob == "n" else "selected_heldout")
            for key in ("lateral", "depth") for knob in ("n", selected)]


def stage_a0b(selected: str) -> list[dict]:
    """12 trials (#102): six display-video pairs of M-approach-staged at the selected gains, not scored. The order
    alternates within pairs (no-video, video, video, no-video, ...) so a host-order effect cannot pass for a recording
    effect. recording_isolation_check.py compares them, including the minimum-distance sign test."""
    jobs = []
    for repeat in range(1, 7):
        pair = [job("a0b", selected, "staged", repeat, "isolation_display_without_video", "novideo"),
                job("a0b", selected, "staged", repeat, "isolation_display_with_video", "video", {"display_video": True})]
        jobs += pair if repeat % 2 else pair[::-1]
    return follow_up(jobs)


def stage_a0c(selected: str) -> list[dict]:
    """2 trials (#102, closing #86): one onboard-camera pair of M-approach-staged at the selected gains, for a RACE
    image that carries the onboard recorder (media_capabilities includes onboard_camera_video; the M2 pin 9f06b6c5
    does not). Compared with recording_isolation_check.py (settling, timing, minimum distance) and
    race_onboard_load_check.py (cam_front / cam_down image delivery)."""
    return follow_up([
        job("a0c", selected, "staged", 1, "isolation_onboard_without_video", "novideo"),
        job("a0c", selected, "staged", 1, "isolation_onboard_with_video", "onboard",
            {"display_video": True, "onboard_camera_video": True})])


def stage_holdout2(selected: str) -> list[dict]:
    """2 trials (#102): the depth-hold holdout rerun on M-depth-hold v2, N and the selected set."""
    if selected == "n":
        raise SystemExit("N ranked best in R-A1: there is no selected set to hold out (protocol: N stays)")
    return follow_up([job("holdout2", knob, "depth2", 1, "nominal_heldout" if knob == "n" else "selected_heldout")
                      for knob in ("n", selected)])


def follow_up(jobs: list[dict]) -> list[dict]:
    for item in jobs:
        item["context"]["follow_up"] = FOLLOW_UP
    return jobs


def stage_a2() -> list[dict]:
    """2 trials: N on M-approach-staged with each tag-size convention (scoring by ground truth is unaffected)."""
    edge = mission_document("staged", "black_square_edge")
    edge["mission_id"] += "-black-square-edge"
    return [job("a2", "n", "staged", 1, "tag_size_lab_configured", "labtag"),
            job("a2", "n", "staged", 1, "tag_size_black_square_edge", "edgetag", mission=edge)]


def stage_b(selected: str) -> list[dict]:
    if selected == "n":
        return [job("b", "n", "contact", repeat, "contact_nominal") for repeat in (1, 2)]
    return [job("b", "n", "contact", 1, "contact_nominal"), job("b", selected, "contact", 1, "contact_selected")]


def stage_m3_0() -> list[dict]:
    """1 trial (#109): the planner on the arm64 dev loop, unscored; it also records the fused estimate against ground
    truth down to contact, which #88 asks for before v1.0 fixes the prediction."""
    return [job("m3-0", M3_GAINS, "planner", 1, "dev_unscored", mission=planner_mission(), arm="planner")]


def stage_m3_a() -> list[dict]:
    """4 trials: the planner twice and the precomputed contact mission twice, interleaved."""
    jobs = []
    for repeat in (1, 2):
        jobs.append(job("m3-a", M3_GAINS, "planner", repeat, "planner", mission=planner_mission(), arm="planner"))
        jobs.append(job("m3-a", M3_GAINS, "contact", repeat, "precomputed_control", mission=control_mission(),
                        arm="control"))
    return jobs


def stage_m3_b() -> list[dict]:
    """4 trials, the planner once each: the lab's tag sizes, speed caps 0.05 and 0.2 m/s, and N gains."""
    return [job("m3-b", M3_GAINS, "planner", 1, "tag_size_lab_configured", "labtag",
                mission=planner_mission("lab_configured"), arm="planner"),
            job("m3-b", M3_GAINS, "planner", 1, "speed_cap_0p05", "cap0p05", mission=planner_mission(speed_cap_mps=0.05),
                arm="planner"),
            job("m3-b", M3_GAINS, "planner", 1, "speed_cap_0p2", "cap0p2", mission=planner_mission(speed_cap_mps=0.2),
                arm="planner"),
            job("m3-b", "n", "planner", 1, "nominal_gains", mission=planner_mission(), arm="planner")]


STAGES = {"a0": stage_a0, "a1": stage_a1, "holdout": stage_holdout, "a2": stage_a2, "b": stage_b, "a0b": stage_a0b,
          "holdout2": stage_holdout2, "a0c": stage_a0c, "m3-0": stage_m3_0, "m3-a": stage_m3_a, "m3-b": stage_m3_b}


def build(stage: str, selected: str | None = None, controller_variant: str = "upstream") -> dict:
    if stage in SELECTED_STAGES and selected not in KNOBS:
        raise SystemExit(f"--selected must be one of {sorted(KNOBS)} for stage {stage}")
    if controller_variant not in CONTROLLER_VARIANTS:
        raise SystemExit(f"--controller-variant must be one of {sorted(CONTROLLER_VARIANTS)}")
    variant = CONTROLLER_VARIANTS[controller_variant]
    if variant and not stage.startswith("m3-"):
        raise SystemExit(f"controller variant {controller_variant} is an M3 arm (piccard-inc/piccard-experiments#88 "
                         f"M3-C); stage {stage} is not M3")
    jobs = STAGES[stage](selected) if stage in SELECTED_STAGES else STAGES[stage]()
    for item in jobs if variant else ():
        context = item["context"]
        item["trial_id"] = context["trial_id"] = re.sub(r"-r(\d+)$", rf"-{variant['token']}-r\1", item["trial_id"])
        context["arm"] = f"{context['arm']}_{variant['arm_suffix']}"
        context["controller_variant"] = controller_variant
    geometry = json.loads(TANK_FLOOR.read_text())
    for key in {item["context"]["mission_key"] for item in jobs}:
        recorded = RECORDED_ONLY.get(key)
        if recorded and stage != recorded[0]:
            raise SystemExit(f"mission {key} is kept for the record only ({recorded[1]})")
    for item in jobs:
        if item["context"]["mission_key"] not in RECORDED_ONLY:
            check_floor(item["mission"], geometry)
    longest = max(horizon(item["mission"]) for item in jobs)  # one horizon per jobs file; shorter missions end early
    return {"recipe": variant["recipe"] if variant else RECIPE, "horizon_s": longest,
            "wall_timeout_s": longest + WALL_MARGIN_S, "jobs": jobs}


def rerun(stage: str, selected: str | None, trial_id: str, controller_variant: str = "upstream") -> dict:
    """The one re-run of an excluded trial (audit problem, non-finite stop, route incomplete): the same job as
    <trial_id>-rerun1, with rerun_of in its context."""
    original = build(stage, selected, controller_variant)
    item = next((copy.deepcopy(j) for j in original["jobs"] if j["trial_id"] == trial_id), None)
    if item is None:
        raise SystemExit(f"{trial_id} is not a job of stage {stage}")
    item["trial_id"] = item["context"]["trial_id"] = f"{trial_id}-rerun1"
    item["context"]["rerun_of"] = trial_id
    return {**original, "jobs": [item]}  # the stage's horizon, so the re-run matches the trial it replaces


def write_missions() -> None:
    MISSION_DIR.mkdir(parents=True, exist_ok=True)
    for key, (name, *_rest) in MISSIONS.items():
        if key not in RECORDED_ONLY:
            check_floor(mission_document(key))
        (MISSION_DIR / name).write_bytes(mission_bytes(mission_document(key)))


def clearance_report(document: dict) -> str:
    """The floor clearance of each distinct mission in a jobs file, printed by the CLI (stderr)."""
    geometry, lines, seen = json.loads(TANK_FLOOR.read_text()), [], set()
    for item in document["jobs"]:
        mission = item["mission"]
        if mission["mission_id"] in seen:
            continue
        seen.add(mission["mission_id"])
        rows = [(p["label"], floor_clearance(p, geometry)) for p in floor_poses(mission)]
        label, value = min(rows, key=lambda row: row[1])
        note = " (recorded v1, not for new stages)" if item["context"]["mission_key"] in RECORDED_ONLY else ""
        lines.append(f"floor clearance {mission['mission_id']}: minimum {value:.3f} m at {label}{note}")
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stage", choices=tuple(STAGES))
    parser.add_argument("--selected", help=", ".join(SELECTED_STAGES) + ": the selected knob (" + ", ".join(KNOBS) + ")")
    parser.add_argument("--out", type=Path, help="write the jobs file here instead of stdout")
    parser.add_argument("--rerun", metavar="TRIAL_ID", help="emit only this job of the stage, as <trial_id>-rerun1")
    parser.add_argument("--write-missions", action="store_true", help="regenerate the mission files and exit")
    parser.add_argument("--controller-variant", choices=tuple(CONTROLLER_VARIANTS), default="upstream",
                        help="M3 stages: the controller arm (piccard-experiments#88 M3-C); keep_xy_integral "
                             "names the variant recipe")
    args = parser.parse_args(argv)
    if args.write_missions:
        write_missions()
        return 0
    if not args.stage:
        parser.error("--stage is required")
    document = (rerun(args.stage, args.selected, args.rerun, args.controller_variant) if args.rerun
                else build(args.stage, args.selected, args.controller_variant))
    sys.stderr.write(clearance_report(document))
    text = json.dumps(document, indent=2, sort_keys=True) + "\n"
    if args.out:
        args.out.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
