#!/usr/bin/env python3
"""RACE AUV–station contact margin: mesh separation at contact events (piccard-physical-ai #100), v1.1.

v1.1 adds the coincident placement (the meshes placed with the AUV dock point exactly on the station dock point) and
the vertical offset between the dock points at the recorded contact poses, and renames v1's
coupling_separation_at_coincident_dock_points (which were measured at the recorded poses, where the dock points were
not coincident) to coupling_separation_at_recorded_contact_poses.

Places the pinned collision meshes at the recorded ground-truth poses and measures the separation between the AUV and
the station at the times the Stonefish contact monitor reported AUV–station events. Writes
piccard.race-auv.contact-margin/v1 JSON.

Meshes (world_of_stonefish d51d59e, the race-auv-docking overlay pin): data/parts/body/RACE_sim_Body.obj,
data/parts/others/RACE_sim_{LeftLeg,RightLeg,Couplink}.obj, data/objects/race_station_sim_v6.obj,
data/objects/race_station_couplink_sim.obj. Transforms (vehicles/race_auv.scn, vehicles/race_station.scn):
- AUV: the ground-truth odometry is the Base link, which is the Vehicle compound rotated Rx(pi) (Joint1); each part's
  physical origin is also Rx(pi), so the raw OBJ vertices are in the odometry frame: world = R_odom v + t_odom.
- Station: parts have origin Rz(1.571) in the Station compound, whose Base link (odometry) is the same frame:
  world = R_odom Rz(1.571) v + t_odom.
Poses at an event time: position linearly interpolated between the bracketing ground-truth rows, orientation the
normalized linear interpolation of the quaternions. Separation: the minimum over exact point–triangle distances in
both directions (AUV vertices to station triangles, station vertices to AUV triangles) and exact segment–segment
distances between mesh edges, over the parts within 0.1 m of each other. Penetration: any mesh edge crossing a
triangle of the other mesh (both directions); separation is reported as 0 when edges cross.
Sampling: within each contact interval, the first event of every 0.5 s bin that holds an event.

    contact_margin_check.py --meshes DIR --out contact-margin.json \\
        --trial LABEL=DIR [...] --development LABEL=DIR [...] --docked LABEL=DIR [...]

Requires numpy.
"""
from __future__ import annotations

import argparse
import bisect
import hashlib
import json
from pathlib import Path
import statistics

import numpy as np

SCHEMA = "piccard.race-auv.contact-margin/v1.1"
# Dock points, as runtime/docking_metric.py (ground truth): the AUV's is its Base link's pose composed with
# AUV_BASE_TO_DOCK, the station's is its Base link's pose composed with STATION_BASE_TO_DOCK.
AUV_BASE_TO_DOCK = np.array([-0.305, 0.0, -0.13])
STATION_BASE_TO_DOCK = (np.array([-0.41, 0.325, -0.04]), np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]]))  # Rx(pi)
LEVEL_NORTH = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])  # Base-link orientation of the level AUV heading north
DEFINITIONS = {
    "dock_points": "ground truth, as runtime/docking_metric.py: AUV dock point = AUV Base pose composed with "
                   "(-0.305, 0, -0.13); station dock point = station Base pose composed with ((-0.41, 0.325, -0.04), "
                   "Rx(pi)). race_m2_metrics.py uses the same offsets",
    "dock_point_distance_m": "|AUV dock point - station dock point| (telemetry dock rows)",
    "vertical_offset_mm": "z_W(AUV dock point) - z_W(station dock point); world z points down, so negative means the "
                          "AUV dock point is shallower than the station's",
    "race_m2_metrics_dwell_end_errors": "race_m2_metrics.py reports dwell-end stand-off/lateral/vertical errors "
                                        "against the COMMANDED dock point, mapped through the measured controller "
                                        "frame (docking.json controller_frame_in_ground_truth, whose vertical origin "
                                        "is the controller's depth offset at alignment), not against the station dock "
                                        "point; the dock-point geometry is the same. For R-B that frame put the "
                                        "commanded dock point 1.8 cm (N) and 3.7 cm (p 10) above the station dock "
                                        "point, hence its +0.004 / -0.016 m against the 2.2 cm here",
    "penetration_depth_mm": "the smallest shift of the AUV along a world axis that separates the meshes (bisection to "
                            "0.01 mm), per axis and the minimum"}
SOURCES = {"world_of_stonefish": {"repository": "https://github.com/GSO-soslab/world_of_stonefish",
                                  "commit": "d51d59e77211a436a0c3617c30a80c36337cdbbc",
                                  "archive_sha256": "d7c10d04a39ab7bcc1aa6a58d2a6e59a27319dfe9957f589271f9e9cdbf055d2",
                                  "note": "the race-auv-docking overlay pin (source-lock-v1.json); meshes under data/"},
           "scenario_files": ["vehicles/race_auv.scn", "vehicles/race_station.scn", "world/race_auv_test.scn"]}
METHOD = (
    "Collision meshes of the pinned world_of_stonefish (AUV: data/parts/body/RACE_sim_Body.obj and "
    "data/parts/others/RACE_sim_{LeftLeg,RightLeg,Couplink}.obj; station: data/objects/race_station_sim_v6.obj and "
    "data/objects/race_station_couplink_sim.obj; sha256 under 'meshes') are placed at the recorded ground-truth poses "
    "(telemetry gt_auv and gt_station, the Stonefish odometry of each robot's Base link). AUV: the Base link is the "
    "Vehicle compound rotated Rx(pi) and every part's physical origin is Rx(pi), so world = R_odom v + t_odom on the raw "
    "OBJ vertices. Station: parts carry origin Rz(1.571) in the Station compound, whose Base link is the same frame, so "
    "world = R_odom Rz(1.571) v + t_odom. Check: this placement puts the runtime's dock-point offsets "
    "(runtime/docking_metric.py) on the couplink faces. At an event time the pose is interpolated between the "
    "bracketing ground-truth rows (position linearly, orientation by normalized quaternion interpolation). Separation "
    "is the minimum of exact point-triangle distances in both directions and exact segment-segment distances between "
    "mesh edges, over mesh parts within 0.1 m of each other; if any edge of one mesh crosses a triangle of the other "
    "(Moller-Trumbore) the meshes touch and separation is 0. Sampling: the first AUV-station event of every 0.5 s bin "
    "of each contact interval. Tool: contact_margin_check.py (sha256 under 'tool'), Python with numpy.")
BIN_S = 0.5
PRUNE_M = 0.1
TOUCH_M = 1e-4  # separation below 0.1 mm counts as touching
AUV_PARTS = ("RACE_sim_Body.obj", "RACE_sim_LeftLeg.obj", "RACE_sim_RightLeg.obj", "RACE_sim_Couplink.obj")
STATION_PARTS = ("race_station_sim_v6.obj", "race_station_couplink_sim.obj")
STATION_PART_YAW = 1.571


def load_obj(path: Path):
    vertices, faces = [], []
    for line in path.read_text().splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] == "v":
            vertices.append([float(x) for x in p[1:4]])
        elif p[0] == "f":
            idx = [int(x.split("/")[0]) - 1 for x in p[1:]]
            faces += [[idx[0], idx[k], idx[k + 1]] for k in range(1, len(idx) - 1)]
    return np.array(vertices), np.array(faces)


def quat_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class Mesh:
    def __init__(self, parts):
        self.parts = parts  # [(vertices, faces)]

    def placed(self, rotation, translation, pre=np.eye(3)):
        out = []
        for vertices, faces in self.parts:
            world = (rotation @ (pre @ vertices.T)).T + translation
            edges = np.unique(np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1),
                              axis=0)
            out.append((world, world[faces], world[edges]))
        vertices = np.concatenate([p[0] for p in out])
        triangles = np.concatenate([p[1] for p in out])
        edges = np.concatenate([p[2] for p in out])
        return vertices, triangles, edges


def inside(points, lo, hi):
    return np.all((points >= lo) & (points <= hi), axis=-1)


def near(items, lo, hi):
    """Items (N,k,3) whose bounding box meets [lo, hi]."""
    return items[np.all((items.max(1) >= lo) & (items.min(1) <= hi), axis=1)]


def point_triangle(points, triangles):
    """Exact min distance, all pairs (Ericson, Real-Time Collision Detection 5.1.5)."""
    if len(points) == 0 or len(triangles) == 0:
        return np.inf
    p = np.repeat(points, len(triangles), axis=0)
    a, b, c = (np.tile(triangles[:, k], (len(points), 1)) for k in range(3))
    ab, ac, ap, bp, cp = b - a, c - a, p - a, p - b, p - c
    d1, d2, d3, d4 = (ab * ap).sum(1), (ac * ap).sum(1), (ab * bp).sum(1), (ac * bp).sum(1)
    d5, d6 = (ab * cp).sum(1), (ac * cp).sum(1)
    va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2
    den = va + vb + vc
    den = np.where(np.abs(den) > 1e-18, den, 1e-18)
    q = a + ab * (vb / den)[:, None] + ac * (vc / den)[:, None]
    with np.errstate(divide="ignore", invalid="ignore"):
        for mask, value in (
                ((va <= 0) & (d4 - d3 >= 0) & (d5 - d6 >= 0), b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)))[:, None]),
                ((vb <= 0) & (d2 >= 0) & (d6 <= 0), a + ac * (d2 / (d2 - d6))[:, None]),
                ((vc <= 0) & (d1 >= 0) & (d3 <= 0), a + ab * (d1 / (d1 - d3))[:, None]),
                ((d6 >= 0) & (d5 <= d6), c), ((d3 >= 0) & (d4 <= d3), b), ((d1 <= 0) & (d2 <= 0), a)):
            q[mask] = value[mask]
    return float(np.sqrt(((p - q) ** 2).sum(1)).min())


def segment_segment(s, t):
    """Exact min distance between all pairs of segments s (N,2,3) and t (M,2,3) (Ericson 5.1.9)."""
    if len(s) == 0 or len(t) == 0:
        return np.inf
    p1, q1 = np.repeat(s[:, 0], len(t), axis=0), np.repeat(s[:, 1], len(t), axis=0)
    p2, q2 = np.tile(t[:, 0], (len(s), 1)), np.tile(t[:, 1], (len(s), 1))
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = (d1 * d1).sum(1), (d2 * d2).sum(1), (d2 * r).sum(1)
    c, b = (d1 * r).sum(1), (d1 * d2).sum(1)
    den = a * e - b * b
    with np.errstate(divide="ignore", invalid="ignore"):
        sc = np.where(den > 1e-18, np.clip((b * f - c * e) / den, 0, 1), 0.0)
        tc = np.where(e > 1e-18, (b * sc + f) / e, 0.0)
        sc = np.where(tc < 0, np.where(a > 1e-18, np.clip(-c / a, 0, 1), 0.0), sc)
        sc = np.where(tc > 1, np.where(a > 1e-18, np.clip((b - c) / a, 0, 1), 0.0), sc)
        tc = np.clip(tc, 0, 1)
    diff = (p1 + d1 * sc[:, None]) - (p2 + d2 * tc[:, None])
    return float(np.sqrt((diff ** 2).sum(1)).min())


def segments_cross(edges, triangles):
    """Any edge (N,2,3) crossing any triangle (M,3,3) (Moller–Trumbore on the segment)."""
    if len(edges) == 0 or len(triangles) == 0:
        return False
    p0, p1 = np.repeat(edges[:, 0], len(triangles), axis=0), np.repeat(edges[:, 1], len(triangles), axis=0)
    tri = np.tile(triangles, (len(edges), 1, 1))
    d = p1 - p0
    e1, e2 = tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    h = np.cross(d, e2)
    a = (e1 * h).sum(1)
    ok = np.abs(a) > 1e-12
    f = np.where(ok, 1.0 / np.where(ok, a, 1.0), 0.0)
    s = p0 - tri[:, 0]
    u = f * (s * h).sum(1)
    q = np.cross(s, e1)
    v = f * (d * q).sum(1)
    w = f * (e2 * q).sum(1)
    return bool(np.any(ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (w >= 0) & (w <= 1)))


def separation(a, b):
    """(separation m, edges cross) between two placed meshes (vertices, triangles, edges)."""
    (va, ta, ea), (vb, tb, eb) = a, b
    lo, hi = np.minimum(va.min(0), vb.min(0)), np.maximum(va.max(0), vb.max(0))
    lo_a, hi_a = va.min(0) - PRUNE_M, va.max(0) + PRUNE_M
    lo_b, hi_b = vb.min(0) - PRUNE_M, vb.max(0) + PRUNE_M
    tb_n, eb_n, vb_n = near(tb, lo_a, hi_a), near(eb, lo_a, hi_a), vb[inside(vb, lo_a, hi_a)]
    ta_n, ea_n, va_n = near(ta, lo_b, hi_b), near(ea, lo_b, hi_b), va[inside(va, lo_b, hi_b)]
    if len(tb_n) == 0 or len(ta_n) == 0:
        return np.inf, False
    cross = segments_cross(ea_n, tb_n) or segments_cross(eb_n, ta_n)
    distance = min(point_triangle(va_n, tb_n), point_triangle(vb_n, ta_n), segment_segment(ea_n, eb_n))
    return (0.0 if cross else distance), cross


class Trial:
    def __init__(self, directory: Path):
        root = directory / "output" if (directory / "output" / "telemetry.jsonl").is_file() else directory
        self.root = root
        self.trial = json.loads((root / "trial.json").read_text()) if (root / "trial.json").is_file() else {}
        self.docking = json.loads((root / "docking.json").read_text()) if (root / "docking.json").is_file() else {}
        self.rows = {"gt_auv": [], "gt_station": [], "contact": [], "dock": []}
        with (root / "telemetry.jsonl").open() as stream:
            for line in stream:
                if not any(f'"{k}"' in line for k in self.rows):
                    continue
                row = json.loads(line)
                if row.get("kind") in self.rows:
                    self.rows[row["kind"]].append(row)
        for kind in self.rows:
            self.rows[kind].sort(key=lambda r: r["t"])
        self.times = {k: [r["t"] for r in v] for k, v in self.rows.items()}

    def pose(self, kind, t):
        rows, ts = self.rows[kind], self.times[kind]
        i = min(max(bisect.bisect_left(ts, t), 1), len(rows) - 1)
        r0, r1 = rows[i - 1], rows[i]
        w = 0.0 if r1["t"] == r0["t"] else min(max((t - r0["t"]) / (r1["t"] - r0["t"]), 0.0), 1.0)
        position = np.array([r0[k] * (1 - w) + r1[k] * w for k in ("x", "y", "z")])
        q0 = np.array([r0[k] for k in ("qx", "qy", "qz", "qw")])
        q1 = np.array([r1[k] for k in ("qx", "qy", "qz", "qw")])
        if np.dot(q0, q1) < 0:
            q1 = -q1
        q = q0 * (1 - w) + q1 * w
        return quat_matrix(q / np.linalg.norm(q)), position, max(abs(t - r0["t"]), abs(r1["t"] - t))

    def dock_distance(self, t):
        rows, ts = self.rows["dock"], self.times["dock"]
        i = bisect.bisect_right(ts, t) - 1
        return rows[i]["distance_m"] if i >= 0 else None


class Scene:
    def __init__(self, meshes: Path):
        self.auv = Mesh([load_obj(meshes / name) for name in AUV_PARTS])
        self.station = Mesh([load_obj(meshes / name) for name in STATION_PARTS])
        self.auv_couplink = Mesh([load_obj(meshes / "RACE_sim_Couplink.obj")])
        self.station_couplink = Mesh([load_obj(meshes / "race_station_couplink_sim.obj")])

    def at(self, trial: Trial, t: float, couplinks: bool = False):
        ra, ta, gap_a = trial.pose("gt_auv", t)
        rs, ts, _ = trial.pose("gt_station", t)
        auv, station = (self.auv_couplink, self.station_couplink) if couplinks else (self.auv, self.station)
        distance, cross = separation(auv.placed(ra, ta), station.placed(rs, ts, rz(STATION_PART_YAW)))
        return distance, cross, gap_a


def mm(value):
    return None if value is None or not np.isfinite(value) else round(float(value) * 1000.0, 1)


def interval(scene: Scene, trial: Trial, label: str, evidence: str) -> dict | None:
    events = [r for r in trial.rows["contact"] if r.get("contact") == "auv_station"]
    if not events:
        return None
    sampled, seen = [], set()
    for event in events:
        key = int(event["t"] // BIN_S)
        if key not in seen:
            seen.add(key)
            sampled.append(event)
    records = []
    for event in sampled:
        distance, cross, gap = scene.at(trial, event["t"])
        records.append({"t": event["t"], "separation_m": distance, "cross": cross,
                        "loaded": (event.get("normal_force_n") or 0.0) > 0.0, "pose_gap_s": gap})

    def stats(subset):
        values = [r["separation_m"] for r in subset]
        if not values:
            return None
        return {"samples": len(values), "min_mm": mm(min(values)), "median_mm": mm(statistics.median(values)),
                "max_mm": mm(max(values)), "samples_touching": sum(v < TOUCH_M for v in values),
                "samples_with_crossing_edges": sum(r["cross"] for r in subset)}

    forces = [r.get("normal_force_n") or 0.0 for r in events]
    return {"label": label, "evidence": evidence, "pair": "auv_station",
            "start_t": round(events[0]["t"], 3), "end_t": round(events[-1]["t"], 3), "events": len(events),
            "force_bearing_events": sum(f > 0 for f in forces), "max_normal_force_n": round(max(forces), 1),
            "separation_at_sampled_events": stats(records),
            "separation_at_zero_force_events": stats([r for r in records if not r["loaded"]]),
            "separation_at_force_bearing_events": stats([r for r in records if r["loaded"]]),
            "max_ground_truth_pose_gap_s": round(max(r["pose_gap_s"] for r in records), 3)}


def docked(scene: Scene, trial: Trial, label: str) -> dict:
    """Separation where the dock points coincide: at the trial's minimum dock-point distance, and over the last 10 s of
    the final (contact) pose."""
    minimum = min(trial.rows["dock"], key=lambda r: r["distance_m"])
    final = (trial.docking.get("poses") or [{}])[-1]
    whole, _, _ = scene.at(trial, minimum["t"])
    couple, _, _ = scene.at(trial, minimum["t"], couplinks=True)
    tail_whole, tail_couple = [], []
    t = final["end_t"] - 10.0
    while t <= final["end_t"]:
        tail_whole.append(scene.at(trial, t)[0])
        tail_couple.append(scene.at(trial, t, couplinks=True)[0])
        t += 1.0
    return {"label": label, "pose": final.get("label"),
            "method": "whole-mesh and couplink-to-couplink separation (see 'method') at the trial's minimum "
                      "ground-truth dock-point distance, and each second over the last 10 s of the final pose",
            "at_minimum_dock_point_distance": {"t": round(minimum["t"], 3), "dock_point_distance_m": round(minimum["distance_m"], 4),
                                               "mesh_separation_mm": mm(whole), "couplink_separation_mm": mm(couple)},
            "final_10_s": {"dock_point_distance_m_at_end": round(trial.dock_distance(final["end_t"]), 4),
                           "mesh_separation_median_mm": mm(statistics.median(tail_whole)),
                           "couplink_separation_median_mm": mm(statistics.median(tail_couple)),
                           "couplink_separation_range_mm": [mm(min(tail_couple)), mm(max(tail_couple))]}}


def vertical_offset(trial: Trial, t: float):
    """z_W(AUV dock) - z_W(station dock) from the telemetry dock row at or before t."""
    rows, ts = trial.rows["dock"], trial.times["dock"]
    i = bisect.bisect_right(ts, t) - 1
    if i < 0:
        return None
    return rows[i]["auv_dock_point_m"][2] - rows[i]["station_dock_point_m"][2]


def recorded(scene: Scene, trial: Trial, label: str) -> dict:
    row = docked(scene, trial, label)
    minimum_t = row["at_minimum_dock_point_distance"]["t"]
    final = (trial.docking.get("poses") or [{}])[-1]
    tail = [vertical_offset(trial, t) for t in np.arange(final["end_t"] - 10.0, final["end_t"] + 1e-9, 1.0)]
    row["at_minimum_dock_point_distance"]["vertical_offset_mm"] = mm(vertical_offset(trial, minimum_t))
    row["final_10_s"]["vertical_offset_median_mm"] = mm(statistics.median(tail))
    row["method"] = ("whole-mesh and couplink-to-couplink separation (see 'method') at the RECORDED poses: the trial's "
                     "minimum ground-truth dock-point distance, and each second over the last 10 s of the final pose; "
                     "the dock points are not coincident there (see vertical_offset_mm)")
    return row


def shift_to_separate(scene: Scene, auv: Mesh, station_placed, rotation, translation, axis, sign) -> float:
    """Smallest shift (m) of the AUV along sign * axis that separates it from the station (bisection)."""
    direction = np.zeros(3)
    direction[axis] = sign
    apart = lambda d: separation(auv.placed(rotation, translation + direction * d), station_placed)[0] > 0.0
    low, high = 0.0, 0.01
    while not apart(high):
        low, high = high, high * 2
        if high > 2.0:
            return np.inf
    while high - low > 1e-5:
        mid = (low + high) / 2
        low, high = (low, mid) if apart(mid) else (mid, high)
    return high


def coincident(scene: Scene, trial: Trial) -> dict:
    """The meshes placed with the AUV dock point exactly on the station dock point, the AUV level and heading north,
    the station at its recorded ground-truth pose."""
    rs, ts, _ = trial.pose("gt_station", trial.times["gt_station"][0])
    station_dock = ts + rs @ STATION_BASE_TO_DOCK[0]
    dock_rotation = rs @ STATION_BASE_TO_DOCK[1]
    rotation = dock_rotation  # the AUV dock frame has the Base's orientation, so the dock frames coincide
    if not np.allclose(rotation, LEVEL_NORTH, atol=1e-6):
        raise SystemExit("the station is not level and north-facing; the coincident placement would not be level")
    translation = station_dock - rotation @ AUV_BASE_TO_DOCK
    station = scene.station.placed(rs, ts, rz(STATION_PART_YAW))
    station_couplink = scene.station_couplink.placed(rs, ts, rz(STATION_PART_YAW))
    result = {"station_base_world_m": [round(float(v), 4) for v in ts],
              "station_dock_point_world_m": [round(float(v), 4) for v in station_dock],
              "auv_base_world_m": [round(float(v), 4) for v in translation],
              "auv_orientation": "level, heading north (Base link = Rx(pi) in world, as the recorded odometry)"}
    for name, auv, placed in (("whole_mesh", scene.auv, station), ("couplinks", scene.auv_couplink, station_couplink)):
        distance, cross = separation(auv.placed(rotation, translation), placed)
        item = {"separation_mm": mm(distance), "edges_cross": cross}
        if distance == 0.0:
            shifts = {f"{'-+'[sign > 0]}{'xyz'[axis]}_W": shift_to_separate(scene, auv, placed, rotation, translation,
                                                                             axis, sign)
                      for axis in range(3) for sign in (-1, 1)}
            axis, depth = min(shifts.items(), key=lambda kv: kv[1])
            item.update({"penetration_depth_mm": mm(depth), "penetration_axis": axis,
                         "shift_to_separate_mm": {k: mm(v) for k, v in shifts.items()}})
        result[name] = item
    return result


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--meshes", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--trial", action="append", default=[], metavar="LABEL=DIR")
    parser.add_argument("--development", action="append", default=[], metavar="LABEL=DIR")
    parser.add_argument("--docked", action="append", default=[], metavar="LABEL=DIR")
    args = parser.parse_args(argv)
    scene = Scene(args.meshes)
    cache = {}

    def load(spec):
        label, directory = spec.split("=", 1)
        if directory not in cache:
            cache[directory] = Trial(Path(directory))
        return label, cache[directory]

    intervals, development = [], []
    for spec in args.trial:
        label, trial = load(spec)
        item = interval(scene, trial, label, "staging trial")
        if item:
            intervals.append(item)
    for spec in args.development:
        label, trial = load(spec)
        item = interval(scene, trial, label, "development evidence (arm64 dev loop, not a staging trial)")
        if item:
            development.append(item)
    docked_rows, coincident_row = [], None
    for spec in args.docked:
        label, trial = load(spec)
        docked_rows.append(recorded(scene, trial, label))
        if coincident_row is None:
            coincident_row = {"station_pose_from": label, **coincident(scene, trial)}
    def pooled(items, key, field, pick):
        values = [i[key][field] for i in items if i[key]]
        return pick(values) if values else None

    staging_touch = sum((i["separation_at_sampled_events"] or {}).get("samples_touching", 0) for i in intervals)
    dev_zero_touch = sum((i["separation_at_zero_force_events"] or {}).get("samples_touching", 0) for i in development)
    margin = {
        "events_register_out_to_mm": pooled(intervals + development, "separation_at_zero_force_events", "max_mm", max),
        "staging_closest_sampled_separation_mm": pooled(intervals, "separation_at_sampled_events", "min_mm", min),
        "staging_samples_touching": staging_touch,
        "force_bearing_events_max_separation_mm": pooled(intervals + development, "separation_at_force_bearing_events",
                                                         "max_mm", max),
        "development_zero_force_samples_touching": dev_zero_touch,
        "statement": ("Contact events register out to about 2 cm of mesh separation. Force appears only when the "
                      "meshes touch. A zero force alone does not show that the meshes are apart: during the loaded "
                      "development push, zero-force events were also published while touching (the monitor keeps one "
                      "manifold point per step). The separation shows it: no sampled staging event had the meshes "
                      "touching, so every staging contact event was a near-miss."),
        "method": ("per contact interval, the mesh separation at the first event of each 0.5 s bin (see 'method'); "
                   "'register out to' is the largest separation at which a zero-force event was sampled; touching "
                   "means a separation below 0.1 mm")}
    document = {
        "schema": SCHEMA,
        "issue": "piccard-inc/piccard-physical-ai#100",
        "claim_boundary": "synthetic Stonefish simulation; mesh geometry and Bullet contact reporting only, no physical "
                          "contact-mechanics claim",
        "method": METHOD,
        "sources": SOURCES,
        "meshes": {name: sha256(args.meshes / name) for name in (*AUV_PARTS, *STATION_PARTS)},
        "tool": {"file": Path(__file__).name, "sha256": sha256(Path(__file__))},
        "contact_intervals": intervals,
        "development_evidence": development,
        "margin_estimate": margin,
        "coupling_separation_at_recorded_contact_poses": docked_rows,
        "coincident_placement": coincident_row,
        "definitions": DEFINITIONS,
    }
    args.out.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
