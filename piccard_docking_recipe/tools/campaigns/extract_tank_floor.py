#!/usr/bin/env python3
"""Extract the RACE tank floor and the AUV's footprint from the pinned world (piccard-physical-ai #102).

The campaign builder asserts that no commanded pose puts the vehicle within 0.3 m of the tank floor. It reads the
geometry from packages/simulation/race-auv-docking/campaigns/tank-floor-v1.json, which this tool writes from a
world_of_stonefish checkout at the race-auv-docking overlay pin (source-lock-v1.json: d51d59e), so the builder needs
no simulator files at build time.

- Tank: world/race_auv_test.scn places data/objects/ORL_Tank.obj with <world_transform xyz rpy> (Stonefish applies
  R = Rz(yaw) Ry(pitch) Rx(roll)). The mesh is a double shell: an outer box (floor at z 5.10, the bound
  runtime/mission.py uses) and the inner box the vehicle swims in. The interior floor is the triangle a vertical ray
  from the vehicle's start position hits first, together with the triangles coplanar with it; the interior's
  horizontal extent is theirs.
- Vehicle: vehicles/race_auv.scn builds the Vehicle compound from external parts (mesh, physical <origin>,
  <compound_transform>) and places it with world_transform rpy 0, so the level Vehicle frame is aligned with world
  NED (z down). The odometry Base link sits at the Vehicle origin (Joint1 rotates it but does not translate it), and
  cg_link is 0.7 m behind it at the same height (runtime/docking_metric.py AUV_BASE_TO_CG_LINK). The tool records the
  parts' horizontal extent and their lowest point below the Base origin.

Standard library only.

    extract_tank_floor.py WORLD_OF_STONEFISH_DIR --commit <sha> --archive-sha256 <sha> --out tank-floor-v1.json
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

SCHEMA = "piccard.race-auv.tank-floor/v1"
SCENARIO = "world/race_auv_test.scn"
VEHICLE = "vehicles/race_auv.scn"
CG_LINK_BEHIND_BASE_M = 0.7  # runtime/docking_metric.py AUV_BASE_TO_CG_LINK
COPLANAR_M = 0.005


def triples(text: str) -> list[float]:
    return [float(v) for v in text.split()]


def rotation(rpy) -> list[list[float]]:
    """R = Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def transform(point, rpy, xyz):
    rot = rotation(rpy)
    return [sum(rot[i][k] * point[k] for k in range(3)) + xyz[i] for i in range(3)]


def load_obj(path: Path):
    vertices, faces = [], []
    for line in path.read_text().splitlines():
        parts = line.split()
        if parts and parts[0] == "v":
            vertices.append([float(v) for v in parts[1:4]])
        elif parts and parts[0] == "f":
            index = [int(v.split("/")[0]) - 1 for v in parts[1:]]
            faces += [[index[0], index[k], index[k + 1]] for k in range(1, len(index) - 1)]
    return vertices, faces


def mesh_path(root: Path, filename: str) -> Path:
    return root / "data" / filename


def pose_of(element, tag: str):
    node = element.find(tag)
    if node is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    return triples(node.get("rpy", "0 0 0")), triples(node.get("xyz", "0 0 0"))


def tank_triangles(root: Path):
    tank = next(s for s in ET.parse(root / SCENARIO).getroot().iter("static") if s.get("name") == "tank")
    physical = tank.find("physical")
    filename = physical.find("mesh").get("filename")
    origin = pose_of(physical, "origin")
    world = pose_of(tank, "world_transform")
    vertices, faces = load_obj(mesh_path(root, filename))
    placed = [transform(transform(v, *origin), *world) for v in vertices]
    return filename, world, [[placed[i] for i in face] for face in faces]


def ray_down(triangles, x, y, z_from=-math.inf):
    """(z, index) of the first triangle below z_from hit by a vertical ray through (x, y); None if none."""
    best = None
    for index, (a, b, c) in enumerate(triangles):
        den = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(den) < 1e-12:
            continue  # vertical triangle
        u = ((b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])) / den
        v = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / den
        if u < -1e-9 or v < -1e-9 or u + v > 1 + 1e-9:
            continue
        z = u * a[2] + v * b[2] + (1 - u - v) * c[2]
        if z >= z_from and (best is None or z < best[0]):
            best = (z, index)
    return best


def plane(triangle):
    a, b, c = triangle
    u = [b[i] - a[i] for i in range(3)]
    v = [c[i] - a[i] for i in range(3)]
    n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
    norm = math.sqrt(sum(x * x for x in n))
    n = [x / norm for x in n]
    return n, sum(n[i] * a[i] for i in range(3))


def interior_floor(triangles, start_xy):
    hit = ray_down(triangles, *start_xy)
    if hit is None:
        raise SystemExit("no tank floor below the vehicle's start position")
    normal, offset = plane(triangles[hit[1]])
    floor = [t for t in triangles
             if all(abs(sum(normal[i] * p[i] for i in range(3)) - offset) <= COPLANAR_M for p in t)]
    return floor


def vehicle_geometry(root: Path):
    robot = ET.parse(root / VEHICLE).getroot().find("robot")
    base_link = robot.find("base_link")
    start = pose_of(robot, "world_transform")
    points, parts = [], []
    for part in base_link.findall("external_part"):
        physical = part.find("physical")
        filename = physical.find("mesh").get("filename")
        origin, compound = pose_of(physical, "origin"), pose_of(part, "compound_transform")
        vertices, _ = load_obj(mesh_path(root, filename))
        points += [transform(transform(v, *origin), *compound) for v in vertices]
        parts.append(filename)
    return {"start_xy": start[1][:2], "start_rpy": start[0], "parts": parts,
            "footprint_base_m": {"x": [round(min(p[0] for p in points), 4), round(max(p[0] for p in points), 4)],
                                 "y": [round(min(p[1] for p in points), 4), round(max(p[1] for p in points), 4)]},
            "lowest_point_below_base_m": round(max(p[2] for p in points), 4)}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def extract(root: Path, commit: str, archive_sha256: str) -> dict:
    tank_mesh, world, triangles = tank_triangles(root)
    vehicle = vehicle_geometry(root)
    if any(abs(a) > 1e-9 for a in vehicle["start_rpy"]):
        raise SystemExit("the vehicle is not placed level (world_transform rpy != 0); the level frame is not world NED")
    floor = interior_floor(triangles, vehicle["start_xy"])
    xs = [p[0] for t in floor for p in t]
    ys = [p[1] for t in floor for p in t]
    files = [SCENARIO, VEHICLE, "data/" + tank_mesh, *("data/" + p for p in vehicle["parts"])]
    return {
        "schema": SCHEMA,
        "issue": "piccard-inc/piccard-physical-ai#102",
        "source": {"world_of_stonefish": {"repository": "https://github.com/GSO-soslab/world_of_stonefish",
                                          "commit": commit, "archive_sha256": archive_sha256},
                   "files_sha256": {name: sha256(root / name) for name in files}},
        "method": ("tank: " + tank_mesh + " placed by the scenario's world_transform (R = Rz Ry Rx); interior floor = "
                   "the triangle first hit by a vertical ray at the vehicle's start position plus the triangles "
                   "coplanar with it. vehicle: the Vehicle compound's external parts (physical origin, then "
                   "compound_transform), level Vehicle frame = world NED; footprint and lowest point relative to the "
                   "Base origin. Tool: tools/campaigns/extract_tank_floor.py"),
        "tank_world_transform": {"rpy": world[0], "xyz": world[1]},
        "interior_floor_triangles_world_m": [[[round(v, 4) for v in p] for p in t] for t in floor],
        "interior_xy_bounds_world_m": {"x": [round(min(xs), 4), round(max(xs), 4)],
                                       "y": [round(min(ys), 4), round(max(ys), 4)]},
        "vehicle": {"footprint_base_m": vehicle["footprint_base_m"],
                    "lowest_point_below_base_m": vehicle["lowest_point_below_base_m"],
                    "cg_link_behind_base_m": CG_LINK_BEHIND_BASE_M,
                    "start_xy_world_m": vehicle["start_xy"]},
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("world_of_stonefish", type=Path)
    parser.add_argument("--commit", required=True)
    parser.add_argument("--archive-sha256", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    document = extract(args.world_of_stonefish, args.commit, args.archive_sha256)
    args.out.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
