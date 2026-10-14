"""The pivot of the fused station pose (M3, piccard-physical-ai #109 follow-up; protocol piccard-experiments #88).

The fuser (race_auv_perception apriltag_fuser_node) solves the station pose from the tags the cameras detect. On the
approach only the forward camera sees tags, and the simulation's apriltag.yaml configures it for three tags, all on
the station's vertical centreline. Their centres pin the pose's position well, while yaw about that line is
unconstrained (three collinear centres), so rotation errors swing the fused dock point about their centroid. The
planner therefore re-anchors: it takes the fused pose's image of this centroid and places the dock point from it
with its own level heading.

The centroid is station construction, not station pose. The runtime derives it from the files the fuser itself
loads: apriltag.yaml's forward-camera tag list and object block, and the station URDF they name. The collector fails
the trial closed if the mission's tag_pivot_m differs from it by more than PIVOT_TOLERANCE_M. No ROS here; the
caller resolves package share directories.
"""
from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ElementTree
from pathlib import Path

FORWARD_CAMERA = "cam_front"
PIVOT_TOLERANCE_M = 0.001
APRILTAG_CONFIG = ("race_auv_bringup", "config/simulation/apriltag.yaml")  # the launch's default config_yaml


def rpy_matrix(roll: float, pitch: float, yaw: float):
    """URDF fixed-axis rpy: Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw),
                              math.sin(yaw))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def link_poses(urdf_text: str) -> dict:
    """{link: (rotation, position)} from the URDF root through fixed joints (the fuser's tags are fixed-jointed)."""
    root = ElementTree.fromstring(urdf_text)
    joints = {}
    for joint in root.iter("joint"):
        child, parent = joint.find("child").get("link"), joint.find("parent").get("link")
        origin = joint.find("origin")
        xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
        rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
        if child in joints:
            raise ValueError(f"link {child} has two parent joints")
        joints[child] = (parent, rpy_matrix(*rpy), xyz)
    poses = {}

    def pose(link: str, depth: int = 0):
        if link in poses:
            return poses[link]
        if depth > len(joints):
            raise ValueError("URDF joint loop")
        if link not in joints:
            poses[link] = ([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [0.0, 0.0, 0.0])
            return poses[link]
        parent, rotation, xyz = joints[link]
        p_rot, p_pos = pose(parent, depth + 1)
        poses[link] = ([[sum(p_rot[i][k] * rotation[k][j] for k in range(3)) for j in range(3)] for i in range(3)],
                       [p_pos[i] + sum(p_rot[i][k] * xyz[k] for k in range(3)) for i in range(3)])
        return poses[link]

    for link in [link.get("name") for link in root.iter("link")] + list(joints):
        pose(link)
    return poses


def in_frame(poses: dict, frame: str, link: str) -> list[float]:
    """The position of `link` in `frame`."""
    f_rot, f_pos = poses[frame]
    delta = [poses[link][1][i] - f_pos[i] for i in range(3)]
    return [sum(f_rot[k][i] * delta[k] for k in range(3)) for i in range(3)]


def layout(apriltag_config: dict, urdf_text: str) -> dict:
    """Every enabled camera's configured tags, positioned in the station's dock frame by the URDF:
    {"dock_link", "cameras": {name: {"tags": [{"tag", "family", "id", "size_m", "position_in_dock_m"}]}}}.
    size_m is the size as configured in this apriltag.yaml. apriltag_config: the parsed apriltag.yaml (with or
    without its top-level `apriltag` key)."""
    config = apriltag_config.get("apriltag", apriltag_config)
    obj = config.get("object") or {}
    dock = str(obj.get("dock_link_name") or "").strip()
    prefix = str(obj.get("tag_link_prefix") or "apriltag")
    if not dock:
        raise ValueError("apriltag.yaml names no object.dock_link_name")
    poses = link_poses(urdf_text)
    if dock not in poses:
        raise ValueError(f"URDF has no link {dock!r}")
    pattern = re.compile(rf"^{re.escape(prefix)}(\d+h\d+)_(\d+)$")
    links = {(f"tag{m.group(1)}", int(m.group(2))): link for link in poses for m in [pattern.match(link)] if m}
    cameras = {}
    for camera in config.get("cameras") or []:
        if not camera.get("enabled", True):
            continue
        name = camera.get("name")
        if name in cameras:
            raise ValueError(f"apriltag.yaml has two enabled {name!r} cameras")
        tags = []
        for tag in camera.get("tags") or config.get("tags") or []:
            key = (str(tag["family"]), int(tag["id"]))
            if key not in links:
                raise ValueError(f"URDF has no link for {key[0]}:{key[1]}, configured for {name}")
            tags.append({"tag": f"{key[0]}:{key[1]}", "family": key[0], "id": key[1], "size_m": tag.get("size"),
                         "position_in_dock_m": [round(v, 6) + 0.0 for v in in_frame(poses, dock, links[key])]})
        cameras[name] = {"tags": sorted(tags, key=lambda t: t["tag"])}
    return {"dock_link": dock, "cameras": cameras}


def derive(apriltag_config: dict, urdf_text: str, camera: str = FORWARD_CAMERA) -> dict:
    """The centroid, in the station's dock frame, of the tags `camera` is configured for."""
    placed = layout(apriltag_config, urdf_text)
    tags = (placed["cameras"].get(camera) or {}).get("tags")
    if not tags:
        raise ValueError(f"apriltag.yaml has no enabled {camera!r} camera with tags")
    found = {t["tag"]: t["position_in_dock_m"] for t in tags}
    centroid = [sum(p[i] for p in found.values()) / len(found) for i in range(3)]
    return {"pivot_m": [round(v, 6) + 0.0 for v in centroid], "camera": camera, "dock_link": placed["dock_link"],
            "tags": dict(sorted(found.items()))}


def installed_files(share_directory, yaml_module) -> tuple[dict, str, Path, Path]:
    """(the parsed apriltag.yaml, the URDF text, their paths) as the fuser loads them."""
    config_path = Path(share_directory(APRILTAG_CONFIG[0])) / APRILTAG_CONFIG[1]
    config = yaml_module.safe_load(config_path.read_text())
    obj = config.get("apriltag", config).get("object") or {}
    urdf_path = Path(share_directory(obj["urdf_package"])) / obj.get("urdf_filename", "urdf/base.urdf")
    return config, urdf_path.read_text(), config_path, urdf_path


def derive_installed(share_directory, yaml_module) -> dict:
    """derive() on the installed files the fuser loads. share_directory: ament's get_package_share_directory."""
    config, urdf_text, config_path, urdf_path = installed_files(share_directory, yaml_module)
    return {**derive(config, urdf_text), "apriltag_config": str(config_path), "urdf": str(urdf_path)}


def tag_record(share_directory, yaml_module, tag_size: str, black_edge_fraction: float) -> dict:
    """trial.json's tags: layout() of the installed files, each size in both conventions. The installed apriltag.yaml
    holds the mission's convention (prepare_candidate scales every size by black_edge_fraction for
    black_square_edge); the other follows from it."""
    config, urdf_text, config_path, urdf_path = installed_files(share_directory, yaml_module)
    result = layout(config, urdf_text)
    for camera in result["cameras"].values():
        for tag in camera["tags"]:
            size = tag.pop("size_m")
            black = size if tag_size == "black_square_edge" else size * black_edge_fraction
            lab = size / black_edge_fraction if tag_size == "black_square_edge" else size
            tag["size_m"] = {"installed": tag_size, "lab_configured": round(lab, 6),
                             "black_square_edge": round(black, 6)}
    return {**result, "apriltag_config": str(config_path), "urdf": str(urdf_path)}


def check(mission_pivot: list[float], derived: dict) -> dict:
    """The record trial.json keeps; `ok` is false when any component differs by more than PIVOT_TOLERANCE_M."""
    difference = max(abs(a - b) for a, b in zip(mission_pivot, derived["pivot_m"]))
    return {"mission_m": list(mission_pivot), "derived": derived, "max_difference_m": round(difference, 9),
            "tolerance_m": PIVOT_TOLERANCE_M, "ok": difference <= PIVOT_TOLERANCE_M}
