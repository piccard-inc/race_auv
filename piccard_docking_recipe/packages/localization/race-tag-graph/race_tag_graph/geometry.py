"""Small pose helpers shared by the builder, the dock-point fit and the evaluation. Quaternions are (x, y, z, w), as
ROS messages and the collector's records carry them."""
from __future__ import annotations

import bisect

import gtsam
import numpy as np


def rot3(q_xyzw) -> gtsam.Rot3:
    x, y, z, w = (float(v) for v in q_xyzw)
    return gtsam.Rot3.Quaternion(w, x, y, z)


def pose3(position, q_xyzw) -> gtsam.Pose3:
    return gtsam.Pose3(rot3(q_xyzw), np.asarray(position, dtype=float))


def xyzw(rotation: gtsam.Rot3) -> list[float]:
    q = rotation.toQuaternion()
    return [float(q.x()), float(q.y()), float(q.z()), float(q.w())]


def interpolate(a: gtsam.Pose3, b: gtsam.Pose3, fraction: float) -> gtsam.Pose3:
    """Linear in position, slerp in rotation."""
    rotation = a.rotation().slerp(fraction, b.rotation())
    return gtsam.Pose3(rotation, a.translation() + fraction * (b.translation() - a.translation()))


class PoseSeries:
    """Stamped poses, sorted, interpolated at any stamp inside their span (None outside it)."""

    def __init__(self, stamps, poses):
        order = sorted(range(len(stamps)), key=lambda i: stamps[i])
        self.stamps = [float(stamps[i]) for i in order]
        self.poses = [poses[i] for i in order]

    def __len__(self) -> int:
        return len(self.stamps)

    def at(self, stamp: float):
        if not self.stamps or stamp < self.stamps[0] or stamp > self.stamps[-1]:
            return None
        i = bisect.bisect_left(self.stamps, stamp)
        if self.stamps[i] == stamp:
            return self.poses[i]
        a, b = self.stamps[i - 1], self.stamps[i]
        return interpolate(self.poses[i - 1], self.poses[i], (stamp - a) / (b - a))


def static_chain(edges: list[dict], parent: str, child: str) -> gtsam.Pose3 | None:
    """parent -> child composed from static edges ({"parent", "child", "xyz", "q"}) by a walk up from the child."""
    by_child = {edge["child"]: edge for edge in edges}
    pose, frame, hops = gtsam.Pose3(), child, 0
    while frame != parent:
        edge = by_child.get(frame)
        if edge is None or hops > len(edges):
            return None
        pose = pose3(edge["xyz"], edge["q"]).compose(pose)
        frame, hops = edge["parent"], hops + 1
    return pose
