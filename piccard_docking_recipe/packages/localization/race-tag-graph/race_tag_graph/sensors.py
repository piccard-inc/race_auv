"""The estimator's only input: the vehicle's own sensor topics, behind an allowlist (#265, research #36).

The factor builder accepts nothing but a SensorStream, and a SensorStream holds only what these readers return:
EKF odometry, the cameras' 3D tag detections and the static transforms. Asking for anything else raises
TruthAccessError: the ground-truth topics, the simulator's odometry, the contact topics, the collector's truth
records (gt_*, dock, alignment) and the lab fuser's dock point, which is a comparison input for the evaluation, not
an estimator input. In a telemetry file the readers skip every other record by its kind before parsing it, so a truth
record is never decoded here; a record of an allowed kind on a forbidden topic raises.

Readers:
- read_telemetry: the Piccard collector's telemetry.jsonl (runtime/collect_trial.py). Tested.
- read_rosbag2: a ROS 2 bag (sqlite3 or mcap) through the rosbags library, for nav_msgs/msg/Odometry,
  vision_msgs/msg/Detection3DArray and tf2_msgs/msg/TFMessage. UNTESTED against a real bag until Yuewei's bag
  arrives; only its message conversions are unit-tested, on stand-in objects.
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path

EKF_ODOMETRY = "/race_auv/odometry/filtered"
DETECTION_TOPICS = {"/cam_front/apriltag_detection/detections3d": "cam_front",
                    "/cam_down/apriltag_detection/detections3d": "cam_down"}
TF_STATIC = "/tf_static"
SENSOR_TOPICS = frozenset({EKF_ODOMETRY, TF_STATIC, *DETECTION_TOPICS})
SENSOR_KINDS = frozenset({"odometry", "detections", "tf_static"})
KIND_TOPICS = {"odometry": {EKF_ODOMETRY}, "detections": set(DETECTION_TOPICS), "tf_static": {TF_STATIC}}

# Named so that a refusal says why; anything outside the allowlist is refused whether named here or not.
FORBIDDEN_KINDS = {"gt_auv": "ground truth", "gt_station": "ground truth", "gt_upstream": "ground truth",
                   "gt_tf_static": "ground truth", "dock": "ground-truth dock relation",
                   "alignment": "ground-truth frame alignment", "contact": "simulator contact",
                   "fused_dock": "the lab fuser's output (evaluation-side comparison input)",
                   "fused_tf": "the lab fuser's output (evaluation-side comparison input)"}
FORBIDDEN_TOPIC_PATTERNS = ((re.compile(r"^/piccard/ground_truth/"), "ground truth"),
                            (re.compile(r"/stonefish/odometry$"), "simulator odometry (ground truth)"),
                            (re.compile(r"/contact"), "simulator contact"),
                            (re.compile(r"^/race_station/dock_point/pose$"), "the lab fuser's output"))

_KIND = re.compile(rb'^\s*\{\s*"kind"\s*:\s*"([A-Za-z0-9_]+)"')


class TruthAccessError(PermissionError):
    """A request for a topic or record kind outside the estimator's sensor allowlist."""


def check_request(kinds=(), topics=()) -> None:
    """Raise TruthAccessError for any requested kind or topic outside the allowlist."""
    for kind in kinds:
        if kind not in SENSOR_KINDS:
            reason = FORBIDDEN_KINDS.get(kind, "not a sensor input")
            raise TruthAccessError(f"record kind {kind!r} is not an estimator input ({reason})")
    for topic in topics:
        if topic not in SENSOR_TOPICS:
            reason = next((why for pattern, why in FORBIDDEN_TOPIC_PATTERNS if pattern.search(topic)),
                          "not a sensor input")
            raise TruthAccessError(f"topic {topic!r} is not an estimator input ({reason})")


@dataclass(frozen=True)
class Odometry:
    stamp: float
    position: tuple
    orientation: tuple          # (x, y, z, w)
    covariance: tuple | None    # 36 values, row-major over (x, y, z, rot x, rot y, rot z), when recorded


@dataclass(frozen=True)
class Detection:
    tag: str                    # "tag36h11:146"
    position: tuple             # in the camera's optical frame, metres
    orientation: tuple | None   # the tag's orientation in the camera frame, when the source records it


@dataclass(frozen=True)
class DetectionArray:
    stamp: float                # the image stamp
    camera: str
    frame_id: str
    detections: tuple


@dataclass(frozen=True)
class SensorStream:
    source: str
    odometry: tuple
    detections: tuple
    tf_static: tuple            # ({"parent", "child", "xyz", "q"}, ...)
    odom_frame: str = "race_auv/odom"
    base_frame: str = "race_auv/base_link"
    notes: tuple = field(default_factory=tuple)


def _sensor_record(record: dict, kind: str):
    topic = record.get("topic")
    if topic not in KIND_TOPICS[kind]:
        check_request(topics=[topic or ""])
        raise TruthAccessError(f"record kind {kind!r} on topic {topic!r} is not an estimator input")
    if kind == "odometry":
        return Odometry(stamp=float(record["stamp"]), position=(record["x"], record["y"], record["z"]),
                        orientation=(record["qx"], record["qy"], record["qz"], record["qw"]),
                        covariance=tuple(record["covariance"]) if record.get("covariance") else None)
    if kind == "detections":
        positions = record.get("positions_m") or []
        ids = record.get("ids") or []
        orientations = record.get("orientations_q") or [None] * len(positions)
        detections = tuple(Detection(tag=str(tag), position=tuple(position),
                                     orientation=tuple(q) if q else None)
                           for tag, position, q in zip(ids, positions, orientations))
        return DetectionArray(stamp=float(record["stamp"]), camera=DETECTION_TOPICS[topic],
                              frame_id=record.get("frame_id") or f"race_auv/{DETECTION_TOPICS[topic]}",
                              detections=detections)
    return tuple(record.get("transforms") or ())


def read_telemetry(path, kinds=SENSOR_KINDS) -> SensorStream:
    """The sensor records of a collector telemetry.jsonl. Records of other kinds are skipped unparsed."""
    kinds = frozenset(kinds)
    check_request(kinds=kinds)
    odometry, detections, tf_static = [], [], []
    with Path(path).open("rb") as stream:
        for line in stream:
            match = _KIND.match(line)
            if match is None:
                continue
            kind = match.group(1).decode()
            if kind not in kinds:
                continue
            record = json.loads(line)
            if record.get("stamp") is None and kind != "tf_static":
                continue
            value = _sensor_record(record, kind)
            if kind == "odometry":
                odometry.append(value)
            elif kind == "detections":
                detections.append(value)
            else:
                tf_static.extend(value)
    notes = []
    if odometry and all(o.covariance is None for o in odometry):
        notes.append("odometry covariance not recorded; the configured odometry noise model applies")
    if detections and all(d.orientation is None for array in detections for d in array.detections):
        notes.append("tag orientations not recorded; position-only landmark factors")
    return SensorStream(source=str(path), odometry=tuple(sorted(odometry, key=lambda o: o.stamp)),
                        detections=tuple(sorted(detections, key=lambda d: d.stamp)), tf_static=tuple(tf_static),
                        notes=tuple(notes))


# ----------------------------------------------------------------------------- rosbag2 (UNTESTED on a real bag)
def _stamp(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def odometry_from_msg(msg) -> Odometry:
    """nav_msgs/msg/Odometry -> Odometry."""
    pose = msg.pose.pose
    covariance = tuple(float(v) for v in msg.pose.covariance)
    return Odometry(stamp=_stamp(msg.header), position=(pose.position.x, pose.position.y, pose.position.z),
                    orientation=(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                    covariance=covariance if any(covariance) else None)


def detections_from_msg(msg, camera: str) -> DetectionArray:
    """vision_msgs/msg/Detection3DArray -> DetectionArray: each detection's id and its first result's pose."""
    detections = []
    for detection in msg.detections:
        if not detection.results:
            continue
        pose = detection.results[0].pose.pose
        detections.append(Detection(tag=str(detection.id),
                                    position=(pose.position.x, pose.position.y, pose.position.z),
                                    orientation=(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                                                 pose.orientation.w)))
    return DetectionArray(stamp=_stamp(msg.header), camera=camera, frame_id=msg.header.frame_id,
                          detections=tuple(detections))


def tf_static_from_msg(msg) -> tuple:
    """tf2_msgs/msg/TFMessage -> static edges."""
    edges = []
    for transform in msg.transforms:
        t, r = transform.transform.translation, transform.transform.rotation
        edges.append({"parent": transform.header.frame_id, "child": transform.child_frame_id,
                      "xyz": [t.x, t.y, t.z], "q": [r.x, r.y, r.z, r.w]})
    return tuple(edges)


DETECTION_TYPE = "vision_msgs/msg/Detection3DArray"


def check_detection_type(topic: str, msgtype: str) -> None:
    """The position landmarks need each tag's solved 3D pose. The lab's LOCALIZATION_ARCHITECTURE.md plans to carry
    apriltag_msgs/msg/AprilTagDetectionArray (four corner pixels per tag, no pose) on the same detections3d topics;
    such a bag feeds corner projection factors, not this formulation, so it is refused here, not misread."""
    if msgtype != DETECTION_TYPE:
        raise ValueError(f"{topic} carries {msgtype}, not {DETECTION_TYPE}: corner detections need the corner "
                         "projection factors of the lab's LOCALIZATION_ARCHITECTURE.md, not built here")


def read_rosbag2(path, odometry_topic: str = EKF_ODOMETRY, detection_topics=None,
                 tf_static_topic: str = TF_STATIC) -> SensorStream:
    """UNTESTED until Yuewei's bag arrives. Reads the allowlisted topics of a rosbag2 directory (sqlite3 or mcap) with
    the rosbags library (`pip install rosbags`), using the message definitions the bag carries and the Jazzy
    typestore otherwise. Detections must be vision_msgs/msg/Detection3DArray (the lab's detections3d topics); a bag
    with only 2D apriltag_msgs detections cannot feed position-only landmark factors."""
    detection_topics = dict(detection_topics or DETECTION_TOPICS)
    check_request(topics=[odometry_topic, tf_static_topic, *detection_topics])
    from rosbags.highlevel import AnyReader  # optional dependency, imported only for a bag
    from rosbags.typesys import Stores, get_typestore

    odometry, detections, tf_static = [], [], []
    wanted = {odometry_topic, tf_static_topic, *detection_topics}
    with AnyReader([Path(path)], default_typestore=get_typestore(Stores.ROS2_JAZZY)) as reader:
        connections = [connection for connection in reader.connections if connection.topic in wanted]
        for connection in connections:
            if connection.topic in detection_topics:
                check_detection_type(connection.topic, connection.msgtype)
        for connection, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            if connection.topic == odometry_topic:
                odometry.append(odometry_from_msg(msg))
            elif connection.topic == tf_static_topic:
                tf_static.extend(tf_static_from_msg(msg))
            else:
                detections.append(detections_from_msg(msg, detection_topics[connection.topic]))
    return SensorStream(source=str(path), odometry=tuple(sorted(odometry, key=lambda o: o.stamp)),
                        detections=tuple(sorted(detections, key=lambda d: d.stamp)), tf_static=tuple(tf_static),
                        notes=("read from a rosbag2 by an untested reader",))
