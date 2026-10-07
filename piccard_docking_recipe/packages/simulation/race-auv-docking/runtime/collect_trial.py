#!/usr/bin/env python3
"""Bounded race-auv-docking/v1 trial. Never reads gain YAML files; reads back what the controller holds.

readiness -> audit (who consumes odometry, TF and ground truth) -> gain readback -> first setpoint ->
controller on -> helm direct_control -> pose mission -> trial.json and docking.json.
A planner mission (M3, #109) flies its fallback pose (the dive) the same way, then starts runtime/planner_fused_dock.py
(a separate node, audited under planner_inputs), keeps the fallback set point until the planner's first command,
stops publishing, and records the planner's stages until it reports complete.
Ground truth (both Stonefish odometries, the contact monitors, upstream's ground-truth node on
/piccard/ground_truth/*) is recorded for scoring only. ROS imports are lazy so the offline tests run without ROS.
This measures synthetic behaviour, not physical validity or sim-to-real transfer.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time

import yaml

import docking_metric
import graph_audit
import simulator_seed
from graph_audit import (CHILD_LINK, CONTACTS, CONTROLLER_NODE, EKF_ODOMETRY, FUSED_CHILD, FUSED_PARENT, GT_AUV,
                         GT_STATION, GT_TF, GT_TF_STATIC, GT_UPSTREAM_POSE, MEDIA_NODES, NS, PLANNER_NODE, WORLD_LINK,
                         consumption_audit)
from mission import is_planner_mission, mission_seconds, parameters_sha256, validate_mission
from prepare_candidate import BLACK_EDGE_FRACTION, FIELDS, MODES
import tag_pivot

# The consumption audit and its frames live in graph_audit (no YAML), so an offline check can recompute it; these
# stay readable here for collect_trial's existing readers.
HELM_NODE, MEDIA_TOPICS, PLANNER_TF_LOOKUPS = graph_audit.HELM_NODE, graph_audit.MEDIA_TOPICS, \
    graph_audit.PLANNER_TF_LOOKUPS
GAIN_ATTRS = {"p": "kp", "i": "ki", "d": "kd", "v": "kv", "pid_min": "pid_min", "pid_max": "pid_max"}
SETPOINT_TOPIC = f"{NS}/mvp_helm/bhv_direct_control/desired_setpoints"
DETECTIONS = {"/cam_front/apriltag_detection/detections3d": "cam_front",
              "/cam_down/apriltag_detection/detections3d": "cam_down"}
FUSED_POSE = "/race_station/dock_point/pose"
SCENARIO_FILES = ("world/race_auv_test.scn", "vehicles/race_auv.scn", "vehicles/race_station.scn",
                  "metadata/materials.scn", "metadata/looks.scn")
PLANNER_STATE = "/piccard/planner/state"
PLANNER_START = "/piccard_planner/start"
PLANNER_STALE_S = 2.0
PLANNER_ECHO_WINDOW_S = 2.0
SETPOINT_PERIOD_S = 1.0
SAMPLE_PERIOD_S = 0.5
STALE_S = 5.0
SETPOINT_ECHO_S = 10.0
SOURCE_PINS = {"race_auv": "da58963c048228856dfdd5917da21a14edfa1985",
               "race_auv_sim": "3601a30f49c7b8ddac2845c41c99af8af92c0e65",
               "race_auv_perception": "5944d5a6ebd44601bcc5579e91b82d5d1b3735cb",
               "race_station": "100bc5a7ac253172b527c036d637da5482eaf0f3",
               "world_of_stonefish": "d51d59e77211a436a0c3617c30a80c36337cdbbc",
               "mvp_control": "6cfea2d5c1e371bd371aa6587066d27c617c55ad",
               "mvp_msgs": "7aa616fab3bdd26dcd4ad178c4bfcd48c4231480",
               "mvp_mission": "84cce537aa08267eb65bcf2b9ea65a27d81bd6f0"}
# Written only by Dockerfile.controller-variant (piccard-inc/piccard-experiments#88, arm M3-C); absent in the recipe
# image.
CONTROLLER_VARIANT_RECORD = Path("/opt/piccard-race-auv/controller-variant.json")


def controller_variant(path: Path = CONTROLLER_VARIANT_RECORD) -> tuple[str, dict]:
    """(controller_variant, source-pin additions) for trial.json: "upstream" and none in the recipe image; in a
    controller-variant image its name, and the mvp_control patch it was built with; "unreadable" if the record is."""
    try:
        record = json.loads(path.read_text())
        name = record["controller_variant"]
        patch = {key: record[key] for key in ("patch", "patch_sha256", "file", "pristine_sha256", "patched_sha256")}
    except FileNotFoundError:
        return "upstream", {}
    except (OSError, ValueError, KeyError, TypeError) as error:
        return "unreadable", {"mvp_control_patch": {"error": f"{type(error).__name__}: {error}"}}
    return name, {"mvp_control_patch": {"controller_variant": name, **patch}}


def finite_json(value, path="", invalid=None, found=None):
    """Keep corrupt numeric observations visible, without nonstandard JSON NaN. `found` collects {path, value}."""
    if invalid is None:
        invalid = []
    if isinstance(value, float) and not math.isfinite(value):
        invalid.append(path)
        if found is not None:
            found.append({"path": path, "value": repr(value)})
        return None
    if isinstance(value, dict):
        return {str(k): finite_json(v, f"{path}.{k}", invalid, found) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v, f"{path}[{i}]", invalid, found) for i, v in enumerate(value)]
    return value


NONFINITE_CAP = 50
# Non-finite values the pinned stack produces by design, listed apart from unexpected ones so that a nonzero
# unexpected count in trial.json is always worth reading.
EXPECTED_NONFINITE = (
    {"event": "service_response", "service_prefix": "helm_change_", "path": ".payload.state.max_duration",
     "value": "inf", "reason": "mvp_msgs/HelmState.max_duration is inf when the helm config sets no max_duration "
                               "(no time limit)"},
)


def expected_nonfinite(event, field):
    return any(event.get("event") == rule["event"] and str(event.get("service") or "").startswith(rule["service_prefix"])
               and field["path"] == rule["path"] and field["value"] == rule["value"] for rule in EXPECTED_NONFINITE)


class NonfiniteLog:
    """Where non-finite values occurred: rows counted in full per class, the first `cap` of each recorded.
    A row is expected only when every non-finite field in it matches EXPECTED_NONFINITE."""

    def __init__(self, cap=NONFINITE_CAP):
        self.cap = cap
        self.classes = {"expected": {"rows": 0, "records": []}, "unexpected": {"rows": 0, "records": []}}

    def add(self, event, fields):
        label = "expected" if fields and all(expected_nonfinite(event, field) for field in fields) else "unexpected"
        entry = self.classes[label]
        entry["rows"] += 1
        if len(entry["records"]) < self.cap:
            record = {key: event.get(key) for key in ("kind", "topic", "event", "service", "t")}
            record["fields"] = fields
            entry["records"].append(record)
        return label

    def summary(self):
        return {"cap": self.cap, "expected_rules": list(EXPECTED_NONFINITE), **self.classes}


def gains_match(expected, actual, modes=tuple(MODES)):
    for mode in modes:
        for axis in MODES[mode]:
            for field in FIELDS:
                try:
                    a, b = expected[mode][axis][field], actual[mode][axis][field]
                    if isinstance(a, bool) or isinstance(b, bool):
                        return False
                    a, b = float(a), float(b)
                    # Upstream YAML::as<float>() precedes float64 ROS readback.
                    if not (math.isfinite(a) and math.isfinite(b)
                            and math.isclose(a, b, rel_tol=2e-7, abs_tol=1e-8)):
                        return False
                except (KeyError, TypeError, ValueError):
                    return False
    return True


def sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def scenario_provenance(wrapper: Path, wos_share: Path, sim_params: Path) -> dict:
    """The scenario the simulator loads: the Piccard wrapper, the resolved world_of_stonefish share and the
    upstream files the wrapper includes, each with its sha256."""
    return {"wrapper": {"path": str(wrapper), "sha256": sha256_file(wrapper)},
            "world_of_stonefish_share": str(wos_share),
            "upstream": [{"path": str(wos_share / name), "sha256": sha256_file(wos_share / name)} for name in SCENARIO_FILES],
            "sim_params": {"path": str(sim_params), "sha256": sha256_file(sim_params)}}


def pivot_record(mission_pivot: list, share_directory) -> dict:
    """The mission's tag_pivot_m against the pivot derived from the installed files the fuser loads (tag_pivot.py);
    a pivot that cannot be derived is recorded with its error and is not ok."""
    try:
        return tag_pivot.check(mission_pivot, tag_pivot.derive_installed(share_directory, yaml))
    except Exception as error:  # noqa: BLE001 - recorded, and the trial fails closed
        return {"mission_m": list(mission_pivot), "derived": None, "error": f"{type(error).__name__}: {error}",
                "tolerance_m": tag_pivot.PIVOT_TOLERANCE_M, "ok": False}


def tags_record(share_directory, tag_size: str) -> dict:
    """trial.json tags: every camera's configured tags, their sizes in both conventions and their positions in the
    station dock frame, from the installed files the fuser loads; an error is recorded, not raised."""
    try:
        return tag_pivot.tag_record(share_directory, yaml, tag_size, BLACK_EDGE_FRACTION)
    except Exception as error:  # noqa: BLE001 - informational record
        return {"error": f"{type(error).__name__}: {error}"}


def track_steps(steps: list, previous, t: float, record: dict):
    """The planner's discrete set-point steps as [t, x, y, z, yaw] of its held set point, one per change of its
    setpoint_updates counts (the first record counts when it already has updates). Returns the counts to compare the
    next record with."""
    updates = record.get("setpoint_updates")
    if updates is None:
        return previous
    if updates != previous and (previous is not None or any(updates.values())):
        held = record.get("held") or record.get("command") or {}
        steps.append([round(t, 3)] + [held.get(key) for key in ("x_m", "y_m", "z_m", "yaw_rad")])
    return updates


def track_rehold(current, t: float, record: dict):
    """The planner's final-stage heading re-hold (v1.2), once: its record the first time a state record carries one,
    with the collector time it arrived (received_t); the planner's own t is on the planner's clock."""
    if current is not None or not record.get("heading_rehold"):
        return current
    return {**record["heading_rehold"], "received_t": round(t, 3)}


def track_final_along(current, t: float, record: dict):
    """The planner's final-stage along phases (protocol v1.5): the latest state record's phase and side, and each
    arrival and re-approach with the collector time it first arrived (received_t); the planner's own t is on its
    clock."""
    along = record.get("final_along")
    if not along:
        return current
    current = current or {"phase": None, "side": None, "arrivals": [], "reapproaches": []}
    for key in ("arrivals", "reapproaches"):
        current[key] += [{**item, "received_t": round(t, 3)} for item in along[key][len(current[key]):]]
    current["phase"], current["side"] = along["phase"], along["side"]
    return current


def track_stale(intervals: list, t: float, record: dict) -> None:
    """The planner's stale intervals in trial time, [{from_t, to_t, reason}]: a run of stale state records with one
    stale_reason (age or frozen); to_t is the first record after it, null while it lasts."""
    reason = record.get("stale_reason") if record.get("state") == "stale" else None
    last = intervals[-1] if intervals and intervals[-1]["to_t"] is None else None
    if last is not None and last["reason"] != reason:
        last["to_t"], last = t, None
    if reason is not None and last is None:
        intervals.append({"from_t": t, "to_t": None, "reason": reason})


def end_audit_verdict(status: str, stop_reason: str, audit: dict) -> tuple[str, str]:
    """A trial is evidence only if the audit still holds when it ends: a completed or censored trial whose
    end-of-trial audit found a problem, or could not run, fails."""
    if status not in ("completed", "budget_censored"):
        return status, stop_reason
    if "error" in audit:
        return "failed", "consumption_audit_at_end_unavailable"
    if audit.get("problems"):
        return "failed", "consumption_audit_at_end:" + ",".join(audit["problems"])
    return status, stop_reason


def ros_node_class():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from rcl_interfaces.srv import GetParameters
    from rosgraph_msgs.msg import Clock
    from rosidl_runtime_py.convert import message_to_ordereddict
    from std_msgs.msg import Bool, Float64, Float64MultiArray, String
    from std_srvs.srv import SetBool, Trigger
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformListener
    from vision_msgs.msg import Detection3DArray
    from visualization_msgs.msg import Marker
    from mvp_msgs.msg import ControlProcess
    from mvp_msgs.srv import ChangeState, GetControlMode, GetControlModes

    latched = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

    def xyz(vector):
        return [vector.x, vector.y, vector.z]

    def quaternion(q):
        return [q.x, q.y, q.z, q.w]

    class Trial(Node):
        def __init__(self, args):
            super().__init__("piccard_trial_collector")
            self.args = args
            self.root = Path(args.output)
            self.started_ns = time.monotonic_ns()
            self.stream = (self.root / "telemetry.jsonl").open("x", encoding="utf-8")
            self.latest_sim = None
            self.counts, self.last_t = {}, {}
            self.latest = {}
            self.helm_state = None
            self.direct_control_seen = False
            self.stop_requested = None
            self.invalid_count = 0
            self.nonfinite = NonfiniteLog()
            self.summary_rows = []
            self.static_edges = {}  # vehicle and ground-truth static TF, for frame validation
            self.tf_edges = {}  # vehicle /tf and /tf_static edges, for the audit
            self.upstream_checks = []
            self.thruster_topics = set()
            self.subscriptions_owned = []
            self.commanded = None
            variant, variant_pins = controller_variant()
            self.metadata = {
                "schema": "piccard.race-auv.trial/v1", "status": "failed",
                "candidate_id": os.environ.get("PICCARD_TRIAL_ID"),
                "epistemic_class": "synthetic_simulation_development",
                "stop_reason": "not_started", "mission_start_t": None, "mission_end_t": None,
                "gains_verified": False, "comparison_context": {}, "tags": None,
                "horizon_seconds": args.horizon_seconds, "wall_timeout_seconds": args.wall_timeout_seconds,
                "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
                "controller_variant": variant, "source_pins": {**SOURCE_PINS, **variant_pins},
                "ground_truth_scope": "scoring only: /race_auv/stonefish/odometry, /race_station/stonefish/odometry, "
                                      "/piccard/contact/*, /piccard/ground_truth/*",
            }
            context = json.loads(Path(args.context_json).read_text()) if args.context_json else {}
            if not isinstance(context, dict):
                raise ValueError("comparison context must be an object")
            self.metadata["comparison_context"] = context
            raw = Path(args.mission_profile_json).read_bytes()
            self.mission = validate_mission(json.loads(raw))
            self.planner_mission = is_planner_mission(self.mission)
            self.planner_sha256 = parameters_sha256(self.mission["planner"]) if self.planner_mission else None
            self.planner_latest = None  # the last planner state record
            self.planner_rows = {"commands": 0, "stale": 0}
            self.planner_stale = []  # track_stale's intervals
            self.tag_pivot = None  # pivot_record, before anything moves
            self.planner_steps, self.planner_updates = [], None  # track_steps
            self.planner_rehold = None  # track_rehold
            self.planner_final_along = None  # track_final_along
            self.planner_stages = []  # [{"label", "standoff_m", "start_t"}] in the order the planner entered them
            self.planner_commands = []  # (t, command) of the last PLANNER_ECHO_WINDOW_S, for the echo check
            self.planner_handover_t = None
            self.metadata.update(mission=self.mission, mission_sha256=hashlib.sha256(raw).hexdigest(),
                                 mission_seconds=mission_seconds(self.mission),
                                 simulator_seed=simulator_seed.read_record(self.root),  # read again at the end
                                 apriltag_tag_size=self.mission["apriltag_tag_size"])
            self.expected = json.loads(Path(args.expected_gains_json).read_text())
            if not gains_match(self.expected, self.expected):
                raise ValueError("expected gains must contain every finite flight and docking gain field")
            self.metadata["expected_gains"] = self.expected
            for topic, cls, kind, qos in [
                ("/clock", Clock, "clock", qos_profile_sensor_data),
                (f"{NS}/controller/process/value", ControlProcess, "value", qos_profile_sensor_data),
                (f"{NS}/controller/process/set_point", ControlProcess, "set_point", qos_profile_sensor_data),
                (f"{NS}/controller/process/error", ControlProcess, "controller_error", qos_profile_sensor_data),
                (f"{NS}/controller/state", Bool, "controller_state", qos_profile_sensor_data),
                (f"{NS}/mvp_helm/current_helm_state", String, "helm_state", qos_profile_sensor_data),
                (EKF_ODOMETRY, Odometry, "odometry", qos_profile_sensor_data),
                (GT_AUV, Odometry, "gt_auv", reliable),
                (GT_STATION, Odometry, "gt_station", reliable),
                (GT_UPSTREAM_POSE, PoseStamped, "gt_upstream", reliable),
                (FUSED_POSE, PoseStamped, "fused_dock", reliable),
                ("/tf", TFMessage, "tf", reliable),
                ("/tf_static", TFMessage, "tf_static", latched),
                (GT_TF_STATIC, TFMessage, "gt_tf_static", latched),
                *[(topic, Marker, "contact", reliable) for topic in CONTACTS],
                *[(topic, Detection3DArray, "detections", reliable) for topic in DETECTIONS],
                *([(PLANNER_STATE, String, "planner", reliable)] if self.planner_mission else []),
            ]:
                self.subscribe(topic, cls, kind, qos)
            for term in ("p", "i", "d", "v"):
                self.subscribe(f"{NS}/controller/process/{term}_value", Float64MultiArray, f"pid_{term}",
                               qos_profile_sensor_data)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.control = self.create_client(SetBool, f"{NS}/controller/set")
            self.change = self.create_client(ChangeState, f"{NS}/mvp_helm/change_state")
            self.params = self.create_client(GetParameters, f"{CONTROLLER_NODE}/get_parameters")
            self.modes = self.create_client(GetControlModes, f"{NS}/controller/get_modes")
            self.active_mode = self.create_client(GetControlMode, f"{NS}/controller/active_mode")
            self.setpoints = self.create_publisher(ControlProcess, SETPOINT_TOPIC, 10)
            self.planner_start = self.create_client(Trigger, PLANNER_START) if self.planner_mission else None
            self.sample_timer = self.create_timer(SAMPLE_PERIOD_S, self.sample_frames)
            self.discovery_timer = self.create_timer(2.0, self.discover_thrusters)

        # ------------------------------------------------------------------ recording
        def t(self):
            return (time.monotonic_ns() - self.started_ns) / 1e9

        def emit(self, kind, topic=None, **fields):
            event = {"kind": kind, "t": self.t(), "monotonic_ns": time.monotonic_ns(),
                     "wall_time_ns": time.time_ns(), "topic": topic, "sim": self.latest_sim, **fields}
            bad, found = [], []
            event = finite_json(event, invalid=bad, found=found)
            if bad:
                event["invalid_fields"] = bad
                self.invalid_count += 1
                self.nonfinite.add(event, found)
            self.stream.write(json.dumps(event, separators=(",", ":"), allow_nan=False) + "\n")
            self.stream.flush()
            if kind in ("dock", "event", "contact", "detections", "fused_dock", "alignment"):
                self.summary_rows.append(event)
            return event

        def subscribe(self, topic, cls, kind, qos):
            callback = lambda msg, topic=topic, kind=kind: self.receive(topic, kind, msg)
            self.subscriptions_owned.append(self.create_subscription(cls, topic, callback, qos))

        def count(self, kind):
            self.counts[kind] = self.counts.get(kind, 0) + 1
            self.last_t[kind] = self.t()

        def receive(self, topic, kind, msg):
            if kind == "tf":  # counted, not recorded row by row
                for transform in msg.transforms:
                    edge = f"{transform.header.frame_id}->{transform.child_frame_id}"
                    self.tf_edges[edge] = self.tf_edges.get(edge, 0) + 1
                return self.count(kind)
            fields = {"stamp": None}
            if hasattr(msg, "header"):
                fields["stamp"] = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                fields["frame_id"] = msg.header.frame_id
            if kind in ("value", "set_point", "controller_error"):
                fields.update(x=msg.position.x, y=msg.position.y, z=msg.position.z, roll=msg.orientation.x,
                              pitch=msg.orientation.y, yaw=msg.orientation.z, u=msg.velocity.x, v=msg.velocity.y,
                              child_frame_id=msg.child_frame_id, control_mode=msg.control_mode)
            elif kind in ("odometry", "gt_auv", "gt_station"):
                pose, twist = msg.pose.pose, msg.twist.twist
                fields.update(child_frame_id=msg.child_frame_id, x=pose.position.x, y=pose.position.y,
                              z=pose.position.z, qx=pose.orientation.x, qy=pose.orientation.y,
                              qz=pose.orientation.z, qw=pose.orientation.w, linear_mps=xyz(twist.linear),
                              angular_radps=xyz(twist.angular))
            elif kind in ("gt_upstream", "fused_dock"):
                fields.update(position_m=xyz(msg.pose.position), orientation_q=quaternion(msg.pose.orientation))
            elif kind in ("tf_static", "gt_tf_static"):
                transforms = []
                for transform in msg.transforms:
                    edge = {"xyz": xyz(transform.transform.translation), "q": quaternion(transform.transform.rotation)}
                    name = f"{transform.header.frame_id}->{transform.child_frame_id}"
                    self.static_edges[name] = edge
                    if kind == "tf_static":
                        self.tf_edges[name] = self.tf_edges.get(name, 0) + 1
                    transforms.append({"parent": transform.header.frame_id, "child": transform.child_frame_id, **edge})
                fields["transforms"] = transforms
            elif kind == "contact":
                start, tip = msg.points[0], msg.points[1]
                force = [tip.x - start.x, tip.y - start.y, tip.z - start.z]
                fields.update(contact=CONTACTS[topic], location_m=xyz(start), normal_force_vector_n=force,
                              normal_force_n=math.sqrt(sum(value * value for value in force)))
            elif kind == "detections":
                fields.update(camera=DETECTIONS[topic], count=len(msg.detections),
                              ids=[detection.id for detection in msg.detections],
                              positions_m=[xyz(detection.results[0].pose.pose.position)
                                           for detection in msg.detections if detection.results])
            elif kind == "clock":
                self.latest_sim = msg.clock.sec + msg.clock.nanosec / 1e9
                return self.count(kind)
            elif kind == "helm_state":
                self.helm_state = fields["state"] = msg.data
                self.direct_control_seen |= msg.data == "direct_control"
            elif kind == "controller_state":
                fields["enabled"] = msg.data
            elif kind == "planner":
                try:
                    record = json.loads(msg.data)
                except ValueError:
                    record = {"state": "unparsed"}
                fields["planner"] = record
                self.planner_latest = {**record, "received_t": self.t()}
                if record.get("command") is not None:
                    self.planner_rows["commands"] += 1
                    self.planner_commands = [(t, c) for t, c in self.planner_commands
                                             if self.t() - t <= PLANNER_ECHO_WINDOW_S] + [(self.t(), record["command"])]
                self.planner_rows["stale"] += record.get("state") == "stale"
                track_stale(self.planner_stale, self.t(), record)
                self.planner_updates = track_steps(self.planner_steps, self.planner_updates, self.t(), record)
                self.planner_rehold = track_rehold(self.planner_rehold, self.t(), record)
                self.planner_final_along = track_final_along(self.planner_final_along, self.t(), record)
            else:  # thruster commands and pid terms
                fields["data"] = msg.data if kind == "thruster" else list(msg.data)
            event = self.emit(kind, topic, **fields)
            self.count(kind)
            self.latest[kind] = event
            if kind == "gt_auv" and "gt_station" in self.latest:
                self.record_dock(event)
            if kind == "gt_upstream" and "gt_auv" in self.latest and "gt_station" in self.latest:
                ours = docking_metric.base_to_base(self.latest["gt_auv"], self.latest["gt_station"])
                difference = math.dist(ours, event["position_m"])
                self.upstream_checks.append({"t": event["t"], "difference_m": difference})
            if kind in ("value", "set_point", "gt_auv") and event.get("invalid_fields"):
                self.stop_requested = "nonfinite_critical_telemetry"

        def record_dock(self, auv):
            state = docking_metric.docking_state(auv, self.latest["gt_station"])
            self.emit("dock", GT_AUV, **state, auv_linear_velocity_mps=auv["linear_mps"],
                      auv_angular_velocity_radps=auv["angular_radps"],
                      velocity_frame=auv.get("child_frame_id"), label=self.commanded and self.commanded["label"])

        def lookup(self, parent, child):
            try:
                transform = self.tf_buffer.lookup_transform(parent, child, Time())
            except Exception:
                return None
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
            return {"stamp": stamp, "xyz": xyz(transform.transform.translation),
                    "q": quaternion(transform.transform.rotation)}

        def sample_frames(self):
            """Controller-frame cg_link against ground truth, and the fused dock-point TF, at 2 Hz."""
            controller = self.lookup(WORLD_LINK, CHILD_LINK)
            if controller and "gt_auv" in self.latest:
                rpy = docking_metric.matrix_rpy(docking_metric.quat_matrix(*controller["q"]))
                self.emit("alignment", None, controller_cg={"position_m": controller["xyz"], "rpy_rad": rpy,
                                                            "stamp": controller["stamp"]},
                          ground_truth_cg=docking_metric.ground_truth_cg(self.latest["gt_auv"]))
            fused = self.lookup(FUSED_PARENT, FUSED_CHILD)
            if fused and fused["stamp"] != self.latest.get("fused_tf_stamp"):
                self.latest["fused_tf_stamp"] = fused["stamp"]
                self.emit("fused_tf", None, parent=FUSED_PARENT, child=FUSED_CHILD, **fused)
                self.count("fused_tf")

        def discover_thrusters(self):
            for topic, types in self.get_topic_names_and_types():
                if (topic.startswith(f"{NS}/control/thruster/") and "std_msgs/msg/Float64" in types
                        and topic not in self.thruster_topics and len(self.thruster_topics) < 16):
                    self.thruster_topics.add(topic)
                    self.subscribe(topic, Float64, "thruster", qos_profile_sensor_data)
                    self.emit("event", event="thruster_subscribed", subscribed_topic=topic)

        # ------------------------------------------------------------------ graph audit
        def audit(self):
            subscriptions = {}
            for name, namespace in self.get_node_names_and_namespaces():
                node = f"{namespace.rstrip('/')}/{name}"
                try:
                    topics = self.get_subscriber_names_and_types_by_node(name, namespace)
                except Exception as exc:
                    topics = []
                    self.emit("event", event="graph_query_failed", node=node, error=str(exc))
                subscriptions[node] = {topic: list(types) for topic, types in topics}
            publishers = {topic: sorted(f"{info.node_namespace.rstrip('/')}/{info.node_name}"
                                        for info in self.get_publishers_info_by_topic(topic))
                          for topic in ("/tf", "/tf_static", GT_TF, GT_TF_STATIC, SETPOINT_TOPIC)}
            setpoint_publishers = publishers.pop(SETPOINT_TOPIC)
            result = consumption_audit(subscriptions, publishers, dict(sorted(self.tf_edges.items())),
                                       planner_expected=self.planner_mission,
                                       planner_lookups=(self.planner_latest or {}).get("tf_lookups"),
                                       setpoint_publishers=setpoint_publishers if self.planner_mission else None)
            result["subscriptions"] = {node: topics for node, topics in subscriptions.items() if node not in MEDIA_NODES}
            return result

        # ------------------------------------------------------------------ services
        def check(self):
            if self.stop_requested:
                raise RuntimeError(self.stop_requested)
            if self.t() >= self.args.wall_timeout_seconds:
                raise TimeoutError("total_wall_timeout")
            if self.args.launch_pid:
                try:
                    os.kill(self.args.launch_pid, 0)
                except ProcessLookupError:
                    raise RuntimeError("launch_process_exited") from None

        def spin(self, duration=0.05, checking=True):
            if checking:
                self.check()
            rclpy.spin_once(self, timeout_sec=duration)

        def call(self, client, req, label, seconds=10.0, cleanup=False, record_payload=True):
            deadline = time.monotonic() + seconds
            while not client.service_is_ready():
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"service_unavailable:{label}")
                self.spin(checking=not cleanup)
            future = client.call_async(req)
            while not future.done():
                if time.monotonic() >= deadline:
                    future.cancel()
                    raise TimeoutError(f"service_timeout:{label}")
                self.spin(checking=not cleanup)
            result = future.result()
            if result is None:
                raise RuntimeError(f"service_empty_response:{label}")
            fields = {"event": "service_response", "service": label}
            if record_payload:
                fields["payload"] = message_to_ordereddict(result)
            self.emit("event", **fields)
            return result

        def mode_gains(self, mode):
            return {axis: {field: getattr(getattr(mode, "pid_" + axis), attr) for field, attr in GAIN_ATTRS.items()}
                    for axis in MODES[mode.name]}

        def read_gains(self):
            """Cached modes and parameters before the mission: both modes must equal the request."""
            response = self.call(self.modes, GetControlModes.Request(), "get_modes", record_payload=False)
            cached = {mode.name: self.mode_gains(mode) for mode in response.modes if mode.name in MODES}
            self.metadata["cached_gains"] = cached
            names = [f"control_modes/{m}/{a}/{f}" for m in MODES for a in MODES[m] for f in FIELDS]
            names += ["controller_frequency", "controller_no_setpoint_timeout", "odometry_source", "tf_prefix",
                      "world_link_initial", "child_link_initial", "generator_type"]
            req = GetParameters.Request()
            req.names = names
            values = self.call(self.params, req, "controller_parameters", record_payload=False).values
            if len(values) != len(names):  # rclcpp answers nothing when any name is undeclared
                raise RuntimeError("controller_parameter_readback_incomplete")
            readback = {}
            for name, value in zip(names, values):
                readback[name] = {1: value.bool_value, 2: value.integer_value, 3: value.double_value,
                                  4: value.string_value}.get(value.type)
            self.metadata["parameter_readback"] = readback
            observed = {m: {a: {f: readback.get(f"control_modes/{m}/{a}/{f}") for f in FIELDS} for a in MODES[m]}
                        for m in MODES}
            self.emit("event", event="cached_gain_readback", gains=cached)
            if not gains_match(self.expected, observed):
                raise RuntimeError("gain_parameter_mismatch")
            if set(cached) != set(MODES) or not gains_match(self.expected, cached):
                raise RuntimeError("gain_mode_mismatch")
            if readback.get("odometry_source") != EKF_ODOMETRY:
                raise RuntimeError("controller_odometry_source_mismatch")

        def read_active_gains(self):
            mode = self.call(self.active_mode, GetControlMode.Request(), "active_mode", record_payload=False).mode
            if mode.name != "docking":
                raise RuntimeError(f"unexpected_active_mode:{mode.name}")
            active = {"docking": self.mode_gains(mode)}
            self.metadata["active_gains"] = active
            self.emit("event", event="active_gain_readback", gains=active)
            if not gains_match(self.expected, active, modes=("docking",)):
                raise RuntimeError("docking_gain_mismatch")
            self.metadata["gains_verified"] = True

        def change_state(self, state, cleanup=False):
            req = ChangeState.Request()
            req.state, req.caller = state, "piccard_bounded_trial"
            response = self.call(self.change, req, f"helm_change_{state}", seconds=2.0 if cleanup else 10.0,
                                 cleanup=cleanup)
            if not response.status or response.state.name != state:
                raise RuntimeError(f"helm_state_rejected:{state}")
            return response

        def enable(self, enabled, cleanup=False):
            req = SetBool.Request()
            req.data = enabled
            response = self.call(self.control, req, f"controller_enable_{enabled}",
                                 seconds=2.0 if cleanup else 10.0, cleanup=cleanup)
            if not response.success:
                raise RuntimeError("controller_enable_rejected")

        # ------------------------------------------------------------------ mission
        def publish_setpoint(self, pose):
            msg = ControlProcess()
            msg.header.frame_id, msg.child_frame_id, msg.control_mode = WORLD_LINK, CHILD_LINK, "docking"
            msg.position.x, msg.position.y, msg.position.z = (float(pose[k]) for k in ("x_m", "y_m", "z_m"))
            msg.orientation.x, msg.orientation.y, msg.orientation.z = (
                float(pose[k]) for k in ("roll_rad", "pitch_rad", "yaw_rad"))
            self.setpoints.publish(msg)
            self.commanded = pose
            self.last_setpoint_publish = self.t()
            self.emit("setpoint_command", SETPOINT_TOPIC, label=pose["label"], frame_id=WORLD_LINK,
                      child_frame_id=CHILD_LINK, **{k: pose[k] for k in pose if k not in ("label", "dwell_s")})

        def setpoint_echoed(self, pose):
            echo = self.latest.get("set_point")
            if not echo or echo["t"] < self.pose_started or echo.get("control_mode") != "docking":
                return False
            linear = all(abs(echo[a] - pose[k]) <= 1e-3 for a, k in (("x", "x_m"), ("y", "y_m"), ("z", "z_m")))
            angular = all(abs(docking_metric.wrap(echo[a] - pose[k])) <= 1e-3
                          for a, k in (("roll", "roll_rad"), ("pitch", "pitch_rad"), ("yaw", "yaw_rad")))
            return linear and angular

        def ready(self):
            return (self.counts.get("value", 0) >= 5 and self.helm_state == "start"
                    and all(self.counts.get(kind, 0) > 0 for kind in ("gt_auv", "gt_station", "odometry"))
                    and all(client.service_is_ready() for client in
                            (self.control, self.change, self.params, self.modes, self.active_mode))
                    and self.setpoints.get_subscription_count() > 0
                    and self.tf_buffer.can_transform(WORLD_LINK, CHILD_LINK, Time())
                    and (not self.planner_mission or (self.planner_start.service_is_ready()
                                                      and self.planner_latest is not None)))

        def run(self):
            self.emit("event", event="collector_started")
            self.metadata["scenario"] = self.args.scenario_provenance
            self.emit("event", event="scenario_provenance", **self.args.scenario_provenance)
            self.metadata["tags"] = tags_record(self.args.package_share, self.mission["apriltag_tag_size"])
            if self.planner_mission:  # the mission's pivot must be the fuser's station construction, not a tuned offset
                self.tag_pivot = pivot_record(self.mission["planner"]["tag_pivot_m"], self.args.package_share)
                if not self.tag_pivot["ok"]:
                    raise RuntimeError("tag_pivot_underivable" if self.tag_pivot["derived"] is None
                                       else "tag_pivot_mismatch")
            ready_deadline = min(self.args.ready_timeout_seconds, self.args.wall_timeout_seconds)
            while not self.ready():
                if self.t() >= ready_deadline:
                    raise TimeoutError("readiness_timeout")
                self.spin()
            self.discover_thrusters()
            if self.planner_mission and self.planner_latest.get("parameters_sha256") != self.planner_sha256:
                raise RuntimeError("planner_parameters_mismatch")
            audit = self.audit()
            self.metadata["consumption_audit_before_mission"] = audit
            if audit["problems"]:
                raise RuntimeError("consumption_audit:" + ",".join(audit["problems"]))
            self.read_gains()
            poses = [self.mission["fallback_pose"]] if self.planner_mission else self.mission["poses"]
            first = poses[0]
            self.pose_started = self.t()
            self.publish_setpoint(first)  # held by bhv_direct_control before the state is entered
            self.enable(True)
            response = self.change_state("direct_control")
            self.metadata["helm_direct_control"] = {"mode": response.state.mode,
                                                    "transitions": list(response.state.transitions)}
            if response.state.mode != "docking":
                raise RuntimeError(f"direct_control_mode_mismatch:{response.state.mode}")
            deadline = self.t() + 10
            while not (self.latest.get("value", {}).get("control_mode") == "docking" and self.direct_control_seen):
                if self.t() > deadline:
                    raise TimeoutError("docking_mode_activation_timeout")
                self.spin()
            self.read_active_gains()
            self.metadata["mission_start_t"] = self.t()
            self.metadata["mission_start_sim"] = self.latest_sim
            self.emit("event", event="mission_start")
            for pose in poses:
                if self.fly(pose) == "horizon":
                    self.metadata.update(status="budget_censored", stop_reason="fixed_wall_horizon")
                    return
            if self.planner_mission and self.plan() == "horizon":
                self.metadata.update(status="budget_censored", stop_reason="fixed_wall_horizon")
                return
            self.metadata.update(status="completed", stop_reason="mission_complete")

        def fly(self, pose):
            self.pose_started = start = self.t()
            self.publish_setpoint(pose)
            self.emit("event", event="pose_start", label=pose["label"])
            echoed = None
            while self.t() - start < pose["dwell_s"]:
                self.spin()
                now = self.t()
                if now - self.last_setpoint_publish >= SETPOINT_PERIOD_S:
                    self.publish_setpoint(pose)
                if echoed is None and self.setpoint_echoed(pose):
                    echoed = now - start
                    self.emit("event", event="setpoint_echoed", label=pose["label"], after_s=echoed)
                if echoed is None and now - start > SETPOINT_ECHO_S:
                    raise RuntimeError(f"setpoint_not_applied:{pose['label']}")
                self.watch(now)
                if now - self.metadata["mission_start_t"] >= self.args.horizon_seconds:
                    self.emit("event", event="pose_end", label=pose["label"], censored=True)
                    return "horizon"
            self.emit("event", event="pose_end", label=pose["label"], censored=False)
            return "dwelled"

        def watch(self, now):
            """The checks every mission step makes: helm in direct_control, the controller and ground truth fresh."""
            if self.helm_state == "kill":
                raise RuntimeError("helm_entered_kill")
            if self.helm_state != "direct_control":
                raise RuntimeError(f"helm_left_direct_control:{self.helm_state}")
            for kind in ("value", "set_point", "gt_auv"):
                if now - self.last_t.get(kind, now) > STALE_S:
                    raise RuntimeError(f"{kind}_stale")

        def planner_echoed(self):
            echo = self.latest.get("set_point")
            if not echo or echo["t"] < self.planner_handover_t or echo.get("control_mode") != "docking":
                return False
            return any(all(abs(echo[a] - command[k]) <= 1e-3 for a, k in (("x", "x_m"), ("y", "y_m"), ("z", "z_m")))
                       and abs(docking_metric.wrap(echo["yaw"] - command["yaw_rad"])) <= 1e-3
                       for _, command in self.planner_commands)

        def plan(self):
            """Start the planner; hold the fallback set point until its first command, then stop publishing and
            follow its stages (as pose_start/pose_end events) until it reports complete or the horizon ends."""
            response = self.call(self.planner_start, Trigger.Request(), "planner_start")
            if not response.success or response.message != self.planner_sha256:
                raise RuntimeError("planner_start_rejected")
            start, stage, echoed = self.t(), None, None
            self.emit("event", event="planner_start")
            while True:
                self.spin()
                now = self.t()
                state = self.planner_latest
                if state.get("parameters_sha256") != self.planner_sha256:
                    raise RuntimeError("planner_parameters_mismatch")
                if now - state["received_t"] > PLANNER_STALE_S:
                    raise RuntimeError("planner_state_stale")
                if self.planner_handover_t is None:
                    if state.get("command") is not None:
                        self.planner_handover_t = now
                        self.emit("event", event="planner_handover")
                    elif now - start > SETPOINT_ECHO_S:
                        raise RuntimeError("planner_never_commanded")
                    elif now - self.last_setpoint_publish >= SETPOINT_PERIOD_S:
                        self.publish_setpoint(self.mission["fallback_pose"])
                elif echoed is None:
                    if self.planner_echoed():
                        echoed = now - self.planner_handover_t
                        self.emit("event", event="planner_setpoint_echoed", after_s=echoed)
                    elif now - self.planner_handover_t > SETPOINT_ECHO_S:
                        raise RuntimeError("planner_setpoint_not_applied")
                if state.get("state") in ("tracking", "stale", "complete") and state.get("stage") != stage:
                    if stage is not None:
                        self.emit("event", event="pose_end", label=self.planner_stages[-1]["label"], censored=False)
                    stage = state["stage"]
                    label = f"planner_standoff_{state['standoff_m']:g}m".replace(".", "p")
                    self.planner_stages.append({"label": label, "standoff_m": state["standoff_m"], "start_t": now})
                    self.commanded = {"label": label}
                    self.emit("event", event="pose_start", label=label)
                if state.get("state") == "complete":
                    self.emit("event", event="pose_end", label=self.planner_stages[-1]["label"], censored=False)
                    return "complete"
                self.watch(now)
                if now - self.metadata["mission_start_t"] >= self.args.horizon_seconds:
                    if stage is not None:
                        self.emit("event", event="pose_end", label=self.planner_stages[-1]["label"], censored=True)
                    return "horizon"

        # ------------------------------------------------------------------ finish
        def finish(self):
            stop = self.t()
            self.metadata["actual_stop_t"] = stop
            start = self.metadata["mission_start_t"]
            self.metadata["mission_end_t"] = stop if start is not None else None
            self.metadata["mission_end_sim"] = self.latest_sim
            self.emit("event", event="mission_end", status=self.metadata["status"], stop_reason=self.metadata["stop_reason"])
            try:
                self.metadata["consumption_audit_at_end"] = self.audit()
            except Exception as exc:
                self.metadata["consumption_audit_at_end"] = {"error": str(exc)}
            status, reason = end_audit_verdict(self.metadata["status"], self.metadata["stop_reason"],
                                               self.metadata["consumption_audit_at_end"])
            if (status, reason) != (self.metadata["status"], self.metadata["stop_reason"]):
                self.metadata.update(status=status, stop_reason=reason)
                self.emit("event", event="failure", error=reason, error_type="ConsumptionAuditAtEnd")
            self.metadata["cleanup_service_results"] = []
            for label, action in [("helm_start", lambda: self.change_state("start", cleanup=True)),
                                  ("controller_disable", lambda: self.enable(False, cleanup=True))]:
                try:
                    action()
                    self.metadata["cleanup_service_results"].append({"action": label, "success": True})
                except Exception as exc:
                    self.metadata["cleanup_service_results"].append({"action": label, "success": False, "error": str(exc)})
            summary_mission = self.mission if not self.planner_mission else {
                "apriltag_tag_size": self.mission["apriltag_tag_size"],
                "poses": [self.mission["fallback_pose"]] + [{"label": s["label"], "standoff_m": s["standoff_m"]}
                                                           for s in self.planner_stages]}
            docking = docking_metric.summarize(self.summary_rows, summary_mission)
            self.metadata["planner"] = None if not self.planner_mission else {
                "node": PLANNER_NODE, "parameters": self.mission["planner"], "parameters_sha256": self.planner_sha256,
                "reported_parameters_sha256": (self.planner_latest or {}).get("parameters_sha256"),
                "stages": self.planner_stages, "complete": (self.planner_latest or {}).get("state") == "complete",
                "handover_t": self.planner_handover_t, "state_rows": self.counts.get("planner", 0),
                "command_rows": self.planner_rows["commands"], "stale_rows": self.planner_rows["stale"],
                "stale_intervals": self.planner_stale, "tag_pivot": self.tag_pivot,
                "setpoint_steps": self.planner_steps, "heading_rehold": self.planner_rehold,
                "final_along": self.planner_final_along,
                "approach_clearance_m": self.mission["planner"]["approach_clearance_m"]}
            docking["frame_validation"] = docking_metric.frame_validation(self.static_edges, self.upstream_checks)
            docking["candidate_id"] = self.metadata["candidate_id"]
            docking["trial_status"] = self.metadata["status"]
            first_gt = next((row for row in self.summary_rows if row["kind"] == "dock"), None)
            self.metadata["observations"] = {
                "helm_reached_direct_control": self.direct_control_seen,
                "ground_truth_relative_pose_samples": self.counts.get("gt_auv", 0),
                "tag_detected": docking["perception"]["first_tag_detection_t"] is not None,
                "fused_dock_point_tf_samples": self.counts.get("fused_tf", 0),
                "fused_dock_point_pose_samples": self.counts.get("fused_dock", 0),
                "contact_events": {name: value["events"] for name, value in docking["contacts"].items()},
                "frame_validation": docking["frame_validation"]["status"],
                "first_ground_truth_t": first_gt["t"] if first_gt else None,
            }
            self.metadata["counts"] = self.counts
            self.metadata["thruster_topics"] = sorted(self.thruster_topics)
            self.metadata["invalid_numeric_events"] = self.invalid_count  # rows of both classes
            self.metadata["nonfinite_values"] = self.nonfinite.summary()
            self.metadata["last_helm_state_before_process_shutdown"] = self.helm_state
            self.metadata["simulator_seed"] = simulator_seed.read_record(self.root)  # launch.log is complete now
            self.stream.flush()
            os.fsync(self.stream.fileno())
            self.stream.close()
            for name, value in (("docking.json", docking), ("trial.json", self.metadata)):
                with (self.root / name).open("x", encoding="utf-8") as stream:
                    json.dump(finite_json(value), stream, indent=2, sort_keys=True, allow_nan=False)
                    stream.write("\n")
    return Trial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--expected-gains-json", required=True)
    parser.add_argument("--mission-profile-json", required=True)
    parser.add_argument("--context-json")
    parser.add_argument("--scenario", required=True, help="the wrapper scenario the simulator loads")
    parser.add_argument("--horizon-seconds", type=float, default=240)
    parser.add_argument("--wall-timeout-seconds", type=float, default=360)
    parser.add_argument("--ready-timeout-seconds", type=float, default=110)
    parser.add_argument("--launch-pid", type=int)
    args = parser.parse_args()
    if not all(math.isfinite(x) and 0 < x <= 3600 for x in
               (args.horizon_seconds, args.wall_timeout_seconds, args.ready_timeout_seconds)):
        parser.error("time bounds must be finite positive seconds <=3600")
    root = Path(args.output)
    root.mkdir(parents=True, exist_ok=True)
    for name in ("telemetry.jsonl", "trial.json", "docking.json"):
        if (root / name).exists():
            parser.error(f"refusing to overwrite {name}")
    from ament_index_python.packages import get_package_share_directory
    args.package_share = get_package_share_directory
    args.scenario_provenance = scenario_provenance(
        Path(args.scenario), Path(get_package_share_directory("world_of_stonefish")),
        Path(get_package_share_directory("race_auv_bringup")) / "config/sim_params.yaml")
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = ros_node_class()(args)

        def stop(signum, _frame):
            node.stop_requested = f"signal_{signum}"
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        try:
            node.run()
        except Exception as exc:
            node.metadata.update(status="failed", stop_reason=str(exc), error_type=type(exc).__name__)
            node.emit("event", event="failure", error=str(exc), error_type=type(exc).__name__)
        finally:
            node.finish()
        return 0 if node.metadata["status"] in ("completed", "budget_censored") else 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
