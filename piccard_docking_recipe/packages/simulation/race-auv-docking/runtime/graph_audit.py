"""The vehicle-stack consumption audit: what the helm and controller consume, and whether ground truth reaches
the vehicle stack. Free of YAML and ROS, so collect_trial runs it in the trial and an offline evidence check can
recompute it from an archived graph; collect_trial imports it unchanged.
"""
from __future__ import annotations

import re

ROBOT = "race_auv"
NS = f"/{ROBOT}"
CONTROLLER_NODE, HELM_NODE = f"{NS}/mvp_control_ros_node", f"{NS}/mvp_helm"
EKF_ODOMETRY = f"{NS}/odometry/filtered"
WORLD_LINK, CHILD_LINK = f"{ROBOT}/world_ned", f"{ROBOT}/cg_link"
FUSED_PARENT, FUSED_CHILD = f"{ROBOT}/base_link", "race_station/dock_point"
GT_AUV, GT_STATION = f"{NS}/stonefish/odometry", "/race_station/stonefish/odometry"
GT_TF, GT_TF_STATIC = "/piccard/ground_truth/tf", "/piccard/ground_truth/tf_static"
GT_UPSTREAM_POSE = "/piccard/ground_truth/upstream_docking_pose"
CONTACTS = {"/piccard/contact/auv_station": "auv_station", "/piccard/contact/auv_tank": "auv_tank"}
GROUND_TRUTH_TOPICS = {GT_AUV, GT_STATION, GT_TF, GT_TF_STATIC, GT_UPSTREAM_POSE, *CONTACTS}
SCORING_NODES = {"/piccard_trial_collector", "/ground_truth_docking"}
# --onboard-camera-video (issue #86): the recorder is the only media node and may hear only the front camera image.
# The audit lists it under media_nodes (not subscriptions), so a recording run's trial.json differs in that key only.
MEDIA_NODES = {"/piccard_media_recorder"}
MEDIA_TOPICS = {f"{NS}/cam_front/stonefish/data/image_color"}
# Nodes whose TF is ground truth by construction. The simulator also holds a /tf broadcaster but only sends
# world_ned -> <robot>/base_link when a scenario sets ros_base_link_transform; the edge checks catch that.
GROUND_TRUTH_TF_PUBLISHERS = {"/ground_truth_docking", "/race_station/robot_state_publisher"}
VEHICLE_FRAME_PREFIXES = (f"{ROBOT}/", "race_station/")
# The only edge into the station's frames the vehicle TF tree may carry: the AprilTag fuser's estimate.
PERCEPTION_EDGES = {(FUSED_PARENT, FUSED_CHILD)}
# race_auv_camera_pkg 5944d5a apriltag_fuser (publish_tag_tfs, default on) also broadcasts one raw per-observation
# frame per detected tag: <reference_frame> -> apriltag_<family>_<id>. rclpy's message info carries no publisher gid,
# so these are accepted only while /apriltag_fuser is a /tf publisher; ground-truth publishers fail on their own.
PERCEPTION_TF_PUBLISHER = "/apriltag_fuser"
PERCEPTION_TAG_FRAME = re.compile(r"^apriltag_tag(36h11|25h9)_[0-9]+$")
# M3 planner (#109). The audit's allowances are stated here, not imported from the planner it audits.
PLANNER_NODE = "/piccard_planner"
PLANNER_TOPICS = {"/tf", "/tf_static", EKF_ODOMETRY}  # the fuser TF (with the vehicle tree) and the EKF odometry
PLANNER_HOUSEKEEPING = {"/parameter_events"}
PLANNER_TF_LOOKUPS = {f"{WORLD_LINK}->{FUSED_PARENT}", f"{FUSED_PARENT}->{FUSED_CHILD}",
                      f"{FUSED_PARENT}->{ROBOT}/auv_dock_point", f"{FUSED_PARENT}->{CHILD_LINK}"}


def planner_inputs(subscriptions: dict, expected: bool, lookups, setpoint_publishers) -> dict | None:
    """M3 (#109): what the planner node hears, and the TF edges it reports looking up. It may hear only the TF tree
    and the EKF odometry, and look up only PLANNER_TF_LOOKUPS; None when there is no planner and none is expected."""
    present = PLANNER_NODE in subscriptions
    if not (present or expected):
        return None
    topics = sorted(subscriptions.get(PLANNER_NODE, {}))
    lookups = sorted(lookups or [])
    extra_topics = sorted(set(topics) - PLANNER_TOPICS - PLANNER_HOUSEKEEPING)
    extra_lookups = sorted(set(lookups) - PLANNER_TF_LOOKUPS)
    return {"node": PLANNER_NODE, "present": present, "expected": expected, "subscriptions": topics,
            "allowed_topics": sorted(PLANNER_TOPICS), "tf_lookups": lookups,
            "allowed_tf_lookups": sorted(PLANNER_TF_LOOKUPS), "topics_outside_allowed": extra_topics,
            "tf_lookups_outside_allowed": extra_lookups, "setpoint_publishers": setpoint_publishers,
            "reads_only_fuser_tf_and_odometry": (present and bool(lookups) and not extra_topics
                                                 and not extra_lookups)}


def consumption_audit(subscriptions: dict, tf_publishers: dict, tf_edges: dict, planner_expected: bool = False,
                      planner_lookups=(), setpoint_publishers=None) -> dict:
    """What the helm and controller consume, and whether ground truth reaches the vehicle stack.
    subscriptions: {node: {topic: [types]}}; tf_publishers: {"/tf": [nodes], ...};
    tf_edges: {"parent->child": messages} observed on /tf and /tf_static; for a planner mission, the planner's own
    report of the TF edges it looks up and the set-point topic's publishers."""
    def odometry(node):
        return sorted(t for t, types in subscriptions.get(node, {}).items() if "nav_msgs/msg/Odometry" in types)

    ground_truth_consumers = {node: sorted(set(topics) & GROUND_TRUTH_TOPICS) for node, topics in subscriptions.items()
                              if node not in SCORING_NODES and set(topics) & GROUND_TRUTH_TOPICS}
    # tf2 C++ listeners run as their own nodes; the helm's and controller's TF arrives through them
    listeners = {node: sorted(topics) for node, topics in subscriptions.items()
                 if node.startswith(f"{NS}/transform_listener_impl")}
    parents = {}
    for edge in tf_edges:
        parent, child = edge.split("->")
        parents.setdefault(child, set()).add(parent)
    vehicle_tf_publishers = set(tf_publishers.get("/tf", [])) | set(tf_publishers.get("/tf_static", []))
    fuser_on_tf = PERCEPTION_TF_PUBLISHER in tf_publishers.get("/tf", [])

    def perception_tag_frame(edge):
        parent, child = edge.split("->")
        return fuser_on_tf and parent == FUSED_PARENT and bool(PERCEPTION_TAG_FRAME.match(child))
    audit = {"controller": {"node": CONTROLLER_NODE, "odometry": odometry(CONTROLLER_NODE)},
             "helm": {"node": HELM_NODE, "odometry": odometry(HELM_NODE)},
             "tf_listeners": listeners, "tf_publishers": tf_publishers, "tf_edges": tf_edges,
             "ground_truth_consumers_outside_scoring": ground_truth_consumers,
             "station_tf_edges_besides_perception": sorted(
                 edge for edge in tf_edges if "race_station/" in edge and tuple(edge.split("->")) not in PERCEPTION_EDGES),
             "perception_tag_frames": sorted(edge for edge in tf_edges if perception_tag_frame(edge)),
             "tf_edges_outside_vehicle_frames": sorted(
                 edge for edge in tf_edges if not perception_tag_frame(edge)
                 and not all(f.startswith(VEHICLE_FRAME_PREFIXES) for f in edge.split("->"))),
             "tf_frames_with_several_parents": {child: sorted(p) for child, p in parents.items() if len(p) > 1},
             "ground_truth_publishers_on_vehicle_tf": sorted(vehicle_tf_publishers & GROUND_TRUTH_TF_PUBLISHERS),
             # the base image's RISE packages stay on the underlay's package path; none may run
             "alpha_rise_nodes": sorted(node for node in subscriptions if "alpha_rise" in node),
             "media_nodes": {"allowed_topics": sorted(MEDIA_TOPICS),
                             "nodes": {node: sorted(topics) for node, topics in sorted(subscriptions.items())
                                       if node in MEDIA_NODES}}}
    planner = planner_inputs(subscriptions, planner_expected, planner_lookups, setpoint_publishers)
    if planner is not None:
        audit["planner_inputs"] = planner
    problems = ["alpha_rise_node_running"] if audit["alpha_rise_nodes"] else []
    if planner is not None:
        if planner["expected"] and not planner["present"]:
            problems.append("planner_not_in_graph")
        elif planner["present"] and not planner["expected"]:
            problems.append("planner_without_planner_mission")
        elif not planner["reads_only_fuser_tf_and_odometry"]:
            problems.append("planner_reads_other_inputs")
    for role in ("controller", "helm"):
        if audit[role]["node"] not in subscriptions:
            problems.append(f"{role}_not_in_graph")
        elif not set(audit[role]["odometry"]) <= {EKF_ODOMETRY}:
            problems.append(f"{role}_consumes_other_odometry")
    if CONTROLLER_NODE in subscriptions and audit["controller"]["odometry"] != [EKF_ODOMETRY]:
        problems.append("controller_not_on_ekf_odometry")
    if any(not set(topics) <= {"/tf", "/tf_static", "/parameter_events"} for topics in listeners.values()):
        problems.append("tf_listener_on_other_topics")
    if ground_truth_consumers:
        problems.append("ground_truth_consumed_by_vehicle_stack")
    if any(audit[key] for key in ("station_tf_edges_besides_perception", "tf_frames_with_several_parents",
                                  "ground_truth_publishers_on_vehicle_tf")):
        problems.append("ground_truth_tf_on_vehicle_tree")
    if any(not set(topics) <= MEDIA_TOPICS for topics in audit["media_nodes"]["nodes"].values()):
        problems.append("media_node_on_other_topics")
    if audit["tf_edges_outside_vehicle_frames"]:  # not provably ground truth, but not a known vehicle/perception frame
        problems.append("unknown_tf_frame_on_vehicle_tree")
    audit["problems"] = problems
    audit["assertion"] = ("helm and controller consume only the EKF odometry and the localization TF; ground truth "
                          "reaches only the scoring nodes; a media node hears only the front camera image"
                          + ("; the planner hears only the TF tree and the EKF odometry" if planner else "")
                          ) if not problems else None
    return audit
