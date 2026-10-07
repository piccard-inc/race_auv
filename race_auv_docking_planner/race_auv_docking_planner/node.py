"""The planner's ROS 2 node: looks up the planner's inputs, ticks planner.StagePlanner and publishes its command.

Subscriptions: the EKF odometry (its age gates every tick), and /tf and /tf_static through tf2_ros's listener, looked
up for the four edges of planner.PLANNER_TF_LOOKUPS only. Publications: the set points on the helm's
bhv_direct_control desired_setpoints, and one JSON state record per tick on /piccard/planner/state. Service:
/piccard_planner/start (std_srvs/Trigger), answered with the parameters' hash. Parameters: the planner's
(parameters.PLANNER_FIELDS) and initial_setpoint.{x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad}, all required;
config/docking_planner_v1_5.yaml holds protocol v1.5's. This is piccard-physical-ai runtime/planner_fused_dock.py's
node at 468d65f6, reading ROS parameters where that read the trial's mission file. ROS imports are lazy, so the
offline tests run without ROS.
"""
from __future__ import annotations

import json
import time

from race_auv_docking_planner.parameters import (INITIAL_SETPOINT, PLANNER_FIELDS, PLANNER_VECTORS, SETPOINT_FIELDS,
                                                 from_ros_parameters, parameters_sha256)
from race_auv_docking_planner.planner import (AUV_DOCK, BASE_LINK, CG_LINK, EKF_ODOMETRY, NODE, PLANNER_TF_LOOKUPS,
                                              SETPOINT_TOPIC, START_SERVICE, STATE_TOPIC, STATION_DOCK,
                                              WAITING_STATE_PERIOD_S, WORLD_LINK, StagePlanner, quat_matrix)

ARRAY_PARAMETERS = ("standoffs_m", *PLANNER_VECTORS)


def gather(lookup, static: dict, odometry_age_s: float | None, max_age_s: float) -> dict | None:
    """The planner's inputs, or None while any is missing or too old. lookup(parent, child, max_age=None) returns
    (transform, stamp) or None; the two static edges are looked up once and kept in static."""
    if odometry_age_s is None or odometry_age_s > max_age_s:
        return None
    for child, key in ((AUV_DOCK, "auv_dock_in_base"), (CG_LINK, "cg_in_base")):  # static: look up once
        if key not in static:
            found = lookup(BASE_LINK, child)
            if found is None:
                return None
            static[key] = found[0]
    station = lookup(BASE_LINK, STATION_DOCK, max_age_s)
    world = lookup(WORLD_LINK, BASE_LINK)
    if station is None or world is None:
        return None
    return {**static, "station_dock_in_base": station[0], "station_stamp": station[1], "base_in_world": world[0]}


def ros_node_class():
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener
    from mvp_msgs.msg import ControlProcess

    reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)

    class Planner(Node):
        def __init__(self):
            super().__init__(NODE)
            names = [*sorted(PLANNER_FIELDS), *(f"{INITIAL_SETPOINT}.{key}" for key in SETPOINT_FIELDS)]
            for name in names:  # no defaults: every value comes from the parameter file
                self.declare_parameter(name, Parameter.Type.DOUBLE_ARRAY if name in ARRAY_PARAMETERS
                                       else Parameter.Type.DOUBLE)
            self.parameters, initial = from_ros_parameters({name: self.get_parameter(name).value for name in names})
            self.parameters_sha256 = parameters_sha256(self.parameters)
            self.planner = StagePlanner(self.parameters, initial=initial)
            self.created = time.monotonic()  # the planner's clock, waiting ticks included
            self.started = None
            self.ticks = 0
            self.last_waiting = 0.0
            self.odometry_received = None
            self.static = {}
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(Odometry, EKF_ODOMETRY, self.odometry, qos_profile_sensor_data)
            self.setpoints = self.create_publisher(ControlProcess, SETPOINT_TOPIC, 10)
            self.state = self.create_publisher(String, STATE_TOPIC, reliable)
            self.create_service(Trigger, START_SERVICE, self.start)
            self.create_timer(1.0 / self.parameters["tick_hz"], self.tick)
            self.get_logger().info(f"protocol parameters sha256 {self.parameters_sha256}; waiting for {START_SERVICE}")

        def odometry(self, _msg):
            self.odometry_received = time.monotonic()

        def start(self, _request, response):
            if self.started is None:
                self.started = time.monotonic()
            response.success, response.message = True, self.parameters_sha256
            return response

        def lookup(self, parent, child, max_age=None):
            """((rotation, position), stamp) of the latest transform, or None if missing or older than max_age."""
            try:
                transform = self.tf_buffer.lookup_transform(parent, child, Time())
            except Exception:
                return None
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
            if max_age is not None and self.get_clock().now().nanoseconds / 1e9 - stamp > max_age:
                return None
            q, v = transform.transform.rotation, transform.transform.translation
            return (quat_matrix(q.x, q.y, q.z, q.w), [v.x, v.y, v.z]), stamp

        def estimate(self):
            age = None if self.odometry_received is None else time.monotonic() - self.odometry_received
            return gather(self.lookup, self.static, age, self.parameters["max_estimate_age_s"])

        def publish_state(self, record):
            msg = String()
            msg.data = json.dumps({**record, "tick": self.ticks, "parameters_sha256": self.parameters_sha256,
                                   "tf_lookups": [f"{p}->{c}" for p, c in PLANNER_TF_LOOKUPS]},
                                  separators=(",", ":"), allow_nan=False)
            self.state.publish(msg)

        def tick(self):
            now = time.monotonic()
            if self.started is None:  # the fallback pose's dwell: observe only (planner notes, 2)
                self.planner.wait(now - self.created, self.estimate())
                if now - self.last_waiting >= WAITING_STATE_PERIOD_S:
                    self.last_waiting = now
                    self.publish_state({"state": "waiting", "event": None, "stage": None, "command": None,
                                        "heading_samples": len(self.planner.headings)})
                return
            self.ticks += 1
            record = self.planner.step(now - self.created, self.estimate())
            command = record["command"]
            if command is not None:
                msg = ControlProcess()
                msg.header.frame_id, msg.child_frame_id, msg.control_mode = WORLD_LINK, CG_LINK, "docking"
                msg.position.x, msg.position.y, msg.position.z = (float(command[k]) for k in ("x_m", "y_m", "z_m"))
                msg.orientation.x, msg.orientation.y, msg.orientation.z = (
                    float(command[k]) for k in ("roll_rad", "pitch_rad", "yaw_rad"))
                self.setpoints.publish(msg)
            self.publish_state({**record, "t": now - self.started})

    return Planner, rclpy


def main(argv: list[str] | None = None) -> int:
    Planner, rclpy = ros_node_class()
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=argv)
    node = Planner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
