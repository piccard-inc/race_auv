"""Headless RACE docking simulation for one race-auv-docking/v1 trial.

Mirrors race_auv_bringup/launch/bringup_simulation.launch.py (pinned race_auv da58963) with these
differences: the scenario is the Piccard wrapper (the upstream race_auv_test.scn, unchanged, plus contact
monitors); rviz, joystick and C2 are not started; the station's simulation bringup and upstream's ground-truth
node are added, both with /tf and /tf_static remapped to /piccard/ground_truth/*. That keeps ground truth out of
the vehicle's TF tree and leaves the AprilTag fuser as the only parent of race_station/dock_point (the station's
robot_state_publisher would otherwise publish a second parent, race_station/base_link). Simulator rate, window
size and rendering quality are upstream's. Launch arguments:
  scenario         absolute path of the wrapper scenario (required)
  apriltag_config  AprilTag pipeline config; empty uses upstream's config/simulation/apriltag.yaml
  variant          empty for the controller and helm includes prepare_candidate configured in place (pinned
                   image); "docking" for piccard/docking-recipe's committed mvp_control_docking_sim and
                   mvp_mission_docking_sim includes
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

ROBOT = "race_auv"
SIMULATION_RATE, WINDOW_X, WINDOW_Y, QUALITY = "100", "1200", "800", "high"  # as upstream simulation.launch.py
GROUND_TRUTH_TF = {"/tf": "/piccard/ground_truth/tf", "/tf_static": "/piccard/ground_truth/tf_static"}
GROUND_TRUTH_POSE = "/piccard/ground_truth/upstream_docking_pose"


def include(package: str, *path: str, arguments: dict | None = None) -> IncludeLaunchDescription:
    source = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package), *path))
    return IncludeLaunchDescription(source, launch_arguments=(arguments or {}).items())


def driver(executable: str, params: str, remappings=(), extra=None) -> Node:
    parameters = ([extra] if extra else []) + [params]
    return Node(package="world_of_stonefish", executable=executable, namespace=ROBOT, name=executable,
                remappings=list(remappings), parameters=parameters)


def ground_truth_scope(*actions) -> GroupAction:
    remaps = [SetRemap(src=src, dst=dst) for src, dst in GROUND_TRUTH_TF.items()]
    return GroupAction(remaps + list(actions), scoped=True)


def build(context):
    world = get_package_share_directory("world_of_stonefish")
    sim_params = os.path.join(get_package_share_directory(f"{ROBOT}_bringup"), "config", "sim_params.yaml")
    scenario = LaunchConfiguration("scenario").perform(context)
    apriltag_config = LaunchConfiguration("apriltag_config").perform(context)
    apriltag_args = {"config": apriltag_config} if apriltag_config else {}
    variant = LaunchConfiguration("variant").perform(context)
    if variant not in ("", "docking"):
        raise RuntimeError(f"unknown launch variant {variant!r}")
    suffix = "_docking" if variant else ""
    return [
        Node(package="stonefish_ros2", executable="stonefish_simulator", name="stonefish_simulator",
             arguments=[os.path.join(world, "data/"), scenario, SIMULATION_RATE, WINDOW_X, WINDOW_Y, QUALITY],
             parameters=[sim_params], output="screen"),  # scenario parse errors land in launch.log
        driver("imu_driver_node", sim_params, [("imu_in/data", "imu/stonefish/data"), ("imu_out/data", "ekf/imu/data")],
               {"frame_id": f"{ROBOT}/imu_sf"}),
        driver("thruster_driver_node", sim_params),
        driver("dvl_driver_node", sim_params, [(f"/{ROBOT}/dvl/twist", f"/{ROBOT}/dvl/raw_twist")]),
        driver("pressure_sensor_node", sim_params, [("depth", "depth/odometry")], {"frame_id": f"{ROBOT}/world"}),
        driver("modem_driver_node", sim_params),
        include(f"{ROBOT}_bringup", "launch", "include", "simulation", "localization_sim.launch.py"),
        include(f"{ROBOT}_bringup", "launch", "include", "description.launch.py"),
        include(f"{ROBOT}_bringup", "launch", "include", "simulation", f"mvp_control{suffix}_sim.launch.py"),
        include(f"{ROBOT}_bringup", "launch", "include", "simulation", f"mvp_mission{suffix}_sim.launch.py"),
        include(f"{ROBOT}_bringup", "launch", "include", "simulation", "apriltag_sim.launch.py", arguments=apriltag_args),
        ground_truth_scope(include("race_station_bringup", "launch", "bringup_simulation.launch.py"),
                           include("race_auv_sim_pkg", "launch", "ground_truth_pose.launch.py",
                                   arguments={"output_topic": GROUND_TRUTH_POSE})),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario"),
        DeclareLaunchArgument("apriltag_config", default_value=""),
        DeclareLaunchArgument("variant", default_value=""),
        OpaqueFunction(function=build),
    ])
