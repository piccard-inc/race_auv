"""The RACE docking planner node with protocol v1.5's parameters (config/docking_planner_v1_5.yaml).

Start it with the docking simulation (race_auv_bringup bringup_docking_simulation.launch.py) once the controller holds
the fallback pose; the planner then waits until /piccard_planner/start is called. params_file selects another
parameter file. The node has no remappings: its topics and TF edges are the ones the planner notes name.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(get_package_share_directory('race_auv_docking_planner'), 'config',
                           'docking_planner_v1_5.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default,
                              description='ROS parameter file with every planner parameter and initial_setpoint'),
        Node(package='race_auv_docking_planner', executable='docking_planner', name='piccard_planner',
             parameters=[LaunchConfiguration('params_file')], output='screen'),
    ])
