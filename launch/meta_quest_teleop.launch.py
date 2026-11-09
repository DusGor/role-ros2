"""
Meta Quest teleop bridge: dex_bimanual_telep teleop_joint_state (ZMQ) -> role_ros2 joint commands.

Defaults come from config/meta_quest_teleop_config.yaml; every key can be overridden as a launch
argument; setup (right | left | bimanual) is required, e.g. setup:=bimanual.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CONFIG_FILE = "meta_quest_teleop_config.yaml"

# (key, type) of every node parameter exposed as a launch argument.
PARAMS = [
    ("server_ip", str),
    ("port", int),
    ("frontend_port", int),
    ("setup", str),
    ("arm_command_blocking", bool),
    ("ramp_max_velocity_arm", float),
    ("ramp_max_velocity_hand", float),
    ("ramp_max_acceleration_arm", float),
    ("ramp_max_acceleration_hand", float),
    ("engage_min_duration", float),
    ("stream_timeout", float),
    ("max_velocity_arm", float),
    ("max_velocity_hand", float),
    ("hand_skeleton", str),
]


def load_config_yaml():
    path = os.path.join(get_package_share_directory("role_ros2"), "config", CONFIG_FILE)
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def to_arg_default(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    return "" if value is None else str(value)


def generate_launch_description():
    config = load_config_yaml()

    args = [
        DeclareLaunchArgument(
            key,
            default_value=to_arg_default(config.get(key)),
            description=f"Default from config/{CONFIG_FILE}",
        )
        for key, _ in PARAMS
    ]

    bridge = Node(
        package="role_ros2",
        executable="zmq_ros2_node",
        name="zmq_ros2_bridge",
        namespace="teleop",
        output="screen",
        parameters=[{
            key: ParameterValue(LaunchConfiguration(key), value_type=value_type)
            for key, value_type in PARAMS
        }],
    )

    return LaunchDescription(args + [bridge])
