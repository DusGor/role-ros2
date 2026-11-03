#!/usr/bin/env python3
# Copyright (c) 2024
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Bimanual Xarm Robot Launch File

Starts two UF850 arms (left_arm, right_arm) with xarm_robot_interface_node,
the aggregator and one robot_state_publisher with the combined bimanual model.
Allegro hands are not started yet.

Usage:
    ros2 launch role_ros2 bimanual_xarm_robot.launch.py
    ros2 launch role_ros2 bimanual_xarm_robot.launch.py use_mock:=true
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace



def load_config_yaml(package_name, config_file):
    """Load launch configuration YAML file."""
    try:
        package_share_dir = get_package_share_directory(package_name)
        config_path = os.path.join(package_share_dir, "config", config_file)
        if os.path.exists(config_path):
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
                return config if config else {}
        else:
            print(f"Warning: Config file not found: {config_path}")
    except Exception as e:
        print(f"Warning: Could not load config file {config_file}: {e}")
    return {}




def create_bimanual_nodes(context: LaunchContext, use_mock, publish_rate):
    """Create left/right arm nodes with joint names from bimanual config."""
    use_mock_str = context.perform_substitution(use_mock)
    publish_rate_str = context.perform_substitution(publish_rate)
    use_mock_bool = use_mock_str.lower() == "true"
    publish_rate_float = float(publish_rate_str)

    config = load_config_yaml("role_ros2", "bimanual_xarm_robot_config.yaml")
    if not config:
        raise RuntimeError("Failed to load bimanual_xarm_robot_config.yaml")

    left = config.get("left_arm", {})
    right = config.get("right_arm", {})
    urdf_file = config.get("urdf_file", "uf850_allegro_bimanual.urdf")
    # use_mock:=true overrides the per-arm setting
    if use_mock_bool:
        left_use_mock = right_use_mock = True
    else:
        left_use_mock = (
            left.get("use_mock", False)
            if isinstance(left.get("use_mock"), bool)
            else False
        )
        right_use_mock = (
            right.get("use_mock", False)
            if isinstance(right.get("use_mock"), bool)
            else False
        )

    left_arm_ip = left.get("robot_ip", "")
    right_arm_ip = right.get("robot_ip", "")

    package_share_dir = get_package_share_directory("role_ros2")
    urdf_path = os.path.join(package_share_dir, "robot_ik", "xarm", urdf_file)
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # ---------- Left arm node ----------
    left_arm_params = [
        {"use_mock": left_use_mock},
        {"ip_address": left_arm_ip},
        {"publish_rate": float(left.get("publish_rate", publish_rate_float))},
        {"namespace": left.get("arm_namespace", "left_arm")},
        {"auto_reset_on_startup": left.get("auto_reset_on_startup", False)},
        {"auto_reset_delay": float(left.get("auto_reset_delay", 5.0))},
        {"arm_joint_names": left.get("arm_joints", [])},
        {"ee_frame_id": left.get("ee_frame_id", "left_link_base")},
        {"home_joints": left.get("home_joints", [0.0] * 6)},
    ]
    left_arm_node = GroupAction(
            [
                PushRosNamespace(left.get("arm_namespace", "left_arm")),
                Node(
                    package="role_ros2",
                    executable="xarm_robot_interface",
                    name="xarm_robot_interface_node",
                    output="screen",
                    parameters=left_arm_params,
                ),
            ]
        )

    # ---------- Right arm node ----------
    right_arm_params = [
        {"use_mock": right_use_mock},
        {"ip_address": right_arm_ip},
        {"publish_rate": float(right.get("publish_rate", publish_rate_float))},
        {"namespace": right.get("arm_namespace", "right_arm")},
        {"auto_reset_on_startup": right.get("auto_reset_on_startup", False)},
        {"auto_reset_delay": float(right.get("auto_reset_delay", 5.0))},
        {"arm_joint_names": right.get("arm_joints", [])},
        {"ee_frame_id": right.get("ee_frame_id", "right_link_base")},
        {"home_joints": right.get("home_joints", [0.0] * 6)},
    ]

    right_arm_node = GroupAction(
            [
                PushRosNamespace(right.get("arm_namespace", "right_arm")),
                Node(
                    package="role_ros2",
                    executable="xarm_robot_interface",
                    name="xarm_robot_interface_node",
                    output="screen",
                    parameters=right_arm_params,
                ),
            ]
        )


    # ---------- Robot state aggregator (all namespaces) ----------
    robot_namespaces = [
        left.get("arm_namespace", "left_arm"),
        right.get("arm_namespace", "right_arm"),
    ]

    agg_timeout = float(
        left.get("timeout_threshold", right.get("timeout_threshold", 1.0))
    )
    aggregator_node = Node(
        package="role_ros2",
        executable="robot_state_aggregator",
        name="robot_state_aggregator_node",
        output="screen",
        parameters=[
            {"robot_namespaces": robot_namespaces},
            {"publish_rate": publish_rate_float},
            {"timeout_threshold": agg_timeout},
        ],
    )

    # ---------- Robot state publisher (bimanual URDF) ----------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    return [left_arm_node, right_arm_node, aggregator_node, robot_state_publisher_node]



def generate_launch_description():
    config = load_config_yaml("role_ros2", "bimanual_xarm_robot_config.yaml")
    left = config.get("left_arm", {})

    def get_default(key, default_value):
        value = left.get(key, default_value)
        if isinstance(value, bool):
            return "true" if value else "false"
        return str(value)

    use_mock = LaunchConfiguration("use_mock", default=get_default("use_mock", "false"))
    publish_rate = LaunchConfiguration(
        "publish_rate", default=get_default("publish_rate", "50.0")
    )


    robot_nodes_opaque = OpaqueFunction(
        function=create_bimanual_nodes,
        args=[use_mock, publish_rate],
    )


    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_mock",
                default_value=get_default("use_mock", "false"),
                description="Use mock interfaces (no real robot)",
            ),
            DeclareLaunchArgument(
                "publish_rate",
                default_value=get_default("publish_rate", "50.0"),
                description="State publishing rate (Hz)",
            ),
            robot_nodes_opaque,
        ]
    )
