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

Starts:
  - two UF850 arms (left_arm, right_arm) with xarm_robot_interface_node.
  - both Allegro hands (hands:=true): one controller_manager with allegro_hand_bringup's
    config/v4/dual_hand (allegro_hand_duo.launch.py without its robot_state_publisher/rviz2;
    right ahr_ on can0 / hand_id 0, left ahl_ on can1 / hand_id 1) and one
    allegro_hand_interface_node per hand (right_hand, left_hand). Posture/grasp controllers only
    with hand_grasp_controllers:=true (BHand library needs pinocchio 3.6.0).
  - the aggregator and one robot_state_publisher with the combined bimanual model.

Usage:
    ros2 launch role_ros2 bimanual_xarm_robot.launch.py
    ros2 launch role_ros2 bimanual_xarm_robot.launch.py use_mock:=true hand_use_mock:=true
    ros2 launch role_ros2 bimanual_xarm_robot.launch.py use_mock:=true hands:=false
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
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

HAND_PREFIXES = {"right": "ahr_", "left": "ahl_"}  # allegro_hand_bringup config/v4/dual_hand
HAND_SUFFIXES = {"right": "_r", "left": "_l"}
HAND_JOINTS_RAW_TOPIC = "allegro_hands/joint_states_raw"



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




def build_dual_allegro_control_actions(device_str, hardware_type, grasp_controllers: bool):
    """Fork of allegro_hand_bringup/launch/allegro_hand_duo.launch.py's control_node, spawners and
    diagnostic_aggregator, without its robot_state_publisher/rviz2 (this file runs one for the
    whole bimanual model). The control_node gets the hands-only dual_hand description."""
    dual_dir = ["config", device_str, "dual_hand"]
    robot_description = {"robot_description": Command([
        "xacro ",
        PathJoinSubstitution([FindPackageShare("allegro_hand_bringup")] + dual_dir + ["allegro_hand.urdf.xacro"]),
        " ros2_control_hardware_type:=", hardware_type,
    ])}
    controllers = PathJoinSubstitution(
        [FindPackageShare("allegro_hand_bringup")] + dual_dir + ["ros2_controllers.yaml"])

    # Remapped so the driver's joint_states don't collide with the aggregated /joint_states.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers],
        remappings=[("joint_states", HAND_JOINTS_RAW_TOPIC)],
        output="screen",
    )

    def spawner(name, inactive=False):
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "-c", "/controller_manager"] + (["--inactive"] if inactive else []))

    actions = [control_node, spawner("joint_state_broadcaster")]
    for suffix in HAND_SUFFIXES.values():
        actions.append(spawner(f"allegro_hand_position_controller{suffix}"))
        if grasp_controllers:
            actions += [spawner(f"allegro_hand_posture_controller{suffix}", True),
                        spawner(f"allegro_hand_grasp_controller{suffix}", True)]

    analyzer_path = os.path.join(get_package_share_directory("allegro_hand_bringup"),
                                 *dual_dir, "diagnostic_analyzers.yaml")
    with open(analyzer_path, "r") as f:
        analyzer_params = yaml.safe_load(f)
    actions.append(Node(package="diagnostic_aggregator", executable="aggregator_node",
                        name="diagnostic_aggregator", output="screen", parameters=[analyzer_params]))
    return actions


def create_hand_interface_node(side, namespace, hand_config, publish_rate, use_mock):
    """role-ros2 wrapper for one hand of the dual_hand driver: ah{r,l}_* -> {side}_hand_* (URDF)."""
    prefix, suffix = HAND_PREFIXES[side], HAND_SUFFIXES[side]
    joints = [f"joint{finger}{k}" for finger in range(4) for k in range(4)]
    return GroupAction([
        PushRosNamespace(namespace),
        Node(
            package="role_ros2",
            executable="allegro_hand_interface",
            name="allegro_hand_interface_node",
            output="screen",
            parameters=[{
                "use_mock": use_mock,
                "namespace": namespace,
                "hand_joint_names": [f"{side}_hand_{j}" for j in joints],
                "driver_joint_names": [prefix + j for j in joints],
                "prefix": prefix,
                "publish_rate": publish_rate,
                "home_joints": [float(v) for v in hand_config.get("home_joints", [0.0] * 16)],
                "move_tolerance": float(hand_config.get("move_tolerance", 0.02)),
                "move_timeout": float(hand_config.get("move_timeout", 5.0)),
                "auto_home_on_startup": bool(hand_config.get("auto_home_on_startup", False)),
                "auto_home_delay": float(hand_config.get("auto_home_delay", 5.0)),
                "official_joint_states_topic": "/" + HAND_JOINTS_RAW_TOPIC,
                "official_position_command_topic": f"/allegro_hand_position_controller{suffix}/commands",
                "official_posture_action": f"/allegro_hand_posture_controller{suffix}/grasp_cmd",
                "official_grasp_action": f"/allegro_hand_grasp_controller{suffix}/grasp_cmd",
            }],
        ),
    ])


def create_bimanual_nodes(context: LaunchContext, use_mock, publish_rate, hands, hand_use_mock,
                          hand_grasp_controllers):
    """Create left/right arm nodes with joint names from bimanual config, plus both hands."""
    use_mock_str = context.perform_substitution(use_mock)
    publish_rate_str = context.perform_substitution(publish_rate)
    use_mock_bool = use_mock_str.lower() == "true"
    publish_rate_float = float(publish_rate_str)
    hands_bool = context.perform_substitution(hands).lower() == "true"
    hand_use_mock_bool = context.perform_substitution(hand_use_mock).lower() == "true"
    hand_grasp_bool = context.perform_substitution(hand_grasp_controllers).lower() == "true"

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


    # ---------- Allegro hands ----------
    hands_cfg = config.get("hands", {})
    hand_nodes, hand_namespaces = [], []
    if hands_bool:
        hand_config = load_config_yaml("role_ros2", "allegro_hand_config.yaml")
        hardware_type = "mock_components" if hand_use_mock_bool else "physical_device"
        hand_nodes = build_dual_allegro_control_actions(
            hand_config.get("device", "v4"), hardware_type, hand_grasp_bool)
        for side in ("right", "left"):
            ns = hands_cfg.get(f"{side}_namespace", f"{side}_hand")
            hand_namespaces.append(ns)
            hand_nodes.append(create_hand_interface_node(
                side, ns, hand_config, publish_rate_float, hand_use_mock_bool))
    print(f"Info: bimanual launch - use_mock={use_mock_bool}, hands={hands_bool}, "
          f"hand_use_mock={hand_use_mock_bool}")

    # ---------- Robot state aggregator (all namespaces) ----------
    robot_namespaces = [
        left.get("arm_namespace", "left_arm"),
        right.get("arm_namespace", "right_arm"),
    ] + hand_namespaces

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

    return [left_arm_node, right_arm_node] + hand_nodes + [aggregator_node, robot_state_publisher_node]



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


    hands_cfg = config.get("hands", {})
    hands_default = "true" if hands_cfg.get("enabled", True) else "false"
    hands = LaunchConfiguration("hands", default=hands_default)
    hand_use_mock = LaunchConfiguration("hand_use_mock", default="false")
    hand_grasp_controllers = LaunchConfiguration("hand_grasp_controllers", default="false")

    robot_nodes_opaque = OpaqueFunction(
        function=create_bimanual_nodes,
        args=[use_mock, publish_rate, hands, hand_use_mock, hand_grasp_controllers],
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
            DeclareLaunchArgument(
                "hands",
                default_value=hands_default,
                description="Start both Allegro hands (dual_hand driver + interface nodes)",
            ),
            DeclareLaunchArgument(
                "hand_use_mock",
                default_value="false",
                description="Allegro hands on mock_components instead of CAN (can0 right, can1 left)",
            ),
            DeclareLaunchArgument(
                "hand_grasp_controllers",
                default_value="false",
                description=(
                    "Spawn the Allegro posture/grasp controllers (reset and grasp_primitive "
                    "services). Requires allegro_hand_grasp_controllers built against pinocchio "
                    "3.6.0 (Wonik prebuilt BHand library)."
                ),
            ),
            robot_nodes_opaque,
        ]
    )
