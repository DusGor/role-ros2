#!/usr/bin/env python3
"""
xArm (UF850) + Allegro Hand Robot Launch File

Starts:
  - xarm_robot_interface_node (/{setup}_arm): arm via the xArm SDK, not ros2_control.
  - controller_manager + spawners: the Allegro driver stack from allegro_hand_bringup
    (allegro_hand.launch.py) without its robot_state_publisher/rviz2. Gets a hand-only
    robot_description so it never drives the arm. Posture/grasp controllers only with
    hand_grasp_controllers:=true (BHand library needs pinocchio 3.6.0).
  - allegro_hand_interface_node (/{setup}_hand): role-ros2 wrapper around the driver.
  - robot_state_aggregator_node: merges arm and hand joint_states into /joint_states.
  - robot_state_publisher: combined uf850_allegro_{setup}.urdf -> /tf.

setup (right | left, required) selects one side of the bimanual setup: the model
uf850_allegro_{setup}.urdf (same names and base pose as that side of uf850_allegro_bimanual.urdf),
joints {setup}_joint1..6 / {setup}_hand_jointXY, namespaces {setup}_arm / {setup}_hand, frame
{setup}_link_base.

Usage:
    ros2 launch role_ros2 xarm_allegro_robot.launch.py setup:=right
    ros2 launch role_ros2 xarm_allegro_robot.launch.py setup:=left use_mock:=true hand_use_mock:=true
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


# Same topic as the dual-hand driver in bimanual_xarm_robot.launch.py.
HAND_JOINTS_RAW_TOPIC = "allegro_hands/joint_states_raw"


def load_config_yaml(package_name, config_file):
    """Load launch configuration YAML file from the package's share dir."""
    try:
        package_share_dir = get_package_share_directory(package_name)
        config_path = os.path.join(package_share_dir, "config", config_file)
        if os.path.exists(config_path):
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
                return config if config else {}
        print(f"Warning: Config file not found: {config_path}")
    except Exception as e:
        print(f"Warning: Could not load config file {config_file}: {e}")
    return {}


def build_allegro_control_actions(
    side_str, device_str, hardware_type, grasp_controllers: bool
):
    """Fork of allegro_hand_bringup/launch/allegro_hand.launch.py's control_node +
    spawners + diagnostic_aggregator, WITHOUT its own robot_state_publisher/rviz2
    (those would duplicate/conflict with this launch file's single combined
    robot_state_publisher). The control_node here is fed a hand-only
    robot_description, built the exact same way the official file builds it -
    never the combined arm+hand URDF used by robot_state_publisher below.
    """
    robot_description_str = Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [
                    FindPackageShare("allegro_hand_bringup"),
                    "config",
                    device_str,
                    "single_hand",
                    "allegro_hand.urdf.xacro",
                ]
            ),
            " hand:=",
            side_str,
            " ros2_control_hardware_type:=",
            hardware_type,
        ]
    )
    robot_description = {"robot_description": robot_description_str}

    ros2_controllers_path = PathJoinSubstitution(
        [
            FindPackageShare("allegro_hand_bringup"),
            "config",
            device_str,
            "single_hand",
            "ros2_controllers.yaml",
        ]
    )

    # Remapped so the driver's joint_states don't collide with the aggregated /joint_states.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        remappings=[("joint_states", HAND_JOINTS_RAW_TOPIC)],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    allegro_hand_imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "allegro_hand_imu_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    allegro_hand_controller_fcc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["allegro_hand_position_controller", "-c", "/controller_manager"],
    )
    allegro_hand_controller_gp_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "allegro_hand_posture_controller",
            "-c",
            "/controller_manager",
            "--inactive",
        ],
    )
    allegro_hand_controller_ge_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "allegro_hand_grasp_controller",
            "-c",
            "/controller_manager",
            "--inactive",
        ],
    )

    bringup_pkg_path = get_package_share_directory("allegro_hand_bringup")
    analyzer_config_path = os.path.join(
        bringup_pkg_path, "config", "v4", "single_hand", "diagnostic_analyzers.yaml"
    )
    with open(analyzer_config_path, "r") as f:
        analyzer_params = yaml.safe_load(f)

    diagnostic_aggregator_node = Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        name="diagnostic_aggregator",
        output="screen",
        parameters=[analyzer_params],
    )

    actions = [
        control_node,
        joint_state_broadcaster_spawner,
        allegro_hand_imu_broadcaster_spawner,
        allegro_hand_controller_fcc_spawner,
        diagnostic_aggregator_node,
    ]
    if grasp_controllers:
        # BHand library (allegro_hand_grasp_library) needs pinocchio 3.6.0.
        actions += [allegro_hand_controller_gp_spawner, allegro_hand_controller_ge_spawner]
    return actions


def create_xarm_allegro_nodes(
    context: LaunchContext,
    use_mock,
    robot_ip,
    publish_rate,
    auto_reset_on_startup,
    auto_reset_delay,
    setup,
    hand_use_mock,
    hand_grasp_controllers,
):
    """OpaqueFunction body — resolves substitutions and builds all Nodes."""
    use_mock_str = context.perform_substitution(use_mock)
    robot_ip_str = context.perform_substitution(robot_ip)
    publish_rate_str = context.perform_substitution(publish_rate)
    auto_reset_str = context.perform_substitution(auto_reset_on_startup)
    auto_reset_delay_str = context.perform_substitution(auto_reset_delay)
    side_str = context.perform_substitution(setup).strip().lower()
    hand_use_mock_str = context.perform_substitution(hand_use_mock)

    use_mock_bool = use_mock_str.lower() == "true"
    auto_reset_bool = auto_reset_str.lower() == "true"
    publish_rate_float = float(publish_rate_str)
    auto_reset_delay_float = float(auto_reset_delay_str)
    hand_use_mock_bool = hand_use_mock_str.lower() == "true"
    hand_grasp_controllers_bool = (
        context.perform_substitution(hand_grasp_controllers).lower() == "true"
    )

    if side_str not in ("left", "right"):
        raise ValueError(
            f"setup must be right or left (got {side_str!r}), e.g. "
            "ros2 launch role_ros2 xarm_allegro_robot.launch.py setup:=right; "
            "both sides: bimanual_xarm_robot.launch.py")
    # names as in uf850_allegro_bimanual.urdf / bimanual_xarm_robot.launch.py
    side_prefix = f"{side_str}_"
    arm_namespace_str = f"{side_str}_arm"
    hand_namespace_str = f"{side_str}_hand"

    config = load_config_yaml("role_ros2", "xarm_allegro_robot_config.yaml")
    arm_joints = [side_prefix + j for j in config.get("arm_joints", [])]
    if not arm_joints:
        raise ValueError(
            "xarm_allegro_robot_config.yaml is missing 'arm_joints'. "
            "Did you forget to colcon build?"
        )
    ee_frame_id = f"{side_prefix}link_base"
    home_joints = config.get("home_joints", [0.0] * len(arm_joints))
    max_joint_velocity = float(config.get("max_joint_velocity", 1.0))

    hand_config = load_config_yaml("role_ros2", "allegro_hand_config.yaml")
    hand_joint_names_driver = hand_config.get("hand_joint_names", [])
    if not hand_joint_names_driver:
        raise ValueError(
            "allegro_hand_config.yaml is missing 'hand_joint_names'. "
            "Did you forget to colcon build?"
        )
    driver_prefix = hand_config.get("prefix", "ah_")
    device_str = hand_config.get("device", "v4")
    hand_home_joints = hand_config.get(
        "home_joints", [0.0] * len(hand_joint_names_driver)
    )
    hand_move_tolerance = float(hand_config.get("move_tolerance", 0.02))
    hand_move_timeout = float(hand_config.get("move_timeout", 5.0))
    hand_auto_home_bool = bool(hand_config.get("auto_home_on_startup", False))
    hand_auto_home_delay = float(hand_config.get("auto_home_delay", 5.0))

    # Driver names (ah_*) -> URDF names ({side}_hand_*).
    urdf_hand_joint_names = [
        f"{side_prefix}hand_" + n[len(driver_prefix) :] if n.startswith(driver_prefix) else n
        for n in hand_joint_names_driver
    ]

    # Combined arm+hand URDF, matching the hand side.
    urdf_filename = f"uf850_allegro_{side_str}.urdf"
    urdf_filepath = os.path.join(
        get_package_share_directory("role_ros2"),
        "robot_ik",
        "xarm",
        urdf_filename,
    )
    if not os.path.exists(urdf_filepath):
        raise FileNotFoundError(
            f"URDF not found: {urdf_filepath}. "
            "Did you run colcon build?"
        )
    with open(urdf_filepath, "r") as f:
        robot_description = f.read()
    print(
        f"Info: loaded combined arm+hand URDF from {urdf_filepath} ({len(robot_description)} bytes)"
    )

    # Allow the launch arg to override the yaml value when explicitly passed.
    if not robot_ip_str:
        robot_ip_str = config.get("robot_ip", "192.168.1.185")

    print(
        f"Info: xarm+allegro launch — use_mock={use_mock_bool}, "
        f"namespace={arm_namespace_str}, robot_ip={robot_ip_str}, "
        f"n_dof={len(arm_joints)}, setup={side_str}, "
        f"hand_namespace={hand_namespace_str}, hand_use_mock={hand_use_mock_bool}"
    )

    # ---------------------------------------------------------------- arm
    arm_node_params = [
        {"use_mock": use_mock_bool},
        {"ip_address": robot_ip_str},
        {"publish_rate": publish_rate_float},
        {"namespace": arm_namespace_str},
        {"arm_joint_names": arm_joints},
        {"ee_frame_id": ee_frame_id},
        {"home_joints": home_joints},
        {"max_joint_velocity": max_joint_velocity},
        {"auto_reset_on_startup": auto_reset_bool},
        {"auto_reset_delay": auto_reset_delay_float},
    ]

    arm_node = GroupAction(
        [
            PushRosNamespace(arm_namespace_str),
            Node(
                package="role_ros2",
                executable="xarm_robot_interface",
                name="xarm_robot_interface_node",
                output="screen",
                parameters=arm_node_params,
            ),
        ]
    )

    # --------------------------------------------------------------- hand
    hardware_type = "mock_components" if hand_use_mock_bool else "physical_device"
    allegro_control_actions = build_allegro_control_actions(
        side_str,
        device_str,
        hardware_type,
        hand_grasp_controllers_bool,
    )

    hand_wrapper = GroupAction(
        [
            PushRosNamespace(hand_namespace_str),
            Node(
                package="role_ros2",
                executable="allegro_hand_interface",
                name="allegro_hand_interface_node",
                output="screen",
                parameters=[
                    {
                        "use_mock": hand_use_mock_bool,
                        "namespace": hand_namespace_str,
                        "hand_joint_names": urdf_hand_joint_names,
                        "driver_joint_names": hand_joint_names_driver,
                        "prefix": driver_prefix,
                        "publish_rate": publish_rate_float,
                        "home_joints": [float(v) for v in hand_home_joints],
                        "move_tolerance": hand_move_tolerance,
                        "move_timeout": hand_move_timeout,
                        "auto_home_on_startup": hand_auto_home_bool,
                        "auto_home_delay": hand_auto_home_delay,
                        "official_joint_states_topic": "/" + HAND_JOINTS_RAW_TOPIC,
                        "official_position_command_topic": "/allegro_hand_position_controller/commands",
                        "official_posture_action": "/allegro_hand_posture_controller/grasp_cmd",
                        "official_grasp_action": "/allegro_hand_grasp_controller/grasp_cmd",
                    }
                ],
            ),
        ]
    )

    # ---------------------------------------------------- shared aggregator
    aggregator_node = Node(
        package="role_ros2",
        executable="robot_state_aggregator",
        name="robot_state_aggregator_node",
        output="screen",
        parameters=[
            {"robot_namespaces": [arm_namespace_str, hand_namespace_str]},
            {"publish_rate": publish_rate_float},
            {"timeout_threshold": 1.0},
        ],
    )

    # Root-level robot_state_publisher: read /joint_states, publish /tf
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    return (
        [arm_node]
        + allegro_control_actions
        + [hand_wrapper, aggregator_node, robot_state_publisher_node]
    )


def generate_launch_description():
    config = load_config_yaml("role_ros2", "xarm_allegro_robot_config.yaml")
    hand_config = load_config_yaml("role_ros2", "allegro_hand_config.yaml")

    use_mock_arg = DeclareLaunchArgument(
        "use_mock",
        default_value=str(config.get("use_mock", False)).lower(),
        description="Use MockXArmInterface (no hardware connection).",
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value=config.get("robot_ip", "192.168.1.185"),
        description="IP address of the xArm controller box.",
    )
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value=str(config.get("publish_rate", 50.0)),
        description="State publish rate (Hz).",
    )
    auto_reset_on_startup_arg = DeclareLaunchArgument(
        "auto_reset_on_startup",
        default_value=str(config.get("auto_reset_on_startup", False)).lower(),
        description="Drive robot to home_joints on startup.",
    )
    auto_reset_delay_arg = DeclareLaunchArgument(
        "auto_reset_delay",
        default_value=str(config.get("auto_reset_delay", 5.0)),
        description="Delay (s) before auto-reset, if enabled.",
    )
    setup_arg = DeclareLaunchArgument(
        "setup",
        default_value="",
        description=(
            "Required: which side of the bimanual setup, right | left. Selects the model "
            "(uf850_allegro_{setup}.urdf), the joint/frame prefix, the namespaces and the "
            "Allegro hand variant."
        ),
    )
    hand_use_mock_arg = DeclareLaunchArgument(
        "hand_use_mock",
        default_value=str(hand_config.get("use_mock", False)).lower(),
        description=(
            "Use ros2_control mock_components for the hand instead of the real "
            "CAN driver. Independent of use_mock, which only affects the arm."
        ),
    )
    hand_grasp_controllers_arg = DeclareLaunchArgument(
        "hand_grasp_controllers",
        default_value="false",
        description=(
            "Spawn the Allegro posture/grasp controllers (needed only for the reset and "
            "grasp_primitive services). Requires allegro_hand_grasp_controllers built "
            "against pinocchio 3.6.0 (Wonik prebuilt BHand library)."
        ),
    )

    return LaunchDescription(
        [
            use_mock_arg,
            robot_ip_arg,
            publish_rate_arg,
            auto_reset_on_startup_arg,
            auto_reset_delay_arg,
            setup_arg,
            hand_use_mock_arg,
            hand_grasp_controllers_arg,
            OpaqueFunction(
                function=create_xarm_allegro_nodes,
                kwargs={
                    "use_mock": LaunchConfiguration("use_mock"),
                    "robot_ip": LaunchConfiguration("robot_ip"),
                    "publish_rate": LaunchConfiguration("publish_rate"),
                    "auto_reset_on_startup": LaunchConfiguration(
                        "auto_reset_on_startup"
                    ),
                    "auto_reset_delay": LaunchConfiguration("auto_reset_delay"),
                    "setup": LaunchConfiguration("setup"),
                    "hand_use_mock": LaunchConfiguration("hand_use_mock"),
                    "hand_grasp_controllers": LaunchConfiguration("hand_grasp_controllers"),
                },
            ),
        ]
    )
