# role-ros2: Robot Learning Full Stack on ROS2

<div align="center">

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Foxy-brightgreen)](https://www.ros.org/)

</div>

<div align="center">
  <img src="assets/banner.png" width="90%" alt="role-ros2 Architecture Overview"/>
</div>

<br>

**role-ros2** (Robot Learning ROS2) is a unified robot learning full stack built on the ROS2 open-source ecosystem. It provides end-to-end tools for data collection (VR teleoperation), labeling, conversion, and camera calibration, with Docker-based separation of robot hardware and algorithms for modularity.

- An integrated toolchain for robot learning: VR data collection, labeling, conversion to TFDS/RLDS, and Charuco-based hand-eye calibration—all in one platform.

- Docker container separation of camera/GPU and robot control, enabling modular deployment and clean hardware–software decoupling.

- VR teleoperation with Oculus Quest controlling Franka Panda (single-arm and bimanual) for real-time trajectory collection.

- DROID / TFDS unified data format compatible with RLDS schema, ready for LeRobot and other training frameworks.

- A Gym-compatible `RobotEnv` interface for imitation learning and reinforcement learning research.

## Quick Start

### Prerequisites

- **Docker** and **Docker Compose**
- **NVIDIA GPU** (optional, for ZED cameras): Requires `nvidia-container-toolkit`
- **X11** (optional, for GUI apps like rviz2): `xhost +local:root` on host

### Full Stack (Camera + Robot)

```bash
# 1. Build and start Docker (from repository root)
docker compose -f docker/ros2_cu118_franka_0.18.x/docker-compose.yaml build
docker compose -f docker/ros2_cu118_franka_0.18.x/docker-compose.yaml up -d

# 2. Enter container and build workspace
docker exec -it ros2_cu118_container bash
cd /app/ros2_ws && colcon build --symlink-install && source install/setup.bash

# 3. Launch bringup control center (from project root)
cd /app/ros2_ws/src/role-ros2 && python3 scripts/bringup.py
```

> [!IMPORTANT]
> For detailed installation, environment selection (ros2_cu118, ros2_franka, full stack), X11 setup, and troubleshooting, see [docker/install.md](docker/install.md).

## Robots & Control

role-ros2 provides unified robot interfaces and VR-based teleoperation:

| Robot | Class | Use Case |
|-------|-------|----------|
| Single-arm Franka | `FrankaRobot` | One-arm manipulation |
| Bimanual Franka | `BimanualFrankaRobot` | Two-arm manipulation |
| UFACTORY xArm6 (beta) | `xarm_robot_interface` node | One-arm manipulation via xArm-Python-SDK |

**VR control (Oculus Quest)**:
- **Hold GRIP** → Enable robot movement
- **Long press A/X** (0.5 s) → **SUCCESS**: Save trajectory, reset, start new
- **Long press B/Y** (0.5 s) → **FAILURE**: Discard, reset, start new

**Action spaces**: `cartesian_velocity`, `cartesian_position`, `joint_velocity`, `joint_position`

```python
from role_ros2.robot.franka.robot import FrankaRobot
import rclpy

rclpy.init()
robot = FrankaRobot()

# Move to joint position
robot.update_joints([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], velocity=False)

# Move to Cartesian position
robot.update_pose([0.5, 0.0, 0.5, 0.0, 0.0, 0.0], velocity=False)

# Control gripper
robot.update_gripper(0.04)  # Open to 4 cm
```

### UF850 + Allegro Hand (single arm / bimanual)

Uses two external repositories in addition to role-ros2. They are only needed for this
setup (Franka / xArm6-only users can skip this):

| Repository | Branch | Provides |
|---|---|---|
| [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) | `humble` | `xarm_description` (UF850 meshes) |
| [DusGor/allegro_hand_ros2_v4](https://github.com/DusGor/allegro_hand_ros2_v4) (fork of [Wonik's](https://github.com/Wonikrobotics-git/allegro_hand_ros2_v4), right thumb tip fix) | `hand_description_fix` | Allegro V4 driver, controllers, `allegro_hand_v4_description` |

**Recommended: Docker.** The `ros2_xarm_allegro` image contains everything (xArm SDK,
ros2_control controllers, both repos pinned to xarm_ros2 `62936f7` and allegro_hand_ros2
`aecb1e9`). See [docker/ros2_xarm_allegro/README.md](docker/ros2_xarm_allegro/README.md).

```bash
docker compose -f docker/ros2_xarm_allegro/docker-compose.yaml build
docker compose -f docker/ros2_xarm_allegro/docker-compose.yaml up -d
docker exec -it ros2_xarm_allegro_container bash
ros2 launch role_ros2 xarm_allegro_robot.launch.py setup:=right use_mock:=true hand_use_mock:=true
```

**Without Docker:** install the apt and pip packages listed in
[`docker/ros2_xarm_allegro/Dockerfile`](docker/ros2_xarm_allegro/Dockerfile), then:

```bash
cd /app/ros2_ws/src
git clone -b humble https://github.com/xArm-Developer/xarm_ros2.git
git clone -b hand_description_fix https://github.com/DusGor/allegro_hand_ros2_v4.git allegro_hand_ros2
touch allegro_hand_ros2/allegro_hand_hardwares/gazebo/plugin/COLCON_IGNORE
cd /app/ros2_ws
colcon build --symlink-install \
  --packages-up-to role_ros2 xarm_description \
    allegro_hand_bringup allegro_hand_control_msgs allegro_hand_v4_description \
    allegro_hand_v4_hardware allegro_hand_position_effort_controller
source install/setup.bash
```

Do not build all of `xarm_ros2` (its MoveIt/Gazebo packages are not needed and require
extra dependencies). Clone `allegro_hand_ros2` only once — either here or inside
`role-ros2/` as described in `docker/ros2_allegro/README.md`, not both.

The Allegro posture/grasp primitives (`reset` and `grasp_primitive` services) are disabled
by default (`hand_grasp_controllers:=false`): Wonik's prebuilt grasp library requires
pinocchio 3.6.0, while ROS Humble currently ships 4.0.0. Joint-position control
(teleoperation, learned policies) is unaffected.

Launch (mock): `ros2 launch role_ros2 xarm_allegro_robot.launch.py setup:=right use_mock:=true hand_use_mock:=true`
— bimanual: `ros2 launch role_ros2 bimanual_xarm_robot.launch.py use_mock:=true hand_use_mock:=true`.
Model sources and regeneration: [`role_ros2/robot_ik/xarm/xacro/README.md`](role_ros2/robot_ik/xarm/xacro/README.md).

### Robot Environment (Gym-compatible)

```python
from role_ros2.robot_env import RobotEnv
import rclpy

rclpy.init()
env = RobotEnv()

obs = env.get_observation()
# obs: 'robot_state', 'images' (camera dict)

action = env.action_space.sample()
obs, reward, done, info = env.step(action)
```

## Data Pipeline

role-ros2 collects trajectories via VR, labels them, and converts to TFRecord/TFDS for training:

```mermaid
flowchart LR
    VR[OculusQuest] --> Collect[collect_trajectory]
    Collect --> HDF5[trajectory.h5]
    HDF5 --> Label[label_data.py]
    Label --> tasklabels[task_labels.json]
    tasklabels --> TFRecord[to_tfrecord.py]
    TFRecord --> TFDS[TFRecord/TFDS]
```

1. **Collect**: `collect_trajectory_franka.py` / `collect_trajectory_bimanual_franka.py` → HDF5 files
2. **Label**: `label_data.py` → `task_labels.json` (success/failure, task_name)
3. **Convert**: `to_tfrecord.py` → TFDS/RLDS, DROID-compatible schema

Output: `role_ros2-train.tfrecord-*`, `dataset_info.json`, loadable via `tfds.builder_from_directory()`.

## Policy Deployment

Running learned policies on a separate GPU server from the real-time robot controller is a common setup in robot learning. role-ros2 integrates with **[policy-websocket](https://github.com/YufengJin/policy_websocket.git)** — a lightweight WebSocket-based inference bridge with binary serialization (msgpack + NumPy) and built-in action chunking support for policies like ACT and Diffusion Policy.

For ready-to-use integrations with popular generalist policies, we provide dedicated forks:

- **[openpi](https://github.com/YufengJin/openpi.git)** — π0 policy inference server, compatible with role-ros2
- **[openvla](https://github.com/YufengJin/openvla.git)** — OpenVLA inference server, compatible with role-ros2

## Scripts & Tools

| Script | Description |
|--------|-------------|
| `bringup.py` | Docker-ROS control center (PyQt5 GUI) |
| `collect_trajectory_franka.py` | Single-arm VR trajectory collection |
| `collect_trajectory_bimanual_franka.py` | Bimanual VR trajectory collection |
| `replay_trajectory.py` | Replay HDF5 trajectories on robot |
| `calibrate_camera.py` | Charuco hand-eye calibration |
| `postprocess/label_data.py` | Label success/failure and task_name |
| `postprocess/to_tfrecord.py` | Convert .h5 → TFRecord/TFDS |
| `misc/trajectory_visualizer.py` | Visualize trajectories (Matplotlib) |
| `misc/hdf5_reader.py` | Inspect HDF5 structure |
| `misc/visualize_tfds.py` | Visualize TFDS/RLDS datasets |

> For detailed usage, arguments, examples, and trajectory structure, see [scripts/README.md](scripts/README.md).

## Docker Architecture

| Container | Purpose | ROS2 | Stack |
|-----------|----------|------|-------|
| `ros2_cu118_container` | Camera, GPU, ZED | Humble | CUDA 11.8, ZED SDK |
| `ros2_polymetis_container` | Robot control | Foxy | Polymetis, libfranka 0.14.x / 0.18.x |

- **Camera only**: `docker/ros2_cu118/`
- **Robot only**: `docker/ros2_franka_libfranka_0.14.x/` or `0.18.x/`
- **Full stack**: `docker/ros2_cu118_franka_0.14.x/` or `0.18.x/`

Both containers communicate via shared `ROS_DOMAIN_ID` and `network_mode: host`. See [docker/install.md](docker/install.md) for details.

## Package Structure

```
role-ros2/
├── assets/                     # Images and diagrams
│   └── banner.png             # Architecture overview
├── scripts/                    # Python script tools
│   ├── bringup.py             # Docker-ROS control center (GUI)
│   ├── collect_trajectory_franka.py
│   ├── collect_trajectory_bimanual_franka.py
│   ├── replay_trajectory.py
│   ├── calibrate_camera.py
│   ├── conf/                  # bringup.py configs
│   │   ├── franka.json
│   │   └── biman_franka.json
│   ├── misc/                  # hdf5_reader, trajectory_visualizer, etc.
│   └── postprocess/           # label_data, to_tfrecord
├── launch/                     # franka_robot, zed_camera, oculus_controller
├── config/                     # Robot, camera, calibration YAML
├── docker/                      # ros2_cu118, ros2_franka_*, ros2_cu118_franka_*
├── nodes/                       # ROS2 nodes (robot, gripper, aggregator)
└── role_ros2/                   # Python modules
    ├── robot/                   # FrankaRobot, BimanualFrankaRobot
    ├── robot_env.py             # Gym environment
    ├── controllers/             # VRPolicy, VRBimanPolicy
    ├── camera/                  # Multi-camera reader
    ├── trajectory_utils/        # collect_trajectory_base, trajectory_writer
    ├── calibration/             # Charuco hand-eye
    └── robot_ik/                # Inverse kinematics
```

## Topics

| Category | Topic | Type |
|----------|-------|------|
| Robot | `/joint_states`, `/robot_state` | JointState, RobotState |
| Camera | `/hand_camera/zed_node/rgb/image_rect_color` | Image |
| Oculus | `/oculus/right_controller/pose`, `/oculus/buttons` | PoseStamped, OculusButtons |

## Resources

- **[docker/install.md](docker/install.md)** — Docker installation, build, run, X11, troubleshooting
- **[scripts/README.md](scripts/README.md)** — Script usage, arguments, examples, trajectory structure
- [Franka ROS2](https://github.com/souljaboy764/franka_ros2)
- [ZED ROS2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper)

## Related Projects

role-ros2's data format and toolchain are inspired by and compatible with the following projects:

- **[DROID](https://github.com/droid-dataset/droid)** — Distributed Robot Interaction Dataset; in-the-wild robot manipulation data and teleoperated collection platform
- **[LeRobot](https://github.com/huggingface/lerobot)** — Hugging Face's real-world robotics library (PyTorch): models, datasets, and policies for robot learning

Thanks to the DROID and LeRobot communities for their open-source contributions to robot learning.

## Contributing

Contributions are welcome. Please ensure:

1. Code follows ROS2 and Python best practices (PEP 8)
2. Documentation and comments are added
3. Related README and docs are updated

## Acknowledgements

This project is developed at [PEARL Lab](https://pearl-lab.com/), TU Darmstadt.

We thank the following contributors for their valuable contributions to this project:

- 🤝 [Xiaogang Jia](https://xiaogangjia.github.io/Personal_Website/)

## License

Apache-2.0