# ros2_xarm_allegro — ROS 2 Humble + UF850 + Allegro Hand V4

Standalone Docker stack for the UF850 + Allegro Hand setup (single arm and
bimanual). It combines what `docker/ros2_xarm` (xArm SDK) and
`docker/ros2_allegro` (Allegro ros2_control driver) provide separately, so both
drivers run in one container:

- ROS 2 Humble on Ubuntu 22.04 (Python 3.10)
- `xarm-python-sdk` for the arm(s), over TCP to the xArm controller box
- the ros2_control / ros2_controllers stack for the Allegro driver
- an underlay workspace in `/opt/ext_ws` with the external repos at pinned commits

| Repository | Commit (build arg) | Built packages |
|---|---|---|
| [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) (`humble`) | `62936f7` (`XARM_ROS2_COMMIT`) | `xarm_description` |
| [DusGor/allegro_hand_ros2_v4](https://github.com/DusGor/allegro_hand_ros2_v4) (`hand_description_fix`, fork of Wonik's `main` with the right thumb tip fix) | `aecb1e9` (`ALLEGRO_HAND_ROS2_COMMIT`) | `allegro_hand_bringup`, `allegro_hand_control_msgs`, `allegro_hand_v4_description`, `allegro_hand_v4_hardware`, `allegro_hand_position_effort_controller` (+ `allegro_hand_io`, `allegro_hand_utility`) |

role-ros2 itself is bind-mounted and built by the entrypoint on top of the underlay.

## Build and start

From the role-ros2 repository root:

```bash
docker compose -f docker/ros2_xarm_allegro/docker-compose.yaml build
docker compose -f docker/ros2_xarm_allegro/docker-compose.yaml up -d
docker exec -it ros2_xarm_allegro_container bash
/smoke_test.sh
```

The image build ends with a sanity check and fails if the xArm SDK, an underlay
package or a ros2_control controller is missing.

To test other commits of the external repos:

```bash
docker compose -f docker/ros2_xarm_allegro/docker-compose.yaml build \
  --build-arg XARM_ROS2_COMMIT=<sha> --build-arg ALLEGRO_HAND_ROS2_COMMIT=<sha>
```

## Launch

```bash
# Single UF850 + Allegro hand (mock arm + mock hand)
ros2 launch role_ros2 xarm_allegro_robot.launch.py setup:=right use_mock:=true hand_use_mock:=true
ros2 launch role_ros2 xarm_allegro_robot.launch.py setup:=left use_mock:=true hand_use_mock:=true

# Bimanual UF850 + both Allegro hands (mock arms + mock hands)
ros2 launch role_ros2 bimanual_xarm_robot.launch.py use_mock:=true hand_use_mock:=true

# RViz
rviz2 -d /app/ros2_ws/src/role-ros2/config/rviz/xarm.rviz
```

Real hardware:
- Arm: set `robot_ip` in `config/xarm_allegro_robot_config.yaml` (or `robot_ip:=...`) and in
  `config/bimanual_xarm_robot_config.yaml`. With an empty or unreachable IP the arm node
  falls back to its mock.
- Hand: CAN setup as described in Wonik's
  [getting_started_v4.md](https://github.com/Wonikrobotics-git/allegro_hand_ros2_v4/blob/main/allegro_hand_bringup/getting_started_v4.md).

## Allegro posture/grasp controllers are disabled by default

`allegro_hand_posture_controller` and `allegro_hand_grasp_controller` (scripted
postures and grasps, used by the wrapper's `reset` and `grasp_primitive` services)
need Wonik's prebuilt `allegro_hand_grasp_library`, which links against pinocchio
3.6.0. ROS Humble currently ships pinocchio 4.0.0, so they are not built here and
`xarm_allegro_robot.launch.py` only spawns them with `hand_grasp_controllers:=true`.
Joint-position control (`/{ns}/joint_position_controller/command`,
`move_to_joint_positions`), as used for teleoperation and learned policies, is
unaffected.

## Notes

- Uses `network_mode: host`. Do not run another role-ros2 container with the same
  `ROS_DOMAIN_ID` at the same time, or their nodes will see each other.
- `docker/ros2_xarm` and `docker/ros2_allegro` remain for the xArm6-only and
  Allegro-only setups.
