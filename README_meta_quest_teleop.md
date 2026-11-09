# Meta Quest teleoperation: UF850 + Allegro (single arm or bimanual)

How to run teleoperation. Headset side: [dex_bimanual_telep](https://github.com/YufengJin/dex_bimanual_telep)
(host, its conda environment). Robot side: this repository in `ros2_xarm_allegro_container`
(`docker/ros2_xarm_allegro/README.md`). Tested with mock robots and a real headset; **not yet on robot
hardware**.

```
Quest app (Open-Teach) --TCP 8087/8110/8095--> H3 tracker --> H1 broker :5555/:5556 --> H2 sender
     H2 sender: mapping, jump gate, IK, collision check --> teleop_joint_state --> C3 bridge --> C1 robot
     C3 bridge: robot_joint_state (measured joints) --> broker --> H2 sender
```

## Terminals

| | Where | What |
|---|---|---|
| H1 | host, dex env | ZMQ broker |
| H2 | host, dex env | teleop sender (mapping, IK, collision check) |
| H3 | host, dex env | hand tracker (Quest stream -> wrist poses, retargeted fingers) |
| C1 | container | robot (mock or real) |
| C2 | container | RViz (optional) |
| C3 | container | bridge (teleop -> robot commands) |

Host terminals: `cd <dex_bimanual_telep> && conda activate <dex env>`.
Container terminals:
```bash
docker compose -f docker/ros2_xarm_allegro/docker-compose.yaml up -d     # once, from the role-ros2 root
docker exec -it ros2_xarm_allegro_container bash
export ROS_DOMAIN_ID=<id>       # same id in every container terminal; pick one no one else uses
```
Start order: H1 -> H2 -> H3 -> C1 -> C2 -> C3, then the OK gesture.

## Host (dex_bimanual_telep)

**H1 broker**
```bash
python scripts/broker.py
```
**H2 sender**
```bash
python scripts/teleop_send_msg_real_robot.py --mapping absolute --mirror
```
- `--mapping absolute`: the first OK gesture calibrates (both hand positions -> robot start palms); afterwards
  a hand pose always maps to the same robot palm pose.
- `--mirror`: operator faces the robot; the left hand drives the robot's right arm/hand. Leave it out when
  standing behind the robot.
- `--scale` (1.0): robot metres per hand metre.

All other options (speed limits, jump gate, app gap) have defaults for daily use; see `--help`.

**H3 tracker**
```bash
python scripts/hand_tracker_meta_quest_v2.py host=<this PC's hotspot address>      # e.g. host=10.42.0.1
```

## Container (role-ros2)

**C1 robot**
```bash
# single arm + hand: one side of the bimanual setup (setup:=right or setup:=left)
ros2 launch role_ros2 xarm_allegro_robot.launch.py setup:=right use_mock:=true hand_use_mock:=true
# bimanual, both hands
ros2 launch role_ros2 bimanual_xarm_robot.launch.py use_mock:=true hand_use_mock:=true
```
Real hardware: leave out `use_mock`/`hand_use_mock`; arm IPs in `config/xarm_allegro_robot_config.yaml` /
`config/bimanual_xarm_robot_config.yaml` (`robot_ip`), hands on CAN (bimanual: right `can0`, left `can1`,
from allegro_hand_bringup `config/v4/dual_hand`).

**C2 RViz**
```bash
rviz2 -d /app/ros2_ws/src/role-ros2/config/rviz/xarm.rviz          # single arm
rviz2 -d /app/ros2_ws/src/role-ros2/config/rviz/biman_xarm.rviz    # bimanual
```
Shows the robot and the hand skeletons (`/teleop/hand_skeleton`; right hand green, left red, each at the arm
it drives).

**C3 bridge** (`setup` must match C1)
```bash
ros2 launch role_ros2 meta_quest_teleop.launch.py setup:=right       # single arm, right side
ros2 launch role_ros2 meta_quest_teleop.launch.py setup:=left        # single arm, left side
ros2 launch role_ros2 meta_quest_teleop.launch.py setup:=bimanual    # both sides
```
Nothing is commanded before an OK gesture; each OK gesture moves the robot slowly from where it is to the
target, then it streams with `max_velocity_arm/hand` (1.0 / 3.0 rad/s); 1 s without messages -> hold until the
next OK gesture. Limits in `config/meta_quest_teleop_config.yaml`.

## Operating

1. **Calibrate and engage:** hands in front of you, comfortably apart; OK gesture with both hands (thumb and
   index tip together, other fingers straight, hold ~1 s). H2 prints `engage: start from measured ...` and
   `calibrated`; the robot moves slowly from where it is to your hands, then follows. Later OK gestures
   resume without recalibrating.
2. **Stop:** Ctrl+C in H2 (or H3); the bridge holds the robot after 1 s without commands.
3. **Recalibrate** (moved chair, or the Quest re-set its origin while the app kept running): restart H2, then
   the OK gesture calibrates again.
4. **Hand lost** (jumps): the arm holds; OK gesture to continue.
5. **Quest app closed:** after 5 s the calibration is dropped and teleop disengages; restart the app, OK gesture.
6. **Collisions:** every step is speed limited and collision-checked (both arms, hands, table; 1 cm margin
   between left and right side). A colliding arm/hand holds and continues slowly.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| H3: `Cannot assign requested address` | `host=` must be this PC's hotspot address (e.g. 10.42.0.1), not the headset's |
| H3 window frozen | Quest app not running / not sending (headset asleep: proximity sensor) |
| H2 only `Running teleoperation...` | no OK gesture detected yet (both hands, hold it) |
| C3: `setup must be right, left or bimanual` | add `setup:=...` |
| C3: `waiting for /<ns>/joint_states` | `setup` does not match the robot launched in C1 |
| C3: `waiting for an OK gesture` | expected after start, a stream stop or the app gap |
| H2: `engage: no robot_joint_state` | C3 not running or not reaching the broker |
| robot offset (app restart, recenter, moved chair) | after an app restart automatic (5 s without data); otherwise restart H2, OK gesture |
| bimanual RViz without fingers | C1 started without `hand_use_mock:=true` on a PC without hands |
