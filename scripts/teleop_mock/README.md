# teleop_mock

Stands in for the Meta Quest + hand tracker of
[dex_bimanual_telep](https://github.com/YufengJin/dex_bimanual_telep)
(`scripts/hand_tracker_meta_quest_v2.py`): publishes `human_wrist_pose`, `allegro_hand_qpos` and
`match_control` into its ZMQ broker, with labelled dropouts. Ground truth goes to
`mock_ground_truth` (and `--csv`).

- `hand_stream.py`: frame generator, no ZMQ (used by the tests)
- `teleop_mock.py`: publisher CLI (`--help`)
- `test_hand_stream.py`: `python3 -m pytest test_hand_stream.py`
- `record_quest_session.py`: records a real headset session (`--help`, `--summary FILE`)
- `replay_quest_session.py`: replays a recorded session (`--help`, `--list FILE`)
- `test_zmq_bridge.py`: logic of `nodes/zmq_ros2_node.py` (finger order, engage follower, speed limit)

## Recording a headset session

Host (or this container, host networking), in this order:
```bash
python scripts/broker.py                                   # dex_bimanual_telep
python3 record_quest_session.py session_$(date +%F_%H%M).jsonl.gz
python scripts/hand_tracker_meta_quest_v2.py port_right=18087 port_left=18110 port_button=18095
python scripts/teleop_send_msg_real_robot.py               # optional, records teleop_joint_state
```
The recorder binds the app's ports from `quest.yaml` (8087, 8110, 8095) and forwards to the tracker
on 18087/18110/18095 at `--forward-host` (default 10.42.0.1, the `host` in your `quest.yaml`).
Press Enter in the recorder terminal to mark an event, e.g. type `left behind back` before hiding
the hand and Enter again (empty) when it is back. Suggested session: OK gesture, 1 min normal
motion, then deliberate losses (hand behind back, hands overlapping, out of view, fast motion),
each marked.

## Replaying a recorded session

Stop the headset app, then start everything as for a live session and replace the headset with:
```bash
python3 replay_quest_session.py session.jsonl.gz --list                     # duration, OK gestures, marks
python3 replay_quest_session.py session.jsonl.gz                            # quest mode: raw app stream -> tracker
python3 replay_quest_session.py session.jsonl.gz --mode broker               # tracker output -> broker, no tracker
```
Nothing moves before an OK gesture in the replayed part. `quest` finds the tracker's address itself (`--host`
overrides) and runs the current tracker on the recorded hands (OK gestures included); `broker` replays the
tracker of the recording (no `hand_keypoints` in recordings before 2026-11-05 evening). Recorded pause/reset
messages (from the former console) are replayed in both modes (`--no-console` skips them); `--mode broker --engage`
sends an OK gesture first, for recordings that start with the tracker already engaged; `--start`, `--end`
(s from the start of the recording) and `--speed` select a part.

## Simulation run

Host, dex_bimanual_telep environment:
```bash
python scripts/broker.py
python scripts/teleop_send_msg_real_robot.py --mapping absolute
```
`--mapping absolute`: the first OK gesture maps both hand positions to the robot start poses (left-right
from the line between the hands, up from gravity); later OK gestures only resume. Mock: `--pause
12+3` sends a pause and, 3 s later, an OK gesture.
Jump gate (`scripts/jump_gate.py`): a wrist jump above `--max-lin-speed`/`--max-ang-speed` holds
that arm until the OK gesture (`match_control`); run the mock with `--reengage-after 1.0` so it sends
one after each dropout.
`test_jump_gate_with_mock.py` runs the gate against the mock's dropouts (finds the dex scripts next
to role-ros2, or via `DEX_TELEOP_SCRIPTS`).

`ros2_xarm_allegro_container`, one terminal each:
```bash
ros2 launch role_ros2 xarm_allegro_robot.launch.py setup:=right use_mock:=true hand_use_mock:=true
ros2 launch role_ros2 meta_quest_teleop.launch.py setup:=right
python3 src/role-ros2/scripts/teleop_mock/teleop_mock.py --dropouts random --seed 1
```
The bridge commands nothing until an OK gesture (`match_control`). Each OK gesture moves the robot
from where it is to the teleop target with bounded speed and acceleration (`ramp_max_*`, at least
`engage_min_duration`), then streams with `max_velocity_*`. A pause or reset message (`match_control`
start 0, `robot_reset`) or `stream_timeout` without messages stop commanding until the next OK gesture. Operator facing
the robot: `teleop_send_msg_real_robot.py --mapping absolute --mirror` (human left drives the robot's right side;
hand poses mirrored before IK, so IK and collision check run on the real robot). With `--mapping
absolute`, the hand skeleton is shown in RViz (`/teleop/hand_skeleton`, mock: flat hand).

Operation (calibration, Quest app restarts): [`README_meta_quest_teleop.md`](../../README_meta_quest_teleop.md),
section "Operating". Mock of an app restart: `--dropouts "both:gap@8+6"` (no data for 6 s;
the sender disengages and needs an OK gesture, `--reengage-after 2`).

## Dropout kinds

| kind | sent | source |
|---|---|---|
| `fallback` | jump to a random pose 0.15–0.5 m away, held, orientation jitters | observed on the real setup |
| `origin` | translation 0, previous rotation | all-zero keypoints through `compute_palm_pose_quest()`; not seen from the app |
| `frozen` | identical frame repeated | |
| `gap` | nothing (both hands) | tracker sends both hands in one message |

The Quest app ([Open-Teach](https://github.com/aadhithya14/Open-Teach),
`VR/Bimanual-Robot-Unity/Assets/Scripts/Gesture Detection/GestureDetector.cs`) sends bone
positions every frame without checking the tracking state, so none of these is flagged upstream.
