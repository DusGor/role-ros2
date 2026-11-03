# UF850 + Allegro V4 robot models (xacro sources)

Sources of the combined UF850 + Allegro V4 models — single arm (left/right) and
bimanual — including the lab wrist adapter and camera mount.

## When you need this
Only to **change** a model. The generated flat URDFs in `../` are committed and
installed by CMake; building and running role_ros2 does not need this folder.

## Regenerate
Requirements: [`xarm_ros2`](https://github.com/xArm-Developer/xarm_ros2) (branch `humble`,
provides `xarm_description`) and [`allegro_hand_ros2`](https://github.com/DusGor/allegro_hand_ros2_v4)
(branch `hand_description_fix`, provides `allegro_hand_v4_description`) in the workspace — see the main README section
"UF850 + Allegro Hand" for clone and build instructions.

```bash
    cd /app/ros2_ws
    colcon build --packages-select xarm_description allegro_hand_v4_description
    source install/setup.bash
    src/role-ros2/role_ros2/robot_ik/xarm/xacro/generate_urdfs.sh          # regenerate
    src/role-ros2/role_ros2/robot_ik/xarm/xacro/generate_urdfs.sh --check  # verify up to date
```

Workflow: edit xacro → `./generate_urdfs.sh` → check in RViz
(`ros2 launch role_ros2 xarm_allegro_robot.launch.py use_mock:=true hand_use_mock:=true`)
→ commit the `.xacro` **and** the regenerated `.urdf` together.

## Files
| File | Purpose |
|---|---|
| `uf850_allegro_{left,right,bimanual}.urdf.xacro` | Top-level models |
| `allegro_hand_arm_attachment.xacro` | Picks hand chirality, mounts the hand on the flange |
| `lab_wrist_adapter.xacro` | Lab bracket + camera holder (left/right offsets are mirrored) |

**Right thumb tip:** Wonik's `allegro_hand_description_right.xacro` puts the thumb tip at 0.0267
(their left hand and the thumb-tip collision box use 0.0423). The URDFs are generated against the
fork [DusGor/allegro_hand_ros2_v4](https://github.com/DusGor/allegro_hand_ros2_v4), branch
`hand_description_fix`, which corrects this.

**The hand appears to float above the adapter plate.** Wonik's palm mesh lacks the back cover
that the lab's hands have. Visual only; the kinematics are correct.

Important xacro args: `attach_xyz_1/2` (arm base positions, bimanual),
`add_camera_mount`, `connector_xyz/rpy`.

## Joint names
- Single arm: `joint1`–`joint6`, `hand_joint00`–`hand_joint33`
- Bimanual: `left_/right_joint1`–`6`, `left_/right_hand_joint00`–`33`
- The Allegro driver publishes `ah_joint…`; `allegro_hand_interface_node` renames them.

## Runtime requirements
The URDFs reference meshes as `package://xarm_description/…` and
`package://allegro_hand_v4_description/…`, so RViz needs both packages at runtime.
`colcon build` and `robot_state_publisher` do not.