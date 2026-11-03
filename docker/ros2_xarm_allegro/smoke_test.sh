#!/bin/bash
# Hardware-free smoke test for the ros2_xarm_allegro container.
set -e
cd /  # python3 -c imports from the cwd; the repo's role_ros2/ source dir would shadow the install

if [ -z "${ROS_DISTRO:-}" ] && [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
if [ -f /opt/ext_ws/install/setup.bash ]; then
    source /opt/ext_ws/install/setup.bash
fi
if [ -f /app/ros2_ws/install/setup.bash ]; then
    source /app/ros2_ws/install/setup.bash
fi

echo "[1/7] ROS2 CLI + ros2_control_node ..."
ros2 --help >/dev/null
ros2 pkg executables controller_manager | grep -q ros2_control_node
echo "      ok"

echo "[2/7] xarm-python-sdk importable ..."
python3 -c "from xarm.wrapper import XArmAPI"
echo "      ok"

echo "[3/7] underlay packages (xarm_ros2 + allegro_hand_ros2) ..."
for p in xarm_description allegro_hand_bringup allegro_hand_control_msgs \
         allegro_hand_v4_description allegro_hand_v4_hardware \
         allegro_hand_position_effort_controller; do
    ros2 pkg prefix "$p" >/dev/null
done
echo "      ok"

echo "[4/7] ros2_control controller packages ..."
for p in joint_state_broadcaster forward_command_controller imu_sensor_broadcaster \
         diagnostic_aggregator; do
    ros2 pkg prefix "$p" >/dev/null
done
echo "      ok"

echo "[5/7] role_ros2 messages built and importable ..."
ros2 interface show role_ros2/msg/HandState >/dev/null
python3 -c "import role_ros2.msg, role_ros2.srv"
echo "      ok"

echo "[6/7] combined UF850 + Allegro URDFs installed ..."
share="$(ros2 pkg prefix role_ros2)/share/role_ros2/robot_ik/xarm"
for f in uf850_allegro_left.urdf uf850_allegro_right.urdf uf850_allegro_bimanual.urdf; do
    test -f "$share/$f"
done
echo "      ok"

echo "[7/7] allegro_hand_interface_node imports cleanly ..."
python3 -c "
import importlib.util
path = '/app/ros2_ws/src/role-ros2/nodes/allegro_hand_interface_node.py'
spec = importlib.util.spec_from_file_location('allegro_hand_interface_node', path)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
assert hasattr(m, 'AllegroHandInterfaceNode')
print('      ok')
"

echo "All checks passed."
