#!/usr/bin/env python3
"""
ROS 2 bridge between the teleop sender on the host and the robot (setup:=right | left | bimanual).

Broker (dex_bimanual_telep scripts/broker.py, :5556) -> this node:
  teleop_joint_state  44 joint targets from teleop_send_msg_real_robot.py, already in robot sides
  match_control       OK gesture (start 1) engages; start 0 / robot_reset stop commanding
  hand_skeleton       hand keypoints for RViz
This node -> ROS: /{side}_arm/ and /{side}_hand/joint_position_controller/command, /teleop/hand_skeleton.
This node -> broker (:5555): robot_joint_state, the measured joints (/{side}_*/joint_states) in the
  same order plus a valid flag per group, so the sender starts each engage from the real robot.

Order of the 44 values: right [joint1..6, jif1, jmf1, jpf1, jth1, jif2, ..., jth4] + left [same]; the hand
joints are joint-major (index, middle, ring, thumb per joint). The Wonik V4 hands are finger-major,
thumb first (hand_joint00..33), hence HAND_TO_V4.

Commanding: nothing before an OK gesture. After each one, every group moves from its measured position
to the live target (SmoothFollower, ramp_max_* limits, at least engage_min_duration), then streams with
at most max_velocity_* per joint. stream_timeout without messages -> hold until the next OK gesture.
"""

import pickle
import time

import numpy as np
import rclpy
import zmq
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from role_ros2.msg import JointPositionCommand


TOPIC = "teleop_joint_state"
N_JOINTS = 44
GROUP_SLICES = {
    "right_arm": slice(0, 6),
    "right_hand": slice(6, 22),
    "left_arm": slice(22, 28),
    "left_hand": slice(28, 44),
}
HAND_TO_V4 = [4 * joint + finger for finger in (3, 0, 1, 2) for joint in range(4)]
OTHER_SIDE = {"right_arm": "left_arm", "left_arm": "right_arm",
              "right_hand": "left_hand", "left_hand": "right_hand"}
CONTROL_TOPICS = ("match_control", "robot_reset")
SKELETON_TOPIC = "hand_skeleton"
ROBOT_STATE_TOPIC = "robot_joint_state"
ENGAGE_DEBOUNCE = 0.5  # s; the tracker sends match_control 5x within 0.25 s
HAND_EDGES = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10),
              (10, 11), (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (0, 17),
              (17, 18), (18, 19), (19, 20)]
SKELETON_COLORS = {"right": (0.1, 0.8, 0.1), "left": (0.9, 0.1, 0.1)}


def join_joint_state(groups: dict):
    """Inverse of split_joint_state for measured robot groups -> (44 values, {group: valid}).
    Groups missing or with the wrong length are zero and invalid."""
    q = np.zeros(N_JOINTS)
    valid = {group: False for group in GROUP_SLICES}
    for robot_group, values in groups.items():
        s = GROUP_SLICES[robot_group]
        values = np.asarray(values, dtype=np.float64)
        if values.shape != (s.stop - s.start,) or not np.isfinite(values).all():
            continue
        group = robot_group
        if group.endswith("_hand"):
            sim = np.zeros(16)
            sim[HAND_TO_V4] = values
            values = sim
        q[GROUP_SLICES[group]] = values
        valid[group] = True
    return q.tolist(), valid


def split_joint_state(value) -> dict | None:
    """Split a teleop_joint_state vector into per-group position lists; None if malformed."""
    q = np.asarray(value, dtype=np.float64)
    if q.shape != (N_JOINTS,) or not np.isfinite(q).all():
        return None
    groups = {group: q[s] for group, s in GROUP_SLICES.items()}
    for hand in ("right_hand", "left_hand"):
        groups[hand] = groups[hand][HAND_TO_V4]
    return {group: values.tolist() for group, values in groups.items()}


def limit_step(last, target, max_velocity: float, dt: float, max_dt: float = 0.1):
    """Move from last towards target by at most max_velocity * dt per joint; returns (positions, limited)."""
    last = np.asarray(last, dtype=np.float64)
    delta = np.asarray(target, dtype=np.float64) - last
    step = max_velocity * min(max(dt, 0.0), max_dt)
    limited = bool(np.any(np.abs(delta) > step))
    return (last + np.clip(delta, -step, step)).tolist(), limited


class SmoothFollower:
    """Catch-up phase after an OK gesture: moves one group (arm or hand) smoothly from its measured
    position to the teleop target, per joint with bounded speed and acceleration. The target may keep
    moving (the hand). Used only in this phase:
      - created per group on every OK gesture (_engage -> _start_follower), from the measured
        joint state at that moment; usually the robot is at rest then (start, after a pause, a
        jump-gate hold or a stream stop). A group that was still moving (e.g. the other arm during a
        one-sided jump-gate hold) also restarts here, from its measured position with speed 0.
      - deleted once on target and engage_min_duration has passed; the group then streams with
        limit_step from its last command. A stop (_disarm) deletes it too.

    State, all per joint: q = the position commanded last (starts at the measured position and is
    then advanced by step() only; the robot is not read again), v = the current speed (starts at 0).
    Every step() result is sent as the command, so q is what the robot was told, not where it is
    (the measured position lags the commands slightly while moving, so it is only used at the start)."""

    def __init__(self, start, max_velocity: float, max_acceleration: float, tolerance: float = 0.005):
        self.q = np.asarray(start, dtype=np.float64).copy()  # measured joints at the engage
        self.v = np.zeros_like(self.q)
        self.vmax, self.amax, self.tol = max_velocity, max_acceleration, tolerance

    def step(self, target, dt: float):
        """Advance q by one control tick (dt, capped at 0.1 s) towards target.
        Returns (positions to command, on_target)."""
        dt = min(max(dt, 0.0), 0.1)
        e = np.asarray(target, dtype=np.float64) - self.q
        if dt <= 0.0:
            return self.q.tolist(), False
        # e: remaining distance per joint (sign = direction)
        adt = self.amax * dt  # largest speed change in one tick (acceleration limit)
        # Braking curve: the highest speed from which the joint still stops exactly at the target
        # when it loses adt of speed every tick. From speed n*adt the ticks cover
        # dt*adt*(n + (n-1) + ... + 1) = adt*dt^2*n(n+1)/2; setting that to |e| and solving for n
        # gives n = sqrt(1/4 + 2|e|/(adt*dt)) - 1/2, speed n*adt. (The continuous sqrt(2*a*|e|)
        # would arrive with leftover speed and stop abruptly in the last tick.)
        v_brake = adt * (np.sqrt(0.25 + 2.0 * np.abs(e) / (adt * dt)) - 0.5)
        # desired speed: towards the target, capped by the speed limit and the braking curve
        v_des = np.sign(e) * np.minimum(self.vmax, v_brake)
        # acceleration limit: the speed changes by at most adt per tick (speeding up and braking)
        self.v += np.clip(v_des - self.v, -self.amax * dt, self.amax * dt)
        step = self.v * dt
        # a joint whose step would reach or pass the target lands exactly on it and stops
        arrive = np.abs(step) >= np.abs(e)
        self.q = np.where(arrive, self.q + e, self.q + step)
        self.v[arrive] = 0.0
        return self.q.tolist(), bool(np.all(np.abs(np.asarray(target) - self.q) < self.tol))


def skeleton_markers(points, frame_id: str, side: str, stamp) -> list:
    r, g, b = SKELETON_COLORS[side]
    markers = []
    for k, (kind, scale) in enumerate(((Marker.LINE_LIST, 0.006), (Marker.SPHERE_LIST, 0.014))):
        m = Marker()
        m.header.frame_id, m.header.stamp = frame_id, stamp
        m.ns, m.id, m.type, m.action = f"hand_skeleton_{side}", k, kind, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
        m.lifetime.nanosec = 500_000_000
        idx = [i for edge in HAND_EDGES for i in edge] if kind == Marker.LINE_LIST else range(len(points))
        m.points = [Point(x=float(points[i][0]), y=float(points[i][1]), z=float(points[i][2])) for i in idx]
        markers.append(m)
    return markers


class ZMQROS2Bridge(Node):
    def __init__(self) -> None:
        super().__init__("zmq_ros2_bridge", namespace="teleop")

        self.declare_parameter("server_ip", "localhost")
        self.declare_parameter("port", 5556)
        self.declare_parameter("frontend_port", 5555)  # broker input, for robot_joint_state
        self.declare_parameter("robot_state_rate", 30.0)  # Hz
        self.declare_parameter("poll_rate", 200.0)
        self.declare_parameter("arm_command_blocking", False)
        self.declare_parameter("ramp_max_velocity_arm", 0.5)  # rad/s, after an engage
        self.declare_parameter("ramp_max_velocity_hand", 1.5)
        self.declare_parameter("ramp_max_acceleration_arm", 1.0)  # rad/s^2
        self.declare_parameter("ramp_max_acceleration_hand", 5.0)
        # the sender's output keeps moving for up to ~2 s after an engage (IK converging)
        self.declare_parameter("engage_min_duration", 2.0)  # s
        self.declare_parameter("stream_timeout", 1.0)  # s without messages -> stop until engage
        self.declare_parameter("max_velocity_arm", 1.0)  # rad/s while streaming
        self.declare_parameter("max_velocity_hand", 3.0)
        # Required: which sides to command (arm + hand each). Namespaces and frames follow the robot
        # launches: {side}_arm, {side}_hand, {side}_link_base.
        self.declare_parameter("setup", "")  # right | left | bimanual
        self.declare_parameter("hand_skeleton", "both")  # both | commanded | none

        server_ip = self.get_parameter("server_ip").value
        port = self.get_parameter("port").value
        self._arm_blocking = self.get_parameter("arm_command_blocking").value
        self._limits = {kind: {
            "ramp_v": self.get_parameter(f"ramp_max_velocity_{kind}").value,
            "ramp_a": self.get_parameter(f"ramp_max_acceleration_{kind}").value,
            "v": self.get_parameter(f"max_velocity_{kind}").value} for kind in ("arm", "hand")}
        self._stream_timeout = self.get_parameter("stream_timeout").value
        self._engage_min_duration = self.get_parameter("engage_min_duration").value

        setup = self.get_parameter("setup").value.strip().lower()
        sides = {"right": ["right"], "left": ["left"], "bimanual": ["right", "left"]}.get(setup)
        if sides is None:
            message = (f"setup must be right, left or bimanual (got {setup!r}), e.g. "
                       "ros2 launch role_ros2 meta_quest_teleop.launch.py setup:=bimanual")
            self.get_logger().fatal(message)
            raise RuntimeError(message)

        self._pubs = {}
        self._measured = {}
        self._namespaces = {f"{side}_{kind}": f"{side}_{kind}" for side in sides for kind in ("arm", "hand")}
        for group in GROUP_SLICES:
            ns = self._namespaces.get(group)
            if ns:
                self._namespaces[group] = ns
                self._pubs[group] = self.create_publisher(
                    JointPositionCommand, f"/{ns}/joint_position_controller/command", 10
                )
                self.create_subscription(
                    JointState, f"/{ns}/joint_states",
                    lambda msg, g=group: self._measured.__setitem__(g, list(msg.position)), 10
                )
        self._base_frames = {f"{side}_arm": f"{side}_link_base" for side in sides}
        self._skeleton_mode = self.get_parameter("hand_skeleton").value.strip().lower()
        self._skeleton_pub = self.create_publisher(MarkerArray, "hand_skeleton", 10)

        self._ctx = zmq.Context.instance()
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.connect(f"tcp://{server_ip}:{port}")
        for topic in (TOPIC, SKELETON_TOPIC) + CONTROL_TOPICS:
            self._sub.setsockopt(zmq.SUBSCRIBE, topic.encode())
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.connect(f"tcp://{server_ip}:{self.get_parameter('frontend_port').value}")
        self.create_timer(1.0 / self.get_parameter("robot_state_rate").value, self._publish_robot_state)
        self.get_logger().info(
            f"Subscribed to '{TOPIC}' at tcp://{server_ip}:{port}; commanding "
            + ", ".join(f"/{ns}" for ns in self._namespaces.values()) + f" (setup {setup})"
            + ". Waiting for an OK gesture (match_control) before commanding."
        )

        self._armed = False
        self._last_engage = -1e9
        self._last_msg_time = None
        # Per group (arm or hand): OK gesture -> _pending -> _followers -> _streaming (+ _behind);
        # _disarm clears all four phases and the robot holds its last command.
        # group -> newest teleop target (kept across _disarm)
        self._targets = {}
        # group -> SmoothFollower: catch-up from the measured joints after an OK gesture,
        # stepped every tick until on target and engage_min_duration has passed
        self._followers = {}
        # groups waiting for a follower (needs a target and a measured joint state); not commanded
        self._pending = set()
        # groups in normal teleop: limit_step towards each new target (max_velocity_arm/hand)
        self._streaming = set()
        # streaming groups clipped by the speed limit: stepped every tick until on target
        self._behind = set()
        # group -> (time, positions) of the last command; limit_step starts here, so the
        # follower -> streaming handover is continuous
        self._last_cmd = {}
        self._last_tick = time.monotonic()
        self.create_timer(1.0 / self.get_parameter("poll_rate").value, self._poll)

    def _kind(self, group: str) -> str:
        return "arm" if group.endswith("_arm") else "hand"

    def _disarm(self, reason: str) -> None:
        if self._armed:
            self.get_logger().info(f"{reason}: holding, waiting for an OK gesture.")
        self._armed = False
        self._followers.clear()
        self._pending.clear()
        self._streaming.clear()
        self._behind.clear()

    def _engage(self, now: float) -> None:
        if now - self._last_engage < ENGAGE_DEBOUNCE:
            return
        self._last_engage = now
        self._armed = True
        self._last_msg_time = now  # the sender resumes its stream after the engage
        self._followers.clear()
        self._streaming.clear()
        self._behind.clear()
        self._pending = set(self._pubs)
        self.get_logger().info("Engaged: moving to the teleop target.")

    def _start_follower(self, group: str, now: float) -> bool:
        target = self._targets.get(group)
        if target is None:
            return False
        # always from the measured joints (from rest): the robot may not be where it was commanded
        start = self._measured.get(group)
        if start is None or len(start) != len(target):
            self.get_logger().warn(
                f"{group}: waiting for /{self._namespaces[group]}/joint_states "
                f"with {len(target)} joints (is that robot launched?), not commanding",
                throttle_duration_sec=2.0)
            return False
        lim = self._limits[self._kind(group)]
        self._followers[group] = SmoothFollower(start, lim["ramp_v"], lim["ramp_a"])
        return True

    def _poll(self) -> None:
        now = time.monotonic()
        dt, self._last_tick = now - self._last_tick, now
        new_target, skeleton = False, None
        # Drain everything queued since the last tick: the sender publishes at its own rate, several
        # topics share this socket, and reading one message per tick would let a backlog (and the
        # robot's delay) grow. Only the newest target and skeleton are kept; match_control and
        # robot_reset are handled in arrival order. zmq.Again is pyzmq's "queue empty" signal for a
        # non-blocking receive.
        while True:
            try:
                topic, packed = self._sub.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                break
            topic = topic.decode(errors="replace")
            # pickle is the upstream wire format; only connect to a trusted broker.
            try:
                msg = pickle.loads(packed)
            except Exception as e:
                self.get_logger().warn(f"Undecodable {topic} message: {e}", throttle_duration_sec=1.0)
                continue
            if topic == "match_control":
                if int(msg.get("start", 0) or 0) == 1:
                    self._engage(now)
                else:
                    self._disarm("Paused")
            elif topic == "robot_reset":
                self._disarm("robot_reset")
            elif topic == SKELETON_TOPIC:
                skeleton = msg
            elif topic == TOPIC:
                try:
                    groups = split_joint_state(msg["value"])
                except Exception as e:
                    self.get_logger().warn(f"Undecodable {TOPIC} message: {e}", throttle_duration_sec=1.0)
                    continue
                if groups is None:
                    self.get_logger().warn(
                        f"Malformed {TOPIC}: expected {N_JOINTS} finite values", throttle_duration_sec=1.0)
                    continue
                self._targets, self._last_msg_time, new_target = groups, now, True

        if skeleton is not None:
            self._publish_skeleton(skeleton)
        if self._last_msg_time is not None and now - self._last_msg_time > self._stream_timeout:
            self._disarm("Teleop stream stopped")
        if not self._armed:
            if new_target:
                self.get_logger().info(
                    f"{TOPIC} received; waiting for an OK gesture before commanding.",
                    throttle_duration_sec=5.0)
            return
        self._command(now, dt, new_target)

    def _command(self, now: float, dt: float, new_target: bool) -> None:
        stamp = self.get_clock().now().to_msg()
        for group, pub in self._pubs.items():
            if group in self._pending and self._start_follower(group, now):
                self._pending.discard(group)
            target = self._targets.get(group)
            if group in self._followers:
                positions, on_target = self._followers[group].step(target, dt)
                if on_target and now - self._last_engage >= self._engage_min_duration:
                    del self._followers[group]
                    self._streaming.add(group)
                    self.get_logger().info(f"{group}: on target, streaming")
            elif group in self._streaming and (new_target or group in self._behind):
                positions, limited = limit_step(
                    self._last_cmd[group][1], target, self._limits[self._kind(group)]["v"],
                    now - self._last_cmd[group][0])
                if limited:
                    self._behind.add(group)
                    self.get_logger().info(f"{group}: speed limit active", throttle_duration_sec=1.0)
                else:
                    self._behind.discard(group)
            else:
                continue
            self._last_cmd[group] = (now, positions)
            cmd = JointPositionCommand()
            cmd.header.stamp = stamp
            cmd.positions = positions
            cmd.blocking = self._arm_blocking if group.endswith("_arm") else False
            pub.publish(cmd)

    def _publish_robot_state(self) -> None:
        if not self._measured:
            return
        value, valid = join_joint_state(dict(self._measured))
        self._pub.send_multipart([ROBOT_STATE_TOPIC.encode(), pickle.dumps(
            {"value": value, "valid": valid, "timestamp": time.time()})])

    def _publish_skeleton(self, msg) -> None:
        if self._skeleton_mode == "none" or not self._base_frames:
            return
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        T_rl = msg.get("T_right_left")
        for side in ("right", "left"):
            points = msg.get(side)
            if points is None or np.shape(points) != (21, 3):
                continue
            points = np.asarray(points, dtype=np.float64)
            group = f"{msg.get('arm_of', {}).get(side, side)}_arm"
            if group not in self._base_frames:
                other = OTHER_SIDE[group]
                if self._skeleton_mode != "both" or other not in self._base_frames or T_rl is None:
                    continue
                T = np.asarray(T_rl, dtype=np.float64)
                if group == "right_arm":
                    T = np.linalg.inv(T)
                points = points @ T[:3, :3].T + T[:3, 3]
                group = other
            markers.markers += skeleton_markers(points, self._base_frames[group], side, stamp)
        if markers.markers:
            self._skeleton_pub.publish(markers)

    def destroy_node(self):
        self._sub.close(linger=0)
        self._pub.close(linger=0)
        super().destroy_node()


def main():
    rclpy.init()
    node = ZMQROS2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
