"""Pure logic of nodes/zmq_ros2_node.py (needs rclpy importable)."""

import importlib.util
from pathlib import Path

import numpy as np

_spec = importlib.util.spec_from_file_location(
    "zmq_ros2_node", Path(__file__).resolve().parents[2] / "nodes" / "zmq_ros2_node.py")
bridge = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(bridge)

SIM_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"] + [
    f"j{finger}{joint}" for joint in range(1, 5) for finger in ("if", "mf", "pf", "th")]


def test_hand_reorder_to_v4():
    names = [bridge.split_joint_state(np.arange(44.0))["right_hand"][i] for i in range(16)]
    assert [SIM_NAMES[int(n)] for n in names] == [
        f"j{finger}{joint}" for finger in ("th", "if", "mf", "pf") for joint in range(1, 5)]


def test_malformed_rejected():
    assert bridge.split_joint_state([0.0] * 43) is None
    assert bridge.split_joint_state([np.nan] * 44) is None


def follow(target_fn, start, vmax=0.5, amax=1.0, dt=0.005, t_end=10.0):
    f = bridge.SmoothFollower(start, vmax, amax)
    ts, qs, done_at = [], [], None
    for k in range(int(t_end / dt)):
        q, done = f.step(target_fn(k * dt), dt)
        ts.append(k * dt); qs.append(q)
        if done and done_at is None:
            done_at = k * dt
    return np.array(ts), np.array(qs), done_at


def test_follower_bounded_speed_and_acceleration():
    ts, qs, done_at = follow(lambda t: [1.0, -0.5, 0.0], [0.0, 0.0, 0.0])
    assert np.abs(qs[0]).max() < 1e-3  # starts where it is
    v = np.diff(qs, axis=0) / 0.005
    assert np.abs(v).max() <= 0.5 + 1e-9
    assert np.abs(np.diff(v, axis=0) / 0.005).max() <= 1.0 + 0.2  # arrival snap: one small step
    assert done_at is not None and done_at < 3.0
    np.testing.assert_allclose(qs[-1], [1.0, -0.5, 0.0], atol=1e-3)


def test_follower_catches_moving_target():
    ts, qs, done_at = follow(lambda t: [0.8 + 0.1 * np.sin(t)], [0.0])
    assert done_at is not None
    assert abs(qs[-1][0] - (0.8 + 0.1 * np.sin(ts[-1]))) < 0.01


def test_limit_step():
    q, limited = bridge.limit_step([0.0, 0.0], [1.0, 0.001], max_velocity=1.0, dt=0.05)
    np.testing.assert_allclose(q, [0.05, 0.001])
    assert limited
    q, limited = bridge.limit_step([0.0], [0.01], 1.0, 0.05)
    assert q == [0.01] and not limited
    q, _ = bridge.limit_step([0.0], [1.0], 1.0, 5.0)  # long gap capped at max_dt
    np.testing.assert_allclose(q, [0.1])


def test_join_is_inverse_of_split():
    q = np.random.default_rng(1).uniform(-1, 1, 44)
    value, valid = bridge.join_joint_state(bridge.split_joint_state(q))
    np.testing.assert_allclose(value, q)
    assert all(valid.values())
    value, valid = bridge.join_joint_state({"right_arm": [0.1] * 6})
    assert valid == {"right_arm": True, "right_hand": False, "left_arm": False, "left_hand": False}
