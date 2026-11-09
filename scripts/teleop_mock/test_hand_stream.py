import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from hand_stream import (HAND0, HANDS, QPOS_CLOSED, QPOS_LIMITS, QPOS_OPEN, Dropout,
                         FallbackConfig, HandStream, MotionConfig, parse_dropouts, random_dropouts)

RATE = 60.0


def run(dropouts, duration=10.0, seed=0, motion=None):
    return list(HandStream(RATE, motion or MotionConfig(), dropouts, seed=seed).frames(duration))


def rot_angle(a, b):
    return R.from_matrix(a[:3, :3].T @ b[:3, :3]).magnitude()


def test_open_closed_within_limits():
    for q in (QPOS_OPEN, QPOS_CLOSED):
        assert np.all(q >= QPOS_LIMITS[:, 0]) and np.all(q <= QPOS_LIMITS[:, 1])


def test_hand0_rotations_proper():
    for pose in HAND0.values():
        assert np.isclose(np.linalg.det(pose[:3, :3]), 1.0)


def test_starts_at_hand0_without_dropouts():
    frames = run([], duration=2.0)
    for hand in HANDS:
        np.testing.assert_allclose(frames[0].hands[hand].pose, HAND0[hand], atol=1e-12)
    assert all(v for f in frames for v, _ in f.labels.values())


def test_valid_frames_follow_true_pose_and_limits():
    frames = run(parse_dropouts("right:fallback@2+1"), duration=6.0)
    for f in frames:
        for hand, s in f.hands.items():
            assert s.qpos.dtype == np.float32 and s.qpos.shape == (16,)
            assert np.all(s.qpos >= QPOS_LIMITS[:, 0] - 1e-6)
            assert np.all(s.qpos <= QPOS_LIMITS[:, 1] + 1e-6)
            if s.valid:
                np.testing.assert_allclose(s.pose, s.true_pose)


def test_max_speed_figure8():
    motion = MotionConfig()
    frames = run([], duration=motion.period, motion=motion)
    pos = np.array([f.hands["right"].pose[:3, 3] for f in frames])
    speed = np.linalg.norm(np.diff(pos, axis=0), axis=1) * RATE
    bound = np.sqrt(2) * motion.amplitude * 2 * np.pi / motion.period
    assert speed.max() <= bound * 1.01


def test_labels_match_events():
    events = parse_dropouts("right:frozen@1+0.5,left:origin@2+0.5,both:gap@3+0.25")
    for f in run(events, duration=4.0):
        for hand in HANDS:
            ev = next((e for e in events if e.affects(hand) and e.active(f.t)), None)
            valid, kind = f.labels[hand]
            assert valid == (ev is None)
            assert kind == (ev.kind if ev else None)


def test_frozen_repeats_last_frame():
    frames = run(parse_dropouts("right:frozen@1+0.5"), duration=2.0)
    frozen = [f for f in frames if f.labels["right"][1] == "frozen"]
    before = frames[frozen[0].index - 1].hands["right"]
    for f in frozen:
        np.testing.assert_array_equal(f.hands["right"].pose, before.pose)
        np.testing.assert_array_equal(f.hands["right"].qpos, before.qpos)


def test_origin_keeps_rotation_zero_translation():
    frames = run(parse_dropouts("left:origin@1+0.5"), duration=2.0)
    origin = [f for f in frames if f.labels["left"][1] == "origin"]
    before = frames[origin[0].index - 1].hands["left"]
    for f in origin:
        np.testing.assert_array_equal(f.hands["left"].pose[:3, 3], 0.0)
        np.testing.assert_array_equal(f.hands["left"].pose[:3, :3], before.pose[:3, :3])


def test_fallback_jumps_then_jitters():
    cfg = FallbackConfig()
    frames = run(parse_dropouts("right:fallback@1+1"), duration=3.0, seed=3)
    fb = [f for f in frames if f.labels["right"][1] == "fallback"]
    start = frames[fb[0].index - 1].hands["right"].pose
    jump = np.linalg.norm(fb[0].hands["right"].pose[:3, 3] - start[:3, 3])
    assert cfg.offset_range[0] - 0.02 <= jump <= cfg.offset_range[1] + 0.02
    poses = np.array([f.hands["right"].pose[:3, 3] for f in fb])
    assert np.ptp(poses, axis=0).max() < 0.03  # position stays roughly put
    angles = [rot_angle(a.hands["right"].pose, b.hands["right"].pose) for a, b in zip(fb, fb[1:])]
    assert 0.0 < np.mean(angles) < np.deg2rad(10)  # orientation keeps changing slightly
    fingers = {f.hands["right"].qpos.tobytes() for f in fb}
    assert len(fingers) == 1
    after = next(f for f in frames if f.t >= 2.0)
    assert after.labels["right"] == (True, None)


def test_gap_sends_nothing():
    frames = run(parse_dropouts("both:gap@1+0.2"), duration=2.0)
    gap = [f for f in frames if f.labels["left"][1] == "gap"]
    assert gap and all(f.hands is None for f in gap)
    assert all(f.labels["right"] == (False, "gap") for f in gap)


def test_deterministic_for_seed():
    events = parse_dropouts("right:fallback@1+1")
    a = run(events, duration=3.0, seed=7)
    b = run(events, duration=3.0, seed=7)
    for fa, fb in zip(a, b):
        np.testing.assert_array_equal(fa.hands["right"].pose, fb.hands["right"].pose)


def test_random_dropouts_no_overlap_and_start_after():
    events = random_dropouts(600.0, np.random.default_rng(0), start_after=3.0)
    assert events and all(e.start >= 3.0 for e in events)
    for hand in HANDS:
        mine = sorted((e for e in events if e.affects(hand)), key=lambda e: e.start)
        same_stream = [e for e in mine if e.hand == hand]
        for a, b in zip(same_stream, same_stream[1:]):
            assert a.start + a.duration <= b.start


def test_parse_rejects_unknown_kind():
    with pytest.raises(ValueError):
        parse_dropouts("right:teleport@1+1")
    with pytest.raises(ValueError):
        Dropout("middle", "gap", 0, 1)


def test_grip_sequence_moves_one_finger_at_a_time():
    from hand_stream import FINGERS, active_finger, true_qpos
    motion = MotionConfig(grip="sequence")
    for k, finger in enumerate(FINGERS):
        t = (k + 0.5) * motion.grip_period  # mid-period: finger fully closed
        assert active_finger(t, motion) == finger
        q = true_qpos(t, motion)
        moved = [i // 4 for i in np.nonzero(np.abs(q - QPOS_OPEN) > 1e-6)[0]]
        assert set(moved) == {k}
        np.testing.assert_allclose(q[4 * k:4 * k + 4], QPOS_CLOSED[4 * k:4 * k + 4], atol=1e-6)


def test_left_hand_uses_tracker_finger_order():
    from hand_stream import true_qpos
    motion = MotionConfig(grip="sequence")
    t = 0.5 * motion.grip_period  # index closed
    right, left = true_qpos(t, motion, "right"), true_qpos(t, motion, "left")
    np.testing.assert_allclose(left[8:12], right[0:4])  # left row 2 = index
    np.testing.assert_allclose(left[0:4], QPOS_OPEN[0:4])


def test_skeleton_palm_centre_matches_pose():
    from hand_stream import skeleton
    for hand in HANDS:
        pts = skeleton(HAND0[hand], hand)
        np.testing.assert_allclose((pts[0] + pts[5] + pts[17]) / 3, HAND0[hand][:3, 3], atol=1e-9)
