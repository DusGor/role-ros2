"""
jump_gate.JumpGate (dex_bimanual_telep, scripts/) against the mock's labelled dropouts, with
TeleopSender's anchoring reduced to target = inv(hand0) @ hand (robot0 and axis alignment are
rigid transforms and do not change step sizes).

Set DEX_TELEOP_SCRIPTS to dex_bimanual_telep/scripts if it is not next to role-ros2.
"""

import os
import sys
from pathlib import Path

import numpy as np
import pytest

from hand_stream import HANDS, HandStream, MotionConfig, random_dropouts

DEX_SCRIPTS = Path(os.environ.get(
    "DEX_TELEOP_SCRIPTS", Path(__file__).resolve().parents[3] / "dex_bimanual_telep" / "scripts"))
if not (DEX_SCRIPTS / "jump_gate.py").exists():
    pytest.skip(f"jump_gate.py not found in {DEX_SCRIPTS}", allow_module_level=True)
sys.path.insert(0, str(DEX_SCRIPTS))
from jump_gate import JumpGate  # noqa: E402

RATE = 60.0
DURATION = 180.0
REENGAGE_AFTER = 1.0
JUMP_KINDS = ("fallback", "origin")


def simulate(seed: int):
    events = random_dropouts(DURATION, np.random.default_rng(seed), mean_interval=6.0)
    stream = HandStream(RATE, MotionConfig(), events, seed=seed)
    gates = {h: JumpGate() for h in HANDS}
    hand0 = {h: None for h in HANDS}
    robot0 = {h: np.eye(4) for h in HANDS}
    target = {h: np.eye(4) for h in HANDS}
    reengage_at = sorted(e.start + e.duration + REENGAGE_AFTER for e in events)
    log = []  # (t, hand, action, event, target position)
    for f in stream.frames(DURATION):
        if (reengage_at and f.t >= reengage_at[0]
                and all(v for v, _ in f.labels.values())):
            reengage_at.pop(0)
            for h in HANDS:
                gates[h].reset()
                hand0[h] = None
        if f.hands is None:
            continue
        for h in HANDS:
            s = f.hands[h]
            action = gates[h].update(s.pose, f.t)
            if action != JumpGate.HOLD:
                if hand0[h] is None:
                    hand0[h], robot0[h] = s.pose, target[h]
                target[h] = robot0[h] @ np.linalg.inv(hand0[h]) @ s.pose
            log.append((f.t, h, action, s.event, target[h][:3, 3].copy()))
    return events, log


@pytest.mark.parametrize("seed", range(4))
def test_target_never_jumps(seed):
    _, log = simulate(seed)
    limit = JumpGate().max_lin_speed
    for h in HANDS:
        rows = [r for r in log if r[1] == h]
        for (t0, *_, p0), (t1, *_, p1) in zip(rows, rows[1:]):
            assert np.linalg.norm(p1 - p0) / (t1 - t0) <= limit + 1e-6, (seed, h, t1)


@pytest.mark.parametrize("seed", range(4))
def test_never_applies_jumped_frames(seed):
    _, log = simulate(seed)
    applied = [r for r in log if r[2] != JumpGate.HOLD and r[3] in JUMP_KINDS]
    assert not applied, applied[:3]


def test_resumes_after_dropouts():
    events, log = simulate(0)
    for e in events:
        end = e.start + e.duration
        window = REENGAGE_AFTER + 1.0
        if end + window + 1.0 > DURATION:
            continue
        # another dropout nearby (either hand: it also delays the OK gesture) muddles the window
        if any(o is not e and o.start < end + window + 1.0 and o.start + o.duration > e.start - 0.1
               for o in events):
            continue
        for h in HANDS:
            if not e.affects(h):
                continue
            later = [r for r in log if r[1] == h and end + window <= r[0] < end + window + 1.0]
            assert later and all(r[2] == JumpGate.USE for r in later), e


def test_no_false_holds_without_dropouts():
    stream = HandStream(RATE, MotionConfig(), [], seed=0)
    gate = JumpGate()
    for f in stream.frames(60.0):
        assert gate.update(f.hands["right"].pose, f.t) == JumpGate.USE
