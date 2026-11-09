"""
Synthetic stand-in for the output of dex_bimanual_telep's hand tracker, with labelled dropouts.

Message shapes (https://github.com/YufengJin/dex_bimanual_telep, scripts/hand_tracker_meta_quest_v2.py):
  human_wrist_pose   {"left": 4x4, "right": 4x4}  palm pose from compute_palm_pose_quest(), Quest
                     tracking space after convert_left_to_right (z -> -z), metres, y up
  allegro_hand_qpos  {"left": float32[16], "right": float32[16]}  right: index, middle, ring, thumb;
                     left: ring, middle, index, thumb (dex-retargeting allegro_hand_left.urdf has
                     finger 0 on the ring side; BiXarmDexModel swaps rows 0 and 2 back)
  match_control      {"start": 1, "ts": float}  sent 5x, 50 ms apart, on the OK gesture
send_msg() adds "timestamp".

Dropout kinds. The Quest app (https://github.com/aadhithya14/Open-Teach,
VR/Bimanual-Robot-Unity/Assets/Scripts/Gesture Detection/GestureDetector.cs) streams OVRSkeleton
bone positions every frame without checking the tracking state:
  fallback  pose jumps to a nearby random pose and stays there, orientation jittering
            (observed by the user)
  origin    all-zero keypoints: compute_palm_pose_quest() keeps the previous rotation,
            translation 0 (a code path, not observed from the app)
  frozen    the same frame repeated
  gap       no message (the tracker sends both hands in one message, so it hits both)
Fingers hold their last valid value during a dropout.
"""

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation as R

HANDS = ("left", "right")
KINDS = ("fallback", "origin", "frozen", "gap")

# Classic Allegro order (index, middle, ring, thumb); limits from
# allegro_hand_v4_description/urdf/allegro_hand_description_right.xacro.
FINGER_LIMITS = np.array([[-0.47, 0.47], [-0.196, 1.61], [-0.174, 1.709], [-0.227, 1.618]])
THUMB_LIMITS = np.array([[0.263, 1.396], [-0.105, 1.163], [-0.189, 1.644], [-0.162, 1.719]])
QPOS_LIMITS = np.vstack([FINGER_LIMITS] * 3 + [THUMB_LIMITS])
FINGERS = ("index", "middle", "ring", "thumb")
QPOS_OPEN = np.array([0.0, 0.0, 0.0, 0.0] * 3 + [0.5, 0.0, 0.0, 0.0], dtype=np.float32)
QPOS_CLOSED = np.array([0.0, 0.3, 0.5, 0.7] * 3 + [0.7, 0.3, 0.5, 0.7], dtype=np.float32)

# Palm frame as in compute_palm_pose_quest(): x towards the index MCP, z palm normal.
# Quest frame: x right, y up, -z forward. Hands at chest height, palms down.
_R_RIGHT = np.column_stack([[0, 0, -1], [1, 0, 0], [0, -1, 0]])
_R_LEFT = _R_RIGHT.copy()  # palm down: x forward, z down for both hands
HAND0 = {}
for _hand, _rot, _x in (("right", _R_RIGHT, 0.2), ("left", _R_LEFT, -0.2)):
    HAND0[_hand] = np.eye(4)
    HAND0[_hand][:3, :3] = _rot
    HAND0[_hand][:3, 3] = [_x, 1.0, -0.4]


@dataclass
class Dropout:
    hand: str  # "left", "right" or "both"
    kind: str
    start: float  # s after streaming starts
    duration: float

    def __post_init__(self):
        if self.kind not in KINDS:
            raise ValueError(f"unknown dropout kind {self.kind!r}, expected one of {KINDS}")
        if self.hand not in HANDS + ("both",):
            raise ValueError(f"unknown hand {self.hand!r}")
        if self.kind == "gap":
            self.hand = "both"

    def active(self, t: float) -> bool:
        return self.start <= t < self.start + self.duration

    def affects(self, hand: str) -> bool:
        return self.hand in (hand, "both")


@dataclass
class MotionConfig:
    kind: str = "figure8"  # "figure8" or "hold"
    amplitude: float = 0.10  # m, along palm x; palm y gets half
    period: float = 8.0  # s
    rot_amplitude_deg: float = 20.0  # about the palm normal
    grip_period: float = 4.0  # s, open -> closed -> open
    grip: str = "all"  # "all" fingers together, or "sequence": one finger per grip_period


@dataclass
class FallbackConfig:
    offset_range: tuple = (0.15, 0.5)  # m from the true pose at dropout start
    rot_offset_max_deg: float = 30.0
    rot_jitter_deg: float = 1.5  # per frame, std
    pos_jitter: float = 0.002  # m per frame, std


@dataclass
class HandSample:
    pose: np.ndarray  # 4x4, as sent
    qpos: np.ndarray  # float32[16], as sent
    true_pose: np.ndarray  # undisturbed trajectory
    valid: bool
    event: str | None


@dataclass
class Frame:
    index: int
    t: float
    hands: dict | None  # None: nothing sent (gap)
    labels: dict = field(default_factory=dict)  # hand -> (valid, event)


def true_pose(hand: str, t: float, motion: MotionConfig) -> np.ndarray:
    local = np.eye(4)
    if motion.kind == "figure8":
        w = 2 * np.pi / motion.period
        phase = 0.0 if hand == "right" else np.pi
        local[:3, 3] = [
            motion.amplitude * np.sin(w * t + phase) - motion.amplitude * np.sin(phase),
            0.5 * motion.amplitude * np.sin(2 * (w * t + phase)) - 0.5 * motion.amplitude * np.sin(2 * phase),
            0.0,
        ]
        angle = np.deg2rad(motion.rot_amplitude_deg) * np.sin(w * t)
        local[:3, :3] = R.from_rotvec([0.0, 0.0, angle]).as_matrix()
    elif motion.kind != "hold":
        raise ValueError(f"unknown motion {motion.kind!r}")
    return HAND0[hand] @ local


def skeleton(pose: np.ndarray, hand: str) -> np.ndarray:
    """21 MediaPipe landmarks of a flat hand at a compute_palm_pose_quest() pose (palm centre =
    mean of wrist, index MCP, pinky MCP), for the hand_keypoints topic."""
    sign = 1.0 if hand == "right" else -1.0
    f = np.array([np.cos(np.deg2rad(12.2)), sign * np.sin(np.deg2rad(12.2)), 0.0])
    s = np.array([-f[1], f[0], 0.0]) * sign  # towards the pinky
    mcp = {5: 0.03 * f - 0.02 * s, 9: 0.035 * f, 13: 0.032 * f + 0.01 * s, 17: 0.03 * f + 0.02 * s}
    pts = np.zeros((21, 3))
    pts[0] = -0.06 * f
    thumb = (f - s) / np.linalg.norm(f - s)
    for k in range(1, 5):
        pts[k] = pts[0] + 0.02 * f - 0.025 * s + 0.03 * (k - 1) * thumb
    for base, p in mcp.items():
        for k in range(4):
            pts[base + k] = p + 0.03 * k * f
    return pts @ pose[:3, :3].T + pose[:3, 3]


def active_finger(t: float, motion: MotionConfig) -> str | None:
    if motion.kind == "hold" or motion.grip != "sequence":
        return None
    return FINGERS[int(t // motion.grip_period) % len(FINGERS)]


def true_qpos(t: float, motion: MotionConfig, hand: str = "right") -> np.ndarray:
    """Finger-major qpos in the tracker's order for that hand (see module docstring)."""
    q = _true_qpos_classic(t, motion)
    if hand == "left":
        q = q.reshape(4, 4)[[2, 1, 0, 3]].reshape(16)
    return q


def _true_qpos_classic(t: float, motion: MotionConfig) -> np.ndarray:
    s = 0.5 * (1.0 - np.cos(2 * np.pi * t / motion.grip_period)) if motion.kind != "hold" else 0.0
    closed = QPOS_OPEN + s * (QPOS_CLOSED - QPOS_OPEN)
    finger = active_finger(t, motion)
    if finger is None:
        return closed.astype(np.float32)
    q = QPOS_OPEN.copy()
    k = FINGERS.index(finger)
    q[4 * k:4 * k + 4] = closed[4 * k:4 * k + 4]
    return q.astype(np.float32)


def random_dropouts(
    duration: float,
    rng: np.random.Generator,
    mean_interval: float = 8.0,
    duration_range: tuple = (0.3, 2.0),
    start_after: float = 3.0,
    kinds: tuple = KINDS,
) -> list:
    """Poisson-spaced dropouts per hand; gaps are drawn once for both hands. No overlaps."""
    events = []
    pose_kinds = [k for k in kinds if k != "gap"]
    streams = [(hand, pose_kinds) for hand in HANDS if pose_kinds]
    if "gap" in kinds:
        streams.append(("both", ["gap"]))
    for hand, hand_kinds in streams:
        interval = mean_interval * (2.0 if hand == "both" else 1.0)
        t = start_after + rng.exponential(interval)
        while t < duration:
            d = float(rng.uniform(*duration_range))
            if hand == "both":
                d = min(d, 0.5)
            events.append(Dropout(hand, str(rng.choice(hand_kinds)), float(t), d))
            t += d + rng.exponential(interval)
    return sorted(events, key=lambda e: e.start)


def parse_dropouts(spec: str) -> list:
    """'right:fallback@5+1.5,both:gap@12+0.4' -> [Dropout, ...] (start@s, +duration s)."""
    events = []
    for item in filter(None, (s.strip() for s in spec.split(","))):
        hand, rest = item.split(":")
        kind, timing = rest.split("@")
        start, dur = timing.split("+")
        events.append(Dropout(hand, kind, float(start), float(dur)))
    return sorted(events, key=lambda e: e.start)


class HandStream:
    """Deterministic frame sequence for a given seed; iterate with frames()."""

    def __init__(self, rate: float, motion: MotionConfig, dropouts: list,
                 fallback: FallbackConfig | None = None, seed: int = 0):
        self.rate = rate
        self.motion = motion
        self.dropouts = sorted(dropouts, key=lambda e: e.start)
        self.fallback = fallback or FallbackConfig()
        self._rng = np.random.default_rng(seed)

    def _event(self, hand: str, t: float):
        return next((e for e in self.dropouts if e.affects(hand) and e.active(t)), None)

    def _fallback_start(self, current: np.ndarray) -> np.ndarray:
        cfg = self.fallback
        direction = self._rng.normal(size=3)
        direction /= np.linalg.norm(direction)
        offset = direction * self._rng.uniform(*cfg.offset_range)
        axis = self._rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        angle = np.deg2rad(self._rng.uniform(0.0, cfg.rot_offset_max_deg))
        pose = current.copy()
        pose[:3, 3] += offset
        pose[:3, :3] = R.from_rotvec(axis * angle).as_matrix() @ current[:3, :3]
        return pose

    def _jitter(self, pose: np.ndarray) -> np.ndarray:
        cfg = self.fallback
        out = pose.copy()
        out[:3, 3] += self._rng.normal(scale=cfg.pos_jitter, size=3)
        rotvec = self._rng.normal(scale=np.deg2rad(cfg.rot_jitter_deg), size=3)
        out[:3, :3] = R.from_rotvec(rotvec).as_matrix() @ pose[:3, :3]
        return out

    def frames(self, duration: float | None = None):
        last = {hand: None for hand in HANDS}  # last sent HandSample
        fallback_base = {}  # (hand, event start) -> fallback pose
        i = 0
        while duration is None or i / self.rate < duration:
            t = i / self.rate
            hands, labels = {}, {}
            gap = False
            for hand in HANDS:
                tp = true_pose(hand, t, self.motion)
                tq = true_qpos(t, self.motion, hand)
                event = self._event(hand, t)
                prev = last[hand]
                if event is None or prev is None:
                    sample = HandSample(tp, tq, tp, True, None)
                elif event.kind == "gap":
                    gap = True
                    sample = None
                elif event.kind == "frozen":
                    sample = HandSample(prev.pose.copy(), prev.qpos.copy(), tp, False, "frozen")
                elif event.kind == "origin":
                    pose = np.eye(4)
                    pose[:3, :3] = prev.pose[:3, :3]
                    sample = HandSample(pose, prev.qpos.copy(), tp, False, "origin")
                else:
                    key = (hand, event.start)
                    if key not in fallback_base:
                        fallback_base[key] = self._fallback_start(prev.pose)
                    sample = HandSample(self._jitter(fallback_base[key]), prev.qpos.copy(), tp,
                                        False, "fallback")
                if sample is None:
                    labels[hand] = (False, "gap")
                else:
                    hands[hand] = sample
                    labels[hand] = (sample.valid, sample.event)
            if gap:
                hands = None
            else:
                last.update(hands)
            yield Frame(i, t, hands, labels)
            i += 1
