#!/usr/bin/env python3
"""
Mock of dex_bimanual_telep's hand tracker: publishes human_wrist_pose, allegro_hand_qpos and
match_control into its ZMQ broker, plus mock_ground_truth with per-frame dropout labels.

  python3 teleop_mock.py --dropouts random --seed 1 --duration 60 --csv /tmp/gt.csv
  python3 teleop_mock.py --dropouts "right:fallback@5+1.5,both:gap@12+0.4"

Wire format as upstream zmq_utils.send_msg(): multipart [topic, pickle(dict)], dict gets
"timestamp"; senders connect to the broker frontend (broker.py, :5555).
"""

import argparse
import csv
import pickle
import signal
import sys
import time

import numpy as np
import zmq

from hand_stream import (KINDS, FallbackConfig, HandStream, MotionConfig, active_finger,
                         parse_dropouts, random_dropouts, skeleton, true_pose)

GT_TOPIC = "mock_ground_truth"


def send(sock, topic: str, msg: dict) -> None:
    msg.setdefault("timestamp", time.time())
    sock.send_multipart([topic.encode(), pickle.dumps(msg)])


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--server-ip", default="localhost")
    p.add_argument("--port", type=int, default=5555, help="broker frontend")
    p.add_argument("--bind", action="store_true", help="bind instead of connecting (no broker)")
    p.add_argument("--rate", type=float, default=60.0, help="Hz")
    p.add_argument("--duration", type=float, default=0.0, help="s of streaming, 0 = until Ctrl+C")
    p.add_argument("--start-delay", type=float, default=1.0, help="s before match_control")
    p.add_argument("--motion", choices=["figure8", "hold"], default="figure8")
    p.add_argument("--amplitude", type=float, default=0.10, help="m")
    p.add_argument("--period", type=float, default=8.0, help="s")
    p.add_argument("--rot-amplitude-deg", type=float, default=20.0)
    p.add_argument("--grip-period", type=float, default=4.0, help="s")
    p.add_argument("--grip", choices=["all", "sequence"], default="all",
                   help="sequence: index, middle, ring, thumb one after another")
    p.add_argument("--dropouts", default="none",
                   help="'none', 'random', or 'hand:kind@start+duration,...' "
                        f"(hand: left/right/both, kind: {'/'.join(KINDS)})")
    p.add_argument("--dropout-interval", type=float, default=8.0, help="s, mean, random mode")
    p.add_argument("--dropout-kinds", default=",".join(KINDS), help="random mode")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--reengage-after", type=float, default=-1.0,
                   help="s after each dropout ends: send match_control (OK gesture); <0 = never")
    p.add_argument("--pause", default="",
                   help="'start+duration,...': match_control start 0 at start, start 1 after duration "
                        "(pause / OK gesture)")
    p.add_argument("--csv", help="write ground truth per frame and hand")
    return p.parse_args(argv)


def parse_pauses(spec: str) -> list:
    """'12+3,30+2' -> [(12.0, 0), (15.0, 1), (30.0, 0), (32.0, 1)] as (t, match_control start)."""
    events = []
    for item in filter(None, (x.strip() for x in spec.split(","))):
        start, duration = (float(v) for v in item.split("+"))
        events += [(start, 0), (start + duration, 1)]
    return sorted(events)


def build_dropouts(args, horizon: float) -> list:
    if args.dropouts == "none":
        return []
    if args.dropouts == "random":
        kinds = tuple(k.strip() for k in args.dropout_kinds.split(",") if k.strip())
        rng = np.random.default_rng(args.seed + 1)
        return random_dropouts(horizon, rng, mean_interval=args.dropout_interval, kinds=kinds)
    return parse_dropouts(args.dropouts)


def main(argv=None) -> int:
    args = parse_args(argv)
    motion = MotionConfig(args.motion, args.amplitude, args.period, args.rot_amplitude_deg,
                          args.grip_period, args.grip)
    horizon = args.duration if args.duration > 0 else 24 * 3600.0
    dropouts = build_dropouts(args, horizon)
    stream = HandStream(args.rate, motion, dropouts, FallbackConfig(), seed=args.seed)

    sock = zmq.Context.instance().socket(zmq.PUB)
    endpoint = f"tcp://*:{args.port}" if args.bind else f"tcp://{args.server_ip}:{args.port}"
    (sock.bind if args.bind else sock.connect)(endpoint)
    print(f"[teleop_mock] {'bind' if args.bind else 'connect'} {endpoint}")
    for e in dropouts[:20]:
        print(f"[teleop_mock] dropout {e.hand:5s} {e.kind:8s} at {e.start:7.2f} s for {e.duration:.2f} s")
    if len(dropouts) > 20:
        print(f"[teleop_mock] ... {len(dropouts) - 20} more")

    stop = False

    def on_sigint(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_sigint)

    csv_file = open(args.csv, "w", newline="") if args.csv else None
    writer = csv.writer(csv_file) if csv_file else None
    if writer:
        writer.writerow(["frame", "t", "hand", "valid", "event", "sent", "x", "y", "z",
                         "true_x", "true_y", "true_z"])

    time.sleep(args.start_delay)
    for _ in range(5):
        send(sock, "match_control", {"start": 1, "ts": time.time()})
        time.sleep(0.05)

    reengage_at = sorted(e.start + e.duration + args.reengage_after for e in dropouts) \
        if args.reengage_after >= 0 else []
    pause_events = parse_pauses(args.pause)
    t0 = time.monotonic()
    sent = 0
    prev_events, prev_finger = {}, None
    try:
        for frame in stream.frames(args.duration if args.duration > 0 else None):
            if stop:
                break
            delay = t0 + frame.t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            for hand, (_, event) in frame.labels.items():
                if event != prev_events.get(hand):
                    print(f"[teleop_mock] {frame.t:7.2f} s  {hand:5s} "
                          + (f"dropout {event}" if event else "tracked"))
                    prev_events[hand] = event
            finger = active_finger(frame.t, motion)
            if finger != prev_finger:
                print(f"[teleop_mock] {frame.t:7.2f} s  closing {finger}")
                prev_finger = finger
            # the OK gesture needs both hands tracked
            reengage = (bool(reengage_at) and frame.t >= reengage_at[0]
                        and all(valid for valid, _ in frame.labels.values()))
            if reengage:
                reengage_at.pop(0)
                send(sock, "match_control", {"start": 1, "ts": time.time()})
                print(f"[teleop_mock] {frame.t:7.2f} s  match_control (OK gesture)")
            while pause_events and frame.t >= pause_events[0][0]:
                _, start = pause_events.pop(0)
                send(sock, "match_control", {"start": start, "ts": time.time()})
                print(f"[teleop_mock] {frame.t:7.2f} s  " + ("resume" if start else "pause"))
            if frame.hands is not None:
                send(sock, "human_wrist_pose", {h: s.pose for h, s in frame.hands.items()})
                send(sock, "allegro_hand_qpos", {h: s.qpos for h, s in frame.hands.items()})
                send(sock, "hand_keypoints", {h: skeleton(s.pose, h) for h, s in frame.hands.items()})
                sent += 1
            send(sock, GT_TOPIC, {
                "frame": frame.index, "t": frame.t, "sent": frame.hands is not None,
                "match_control": reengage,
                **{h: {"valid": v, "event": ev} for h, (v, ev) in frame.labels.items()},
            })
            if writer:
                for hand, (valid, event) in frame.labels.items():
                    s = frame.hands.get(hand) if frame.hands else None
                    pos = s.pose[:3, 3] if s is not None else [""] * 3
                    tp = (s.true_pose if s is not None else true_pose(hand, frame.t, motion))[:3, 3]
                    writer.writerow([frame.index, f"{frame.t:.4f}", hand, int(valid), event or "",
                                     int(s is not None), *pos, *tp])
    finally:
        if csv_file:
            csv_file.close()
        sock.close(linger=500)
    print(f"[teleop_mock] stopped after {sent} frames")
    return 0


if __name__ == "__main__":
    sys.exit(main())
