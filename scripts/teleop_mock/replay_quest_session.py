#!/usr/bin/env python3
"""
Replay a session recorded with record_quest_session.py, with the original timing.

  quest   the raw Quest app stream is pushed to the hand tracker's ports instead of the headset
          (tracker, sender and robot side run as usual; the recorded OK gestures are in the data)
  broker  the tracker's recorded output (human_wrist_pose, allegro_hand_qpos, hand_keypoints,
          match_control) is published to the broker frontend; no tracker needed, but it is the
          tracker of the recording, not the current one
Console commands of the recording (match_control start 0, robot_reset) are replayed in both modes,
marks are printed when they come up. Stop the real headset app first.
Nothing moves before an OK gesture: the tracker only sends after it sees one in the hand data, so the
replayed part needs one (--list shows them). Recordings that start with the tracker already engaged
(tracker output from the first second on, no OK gesture) work with --mode broker --engage.

  python3 replay_quest_session.py session.jsonl.gz --list
  python3 replay_quest_session.py session.jsonl.gz                           # finds the tracker's address
  python3 replay_quest_session.py session.jsonl.gz --mode broker --start 60 --end 120 --speed 0.5
"""

import argparse
import base64
import fcntl
import gzip
import json
import pickle
import socket
import struct
import sys
import time

import zmq

QUEST_STREAMS = ("right", "left", "button")
TRACKER_TOPICS = ("human_wrist_pose", "allegro_hand_qpos", "hand_keypoints", "match_control")
CONSOLE_TOPICS = ("match_control", "robot_reset")


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("file")
    p.add_argument("--mode", choices=["quest", "broker"], default="quest")
    p.add_argument("--host", default="auto", help="quest: the tracker's address (its host=); auto: search this PC")
    p.add_argument("--ports", default="8087,8110,8095", help="quest: right,left,button")
    p.add_argument("--broker", default="tcp://localhost:5555", help="broker frontend")
    p.add_argument("--speed", type=float, default=1.0)
    p.add_argument("--start", type=float, default=0.0, help="s from the start of the recording")
    p.add_argument("--end", type=float, default=None, help="s from the start of the recording")
    p.add_argument("--no-console", action="store_true", help="skip recorded pause/reset messages")
    p.add_argument("--engage", action="store_true",
                   help="broker: send an OK gesture (match_control) before the replayed data")
    p.add_argument("--list", action="store_true", help="print duration and marks, do not replay")
    return p.parse_args(argv)


def load(path):
    opener = gzip.open if path.endswith(".gz") else open
    with opener(path, "rt") as f:
        return [json.loads(line) for line in f if line.strip()]


def local_addresses():
    out = ["127.0.0.1"]
    for _, name in socket.if_nameindex():
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                packed = fcntl.ioctl(s.fileno(), 0x8915, struct.pack("256s", name[:15].encode()))  # SIOCGIFADDR
            out.append(socket.inet_ntoa(packed[20:24]))
        except OSError:
            pass
    return list(dict.fromkeys(out))


def listening(host, port) -> bool:
    try:
        with socket.create_connection((host, port), timeout=0.5):
            return True
    except OSError:
        return False


def ok_gestures(records, t_rec0):
    """Recording times (s) of OK gestures (match_control start 1 bursts) seen on the broker."""
    out, prev = [], -1e9
    for r in records:
        if r.get("type") == "broker" and r["stream"] == "match_control":
            try:
                start = int(pickle.loads(base64.b64decode(r["data"][-1])).get("start", 0) or 0)
            except Exception:
                continue
            if start == 1:
                if r["t"] - prev > 1.0:
                    out.append(r["t"] - t_rec0)
                prev = r["t"]
    return out


def is_console(topic, frames):
    if topic == "robot_reset":
        return True
    try:  # recorded by our own recorder from the local broker
        return int(pickle.loads(frames[-1]).get("start", 0) or 0) == 0
    except Exception:
        return False


def events(records, mode, console):
    """(t, kind, payload) in recording order; kind: quest stream name, 'broker', 'mark'."""
    out = []
    for r in records:
        if r["type"] == "mark":
            out.append((r["t"], "mark", r.get("text", "")))
        elif r["type"] == "quest" and mode == "quest" and r["stream"] in QUEST_STREAMS:
            out.append((r["t"], r["stream"], base64.b64decode(r["data"])))
        elif r["type"] == "broker":
            frames = [base64.b64decode(d) for d in r["data"]]
            topic = r["stream"]
            if mode == "broker" and topic in TRACKER_TOPICS and not (topic == "match_control" and is_console(topic, frames)):
                out.append((r["t"], "broker", [topic.encode()] + frames))
            elif console and topic in CONSOLE_TOPICS and is_console(topic, frames):
                out.append((r["t"], "broker", [topic.encode()] + frames))
    return sorted(out, key=lambda e: e[0])


def main(argv=None) -> int:
    args = parse_args(argv)
    records = load(args.file)
    t_rec0 = min(r["t"] for r in records)
    evs = events(records, args.mode, not args.no_console)
    duration = max(r["t"] for r in records) - t_rec0
    oks = ok_gestures(records, t_rec0)
    if args.list:
        print(f"{args.file}: {duration:.1f} s, {sum(e[1] in QUEST_STREAMS for e in evs)} quest messages")
        print("  OK gestures at s: " + (", ".join(f"{t:.1f}" for t in oks) or "none (nothing will move)"))
        for t, kind, payload in evs:
            if kind == "mark":
                print(f"  {t - t_rec0:7.1f} s  mark: {payload or '(no text)'}")
            elif kind == "broker" and payload[0].decode() in CONSOLE_TOPICS:
                print(f"  {t - t_rec0:7.1f} s  console: {payload[0].decode()}")
        return 0

    end = args.end if args.end is not None else duration
    evs = [e for e in evs if args.start <= e[0] - t_rec0 <= end]
    if not evs:
        print("nothing to replay in that range")
        return 1
    if args.engage and args.mode != "broker":
        print("[replay] --engage needs --mode broker (the tracker engages only on an OK gesture in the hand data)")
        return 1
    in_range = [t for t in oks if args.start <= t <= end]
    if args.engage:
        print("[replay] --engage: OK gesture sent before the replayed data")
    elif in_range:
        print("[replay] OK gestures in this part at s: " + ", ".join(f"{t:.1f}" for t in in_range))
    else:
        print("[replay] WARNING: no OK gesture in this part; nothing moves unless teleop is already engaged "
              "(--list shows where the OK gestures are; tracker output without one: --mode broker --engage)")
    ctx = zmq.Context.instance()
    socks = {}
    if args.mode == "quest":
        ports = [int(p) for p in args.ports.split(",")]
        if args.host == "auto":
            found = [h for h in local_addresses() if listening(h, ports[0])]
            if not found:
                print(f"[replay] no tracker listening on port {ports[0]} on this PC "
                      f"({', '.join(local_addresses())}); start H3, or pass --host")
                return 1
            args.host = found[0]
        elif not listening(args.host, ports[0]):
            print(f"[replay] nothing listens on {args.host}:{ports[0]}; --host must be the tracker's host= "
                  f"(listening here: {', '.join(h for h in local_addresses() if listening(h, ports[0])) or 'none'})")
            return 1
        print(f"[replay] tracker at {args.host}")
        for name, port in zip(QUEST_STREAMS, args.ports.split(",")):
            socks[name] = ctx.socket(zmq.PUSH)
            socks[name].connect(f"tcp://{args.host}:{port}")
    pub = ctx.socket(zmq.PUB)
    pub.connect(args.broker)
    time.sleep(0.5)  # let the connections settle

    print(f"[replay] {args.mode} mode, {args.start:.1f}-{end:.1f} s of {duration:.1f} s, speed {args.speed}")
    if args.engage:
        for _ in range(5):  # like the tracker on an OK gesture
            pub.send_multipart([b"match_control", pickle.dumps({"start": 1, "ts": time.time(), "timestamp": time.time()})])
            time.sleep(0.05)
    t0_wall, t0_rec = time.monotonic(), evs[0][0]
    sent, next_status = 0, 10.0
    ok_left = list(in_range)
    try:
        for t, kind, payload in evs:
            delay = t0_wall + (t - t0_rec) / args.speed - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            rel = t - t_rec0
            while ok_left and rel >= ok_left[0]:
                print(f"[replay] {ok_left.pop(0):7.1f} s  OK gesture in the recording")
            if rel - args.start >= next_status:
                print(f"[replay] {rel:7.1f} s / {end:.1f} s, {sent} messages")
                next_status += 10.0
            if kind == "mark":
                print(f"[replay] {rel:7.1f} s  mark: {payload or '(no text)'}")
            elif kind == "broker":
                pub.send_multipart(payload)
                if payload[0].decode() in CONSOLE_TOPICS and args.mode == "quest" or \
                        payload[0].decode() == "robot_reset":
                    print(f"[replay] {rel:7.1f} s  console: {payload[0].decode()}")
            else:
                socks[kind].send(payload, zmq.NOBLOCK)
            sent += 1
    except KeyboardInterrupt:
        pass
    except zmq.Again:
        print("[replay] tracker not reachable (is it running with this host and these ports?)")
        return 1
    finally:
        for s in list(socks.values()) + [pub]:
            s.close(linger=0)
    print(f"[replay] done, {sent} messages")
    return 0


if __name__ == "__main__":
    sys.exit(main())
