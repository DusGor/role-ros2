#!/usr/bin/env python3
"""
Record a Meta Quest teleop session of dex_bimanual_telep (https://github.com/YufengJin/dex_bimanual_telep).

Two taps, both stored as raw bytes with arrival time (JSON lines, gzip if the name ends in .gz):
  quest   sits between the Quest app and the hand tracker: binds the app's PUSH ports
          (quest.yaml: right 8087, left 8110, button 8095), records every message and forwards it
          to the tracker, which is started on other ports:
            python scripts/hand_tracker_meta_quest_v2.py port_right=18087 port_left=18110 port_button=18095
  broker  subscribes to everything on the broker backend (broker.py, :5556)
Enter in this terminal writes a mark (optional text, e.g. "left hand behind back").

  python3 record_quest_session.py session.jsonl.gz
  python3 record_quest_session.py --summary session.jsonl.gz
"""

import argparse
import base64
import collections
import gzip
import json
import socket
import sys
import threading
import time

import zmq

QUEST_STREAMS = ("right", "left", "button")


def open_out(path: str, mode: str):
    return gzip.open(path, mode + "t") if path.endswith(".gz") else open(path, mode)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("file")
    p.add_argument("--summary", action="store_true", help="print a summary of an existing recording")
    p.add_argument("--no-quest", action="store_true", help="broker tap only")
    p.add_argument("--no-broker", action="store_true", help="quest tap only")
    p.add_argument("--quest-ports", default="8087,8110,8095", help="right,left,button (app side)")
    p.add_argument("--forward-host", default="10.42.0.1", help="tracker bind address (quest.yaml host)")
    p.add_argument("--forward-ports", default="18087,18110,18095", help="right,left,button (tracker side)")
    p.add_argument("--broker", default="tcp://localhost:5556")
    return p.parse_args(argv)


class Recorder:
    def __init__(self, path: str):
        self._f = open_out(path, "w")
        self._lock = threading.Lock()
        self.counts = collections.Counter()

    def write(self, record: dict) -> None:
        line = json.dumps(record)
        with self._lock:
            self._f.write(line + "\n")
            self.counts[record.get("stream") or record["type"]] += 1

    def close(self) -> None:
        with self._lock:
            self._f.close()


def b64(data: bytes) -> str:
    return base64.b64encode(data).decode("ascii")


def run_quest_tap(ctx, rec: Recorder, args, stop: threading.Event) -> None:
    ports = [int(p) for p in args.quest_ports.split(",")]
    fwd_ports = [int(p) for p in args.forward_ports.split(",")]
    poller = zmq.Poller()
    pairs = {}
    for name, port, fwd in zip(QUEST_STREAMS, ports, fwd_ports):
        pull = ctx.socket(zmq.PULL)
        pull.bind(f"tcp://*:{port}")
        push = ctx.socket(zmq.PUSH)
        push.setsockopt(zmq.SNDHWM, 10)
        push.connect(f"tcp://{args.forward_host}:{fwd}")
        poller.register(pull, zmq.POLLIN)
        pairs[pull] = (name, push)
    while not stop.is_set():
        for sock, _ in poller.poll(100):
            data = sock.recv()
            name, push = pairs[sock]
            rec.write({"type": "quest", "stream": name, "t": time.time(), "data": b64(data)})
            try:
                push.send(data, zmq.NOBLOCK)
            except zmq.Again:
                rec.counts["forward_dropped"] += 1
    for pull, (_, push) in pairs.items():
        pull.close(linger=0)
        push.close(linger=0)


def run_broker_tap(ctx, rec: Recorder, args, stop: threading.Event) -> None:
    sub = ctx.socket(zmq.SUB)
    sub.connect(args.broker)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    while not stop.is_set():
        if sub.poll(100):
            parts = sub.recv_multipart()
            topic = parts[0].decode(errors="replace")
            rec.write({"type": "broker", "stream": topic, "t": time.time(),
                       "data": [b64(p) for p in parts[1:]]})
    sub.close(linger=0)


def run_marks(rec: Recorder, stop: threading.Event) -> None:
    for line in sys.stdin:
        if stop.is_set():
            break
        rec.write({"type": "mark", "t": time.time(), "text": line.strip()})
        print(f"[record] mark: {line.strip() or '(no text)'}")


def record(args) -> int:
    rec = Recorder(args.file)
    rec.write({"type": "meta", "t": time.time(), "host": socket.gethostname(), "args": vars(args)})
    ctx = zmq.Context.instance()
    stop = threading.Event()
    threads = []
    if not args.no_quest:
        threads.append(threading.Thread(target=run_quest_tap, args=(ctx, rec, args, stop), daemon=True))
    if not args.no_broker:
        threads.append(threading.Thread(target=run_broker_tap, args=(ctx, rec, args, stop), daemon=True))
    threading.Thread(target=run_marks, args=(rec, stop), daemon=True).start()
    for t in threads:
        t.start()
    print(f"[record] writing {args.file}; Enter = mark, Ctrl+C = stop")
    t0, last = time.time(), collections.Counter()
    try:
        while True:
            time.sleep(1.0)
            now = dict(rec.counts)
            rates = ", ".join(f"{k} {now[k] - last[k]}/s" for k in sorted(now) if k != "meta")
            print(f"[record] {time.time() - t0:6.0f} s  {rates or 'no data yet'}")
            last = collections.Counter(now)
    except KeyboardInterrupt:
        pass
    stop.set()
    for t in threads:
        t.join(timeout=2.0)
    rec.close()
    print(f"[record] stopped: {dict(rec.counts)}")
    return 0


def summary(path: str) -> int:
    counts, first, last, marks = collections.Counter(), None, None, []
    with open_out(path, "r") as f:
        for line in f:
            r = json.loads(line)
            if r["type"] == "meta":
                print(f"host {r['host']}, args {r['args']}")
                continue
            first = r["t"] if first is None else first
            last = r["t"]
            if r["type"] == "mark":
                marks.append(r)
            else:
                counts[(r["type"], r["stream"])] += 1
    if first is None:
        print("no data")
        return 1
    dur = last - first
    print(f"duration {dur:.1f} s")
    for (kind, stream), n in sorted(counts.items()):
        print(f"  {kind:6s} {stream:22s} {n:7d}  {n / max(dur, 1e-9):6.1f}/s")
    for m in marks:
        print(f"  mark at {m['t'] - first:7.2f} s: {m['text']}")
    return 0


def main(argv=None) -> int:
    args = parse_args(argv)
    return summary(args.file) if args.summary else record(args)


if __name__ == "__main__":
    sys.exit(main())
