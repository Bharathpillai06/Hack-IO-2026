"""Headless stand-in for the Godot client's telemetry path (baseline behavior).

Reproduces what scripts/TouchObject.gd does, per drone node:
  - its own UDP socket that sends "subscribe" to the server (one per drone)
  - each frame, read at most ONE packet
  - parse with .replace("'", '"').replace("(", "").replace(")", "") + JSON
  - ghost = ghost.interpolate_with(latest_pos, delta)   (exponential smoothing)

Measures per run:
  transport  arrival time - sample_t of the drone's own entry
  age        each frame, how old the position the node is holding is
             (transport + time waiting for the next packet)
  rate       updates/s each drone node actually applied
  rx bytes   bytes received across all sockets
  frame cost ms spent per frame on telemetry work (what Godot's main thread pays)
  twin error distance from ghost (what the user sees) and from latest packet
             to ground truth, for mock://N-circle drones
"""
import json
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "python server", "classe refactoring"))
from classes.uav import mock_patterns  # noqa: E402

SERVER = ("127.0.0.1", 20003)
FPS = 60.0


def parse_godot_style(raw):
    s = raw.decode("ascii").replace("'", '"').replace("(", "").replace(")", "")
    return json.loads(s)


def pct(vals, p):
    if not vals:
        return float("nan")
    v = sorted(vals)
    return v[min(len(v) - 1, int(round(p / 100.0 * (len(v) - 1))))]


def dist(a, b):
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


class DroneNode:
    def __init__(self, name):
        self.name = name
        self.idx = mock_patterns.drone_index(name)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.pos = None
        self.ghost = None
        self.updates = 0
        self.t_latest = 0.0

    def subscribe(self, timeout=5.0):
        # Godot: put_packet("subscribe") then udp.wait() blocks for first packet
        self.sock.sendto(b"subscribe", SERVER)
        self.sock.settimeout(timeout)
        while True:
            d = parse_godot_style(self.sock.recv(65535))
            if self.name in d:
                break
        self.pos = [float(v) for v in d[self.name]["position"].split(",")]
        self.t_latest = float(d[self.name]["t"])
        self.ghost = list(self.pos)
        self.sock.setblocking(False)

    def stop(self):
        try:
            self.sock.sendto(b"stop", SERVER)
        finally:
            self.sock.close()


def run(names, duration, warmup=1.0):
    nodes = [DroneNode(n) for n in names]
    for n in nodes:
        n.subscribe()

    lat, age, frame_ms, err_ghost, err_latest = [], [], [], [], []
    rx_bytes = 0
    t_start = time.time()
    t_meas = t_start + warmup
    t_end = t_meas + duration
    last = time.perf_counter()
    next_frame = last

    while True:
        now_pc = time.perf_counter()
        delta = now_pc - last
        last = now_pc
        now = time.time()
        if now >= t_end:
            break
        measuring = now >= t_meas
        work0 = time.perf_counter()

        for n in nodes:
            # Godot: fantasma.interpolate_with(target, delta)
            w = min(delta, 1.0)
            n.ghost = [g + (p - g) * w for g, p in zip(n.ghost, n.pos)]
            # Godot: if get_available_packet_count() > 0: read ONE packet
            try:
                raw = n.sock.recv(65535)
            except BlockingIOError:
                raw = None
            if raw is not None:
                d = parse_godot_style(raw)
                e = d[n.name]
                n.pos = [float(v) for v in e["position"].split(",")]
                n.t_latest = float(e["t"])
                if measuring:
                    rx_bytes += len(raw)
                    n.updates += 1
                    lat.append(time.time() - float(e["t"]))
            if measuring:
                age.append(time.time() - n.t_latest)
            if measuring and n.name.endswith("-circle"):
                truth = mock_patterns.circle(n.idx, time.time())
                err_ghost.append(dist(n.ghost, truth))
                err_latest.append(dist(n.pos, truth))

        if measuring:
            frame_ms.append((time.perf_counter() - work0) * 1000.0)
        next_frame += 1.0 / FPS
        time.sleep(max(0.0, next_frame - time.perf_counter()))

    for n in nodes:
        n.stop()

    rates = [n.updates / duration for n in nodes]
    return {
        "drones": len(nodes),
        "transport_p50_ms": pct(lat, 50) * 1000,
        "transport_p95_ms": pct(lat, 95) * 1000,
        "age_p50_ms": pct(age, 50) * 1000,
        "age_p95_ms": pct(age, 95) * 1000,
        "rate_per_drone_hz": sum(rates) / len(rates),
        "rx_kbps": rx_bytes * 8 / 1000.0 / duration,
        "frame_cost_p50_ms": pct(frame_ms, 50),
        "frame_cost_p95_ms": pct(frame_ms, 95),
        "twin_err_ghost_p50_m": pct(err_ghost, 50),
        "twin_err_ghost_p95_m": pct(err_ghost, 95),
        "twin_err_latest_p50_m": pct(err_latest, 50),
        "twin_err_latest_p95_m": pct(err_latest, 95),
    }
