"""Twinfield scripted simulation: fly the sim drone from Python and test your code.

Same drone model as the web sim (splat/site/index.html), runs headless and much faster
than real time, and can stream to the 3D view so you can watch a script fly.

    from hackio_sim import Sim
    sim = Sim("tuttle-park")              # loads scenes/tuttle-park (occupancy + recorded track)
    sim.takeoff(2.0)
    sim.goto(5, 10, 3)                    # meters, x east / y north / z up
    sim.velocity(vx=1.0, vy=0, vz=0, yaw_rate=0, seconds=3)
    sim.land()
    print(sim.report())                   # distance, max speed, collisions, ...
    sim.save_log("flight.csv")

Frame: ENU meters with the origin at the scene's first GPS fix (the takeoff spot for ANAFI scenes).
Yaw: heading in degrees clockwise from north, like the ANAFI.
"""
import bisect
import csv
import json
import math
import os
import time
import urllib.request

D2R = math.pi / 180
G = 9.81
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))


class Params:
    """Drone model constants (kept identical to the web sim)."""
    max_tilt = 25 * D2R      # rad
    tau_att = 0.15           # s, attitude response
    tau_vz = 0.35            # s, climb response
    drag = 0.35              # 1/s, linear drag
    kv = 2.2                 # 1/s, velocity loop gain
    max_v = 6.0              # m/s horizontal (velocity mode)
    max_vz = 2.5             # m/s
    max_yaw_rate = 90.0      # deg/s


class Collision(Exception):
    pass


class Sim:
    def __init__(self, scene=None, scenes_dir=None, dt=0.01, viz=None, realtime=False,
                 stop_on_collision=False, params=Params):
        """scene: name of a folder in scenes/ (or a path). None = empty world.
        viz: URL of a running sim app (e.g. "http://127.0.0.1:8800") to watch the flight live.
        realtime: pace the simulation to wall-clock time (automatic when viz is set)."""
        self.P = params
        self.dt = dt
        self.viz = viz.rstrip("/") if viz else None
        self.realtime = realtime or bool(viz)
        self.stop_on_collision = stop_on_collision
        self.occ, self.cell, self.track, self.meta = set(), 0.5, None, {}
        if scene:
            d = scene if os.path.isdir(scene) else os.path.join(scenes_dir or os.path.join(REPO, "scenes"), scene)
            if not os.path.isdir(d):
                raise FileNotFoundError(f"scene folder not found: {d}")
            occ = json.load(open(os.path.join(d, "occupancy.json")))
            self.cell = occ["cell"]
            self.occ = {tuple(c) for c in occ["cells"]}
            if os.path.exists(os.path.join(d, "track.json")):
                self.track = json.load(open(os.path.join(d, "track.json")))
            if os.path.exists(os.path.join(d, "scene_meta.json")):
                self.meta = json.load(open(os.path.join(d, "scene_meta.json")))
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self, x=0.0, y=0.0, yaw=None):
        if yaw is None:
            yaw = self.track["yaw"][0] if self.track else 0.0
        self.p = [x, y, 0.0]
        self.v = [0.0, 0.0, 0.0]
        self.roll = self.pitch = 0.0
        self.yaw = yaw % 360
        self.landed = True
        self.t = 0.0
        self.collisions = 0
        self.log = []
        self._wall0 = time.time()
        self._last_viz = 0.0
        return self.state()

    def state(self):
        return {"t": round(self.t, 3), "x": self.p[0], "y": self.p[1], "z": self.p[2],
                "vx": self.v[0], "vy": self.v[1], "vz": self.v[2],
                "roll_deg": self.roll / D2R, "pitch_deg": self.pitch / D2R, "yaw_deg": self.yaw,
                "speed": math.hypot(self.v[0], self.v[1]), "landed": self.landed,
                "near_obstacle": self.near_obstacle()}

    # ------------------------------------------------------------------ physics
    def _step(self, roll_cmd, pitch_cmd, vz_cmd, yaw_rate):
        P, dt = self.P, self.dt
        roll_cmd = max(-P.max_tilt, min(P.max_tilt, roll_cmd))
        pitch_cmd = max(-P.max_tilt, min(P.max_tilt, pitch_cmd))
        if self.landed and vz_cmd > 0:
            self.landed = False
        if not self.landed:
            yr = self.yaw * D2R
            fE, fN, rE, rN = math.sin(yr), math.cos(yr), math.cos(yr), -math.sin(yr)
            a = min(1.0, dt / P.tau_att)
            self.pitch += (pitch_cmd - self.pitch) * a
            self.roll += (roll_cmd - self.roll) * a
            yaw_rate = max(-P.max_yaw_rate, min(P.max_yaw_rate, yaw_rate))
            self.yaw = (self.yaw + yaw_rate * dt) % 360
            aF, aR = G * math.tan(self.pitch), G * math.tan(self.roll)
            self.v[0] += (aF * fE + aR * rE - P.drag * self.v[0]) * dt
            self.v[1] += (aF * fN + aR * rN - P.drag * self.v[1]) * dt
            vz = max(-P.max_vz, min(P.max_vz, vz_cmd))
            self.v[2] += (vz - self.v[2]) * min(1.0, dt / P.tau_vz)
            for i in range(3):
                self.p[i] += self.v[i] * dt
            if self.p[2] <= 0:
                self.p[2] = 0.0
                self.v[2] = max(0.0, self.v[2])
                if vz_cmd <= 0 and math.hypot(self.v[0], self.v[1]) < 0.3:
                    self.landed = True
                    self.v = [0.0, 0.0, 0.0]; self.roll = self.pitch = 0.0
        self.t += dt
        hit = self.in_collision()
        if hit:
            self.collisions += 1
            if self.stop_on_collision:
                raise Collision(f"collision at t={self.t:.2f}s, position {self.p}")
        self.log.append((round(self.t, 3), *[round(v, 4) for v in self.p], *[round(v, 4) for v in self.v],
                         round(self.roll / D2R, 2), round(self.pitch / D2R, 2), round(self.yaw, 2), int(hit)))
        self._pace_and_stream()

    def _tilt_for_velocity(self, vE, vN):
        """Velocity loop: desired world velocity -> roll/pitch commands (like the web sim)."""
        yr = self.yaw * D2R
        fE, fN, rE, rN = math.sin(yr), math.cos(yr), math.cos(yr), -math.sin(yr)
        aE, aN = self.P.kv * (vE - self.v[0]), self.P.kv * (vN - self.v[1])
        return math.atan2(aE * rE + aN * rN, G), math.atan2(aE * fE + aN * fN, G)

    # ------------------------------------------------------------------ commands
    def velocity(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0, seconds=1.0, frame="world"):
        """Hold a velocity command. frame="world": vx east, vy north. frame="body": vx forward, vy right."""
        for _ in range(max(1, round(seconds / self.dt))):
            if frame == "body":
                yr = self.yaw * D2R
                vE = vx * math.sin(yr) + vy * math.cos(yr)
                vN = vx * math.cos(yr) - vy * math.sin(yr)
            else:
                vE, vN = vx, vy
            roll, pitch = self._tilt_for_velocity(vE, vN)
            self._step(roll, pitch, vz, yaw_rate)
        return self.state()

    def attitude(self, roll_deg=0.0, pitch_deg=0.0, yaw_rate=0.0, vz=0.0, seconds=1.0):
        """Hold an attitude command (ANAFI manual-control style). Positive pitch = forward."""
        for _ in range(max(1, round(seconds / self.dt))):
            self._step(roll_deg * D2R, pitch_deg * D2R, vz, yaw_rate)
        return self.state()

    def hover(self, seconds=1.0):
        return self.velocity(0, 0, 0, 0, seconds)

    def takeoff(self, height=2.0, timeout=15.0):
        t_end = self.t + timeout
        self.landed = False
        while self.t < t_end and abs(height - self.p[2]) > 0.05:
            roll, pitch = self._tilt_for_velocity(0, 0)
            self._step(roll, pitch, max(-1.0, min(1.5, 1.5 * (height - self.p[2]))), 0)
        return self.state()

    def goto(self, x, y, z=None, yaw=None, speed=3.0, tol=0.15, timeout=60.0):
        """Fly to a point (meters, ENU). Returns True if reached within tol before timeout."""
        z = self.p[2] if z is None else z
        t_end = self.t + timeout
        while self.t < t_end:
            dx, dy, dz = x - self.p[0], y - self.p[1], z - self.p[2]
            d = math.hypot(dx, dy)
            if d < tol and abs(dz) < tol and math.hypot(self.v[0], self.v[1]) < 0.3:
                return True
            k = min(speed, 1.2 * d) / max(d, 1e-6)
            roll, pitch = self._tilt_for_velocity(dx * k, dy * k)
            yaw_rate = 0.0
            if yaw is not None:
                dyaw = (yaw - self.yaw + 540) % 360 - 180
                yaw_rate = 2 * dyaw
            self._step(roll, pitch, max(-1.5, min(1.5, 1.5 * dz)), yaw_rate)
        return False

    def fly_path(self, points, speed=3.0, look_at=None, yaw="path", lookahead=1.5, tol=0.3,
                 timeout=None):
        """Fly through a list of ENU waypoints without stopping at each one (carrot following).
        look_at: (x, y[, z]) to keep the nose on a point (orbits). Otherwise yaw="path" turns the
        nose along the direction of travel, yaw=None holds the current heading.
        Returns {"reached", "time_s", "mean_xtrack_m", "max_xtrack_m"}."""
        pts = [tuple(p) for p in points]
        if len(pts) < 2:
            pts = [tuple(self.p)] + pts
        seg_len = [math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
        total = sum(seg_len)
        timeout = timeout or 30.0 + 3.0 * total / max(speed, 0.1)
        t0, t_end, s_done, i, xt = self.t, self.t + timeout, 0.0, 0, []
        while self.t < t_end:
            # advance along the path: project onto the current segment, move on when past its end
            while True:
                a, b = pts[i], pts[i + 1]
                L = seg_len[i] or 1e-6
                u = sum((self.p[j] - a[j]) * (b[j] - a[j]) for j in range(3)) / L
                if u >= L and i < len(pts) - 2:
                    s_done += seg_len[i]; i += 1
                else:
                    break
            u = max(0.0, min(L, u))
            foot = [a[j] + (b[j] - a[j]) * u / L for j in range(3)]
            xt.append(math.dist(foot, self.p))
            s_tgt = min(total, s_done + u + lookahead)
            k, s = i, s_done
            while k < len(seg_len) - 1 and s + seg_len[k] < s_tgt:
                s += seg_len[k]; k += 1
            f = 0.0 if not seg_len[k] else max(0.0, min(1.0, (s_tgt - s) / seg_len[k]))
            tgt = [pts[k][j] + (pts[k + 1][j] - pts[k][j]) * f for j in range(3)]
            to_end = total - (s_done + u) + math.dist(foot, self.p)
            if to_end < tol and math.hypot(self.v[0], self.v[1]) < 0.5:
                return {"reached": True, "time_s": round(self.t - t0, 2),
                        "mean_xtrack_m": round(sum(xt) / len(xt), 3), "max_xtrack_m": round(max(xt), 3)}
            dx, dy = tgt[0] - self.p[0], tgt[1] - self.p[1]
            d = math.hypot(dx, dy)
            v = min(speed, 1.2 * to_end) / max(d, 1e-6)
            roll, pitch = self._tilt_for_velocity(dx * v, dy * v)
            want, ff = None, 0.0
            if look_at is not None:
                rx, ry = look_at[0] - self.p[0], look_at[1] - self.p[1]
                want = math.degrees(math.atan2(rx, ry))
                ff = math.degrees((rx * self.v[1] - ry * self.v[0]) / max(rx * rx + ry * ry, 1e-6))
            elif yaw == "path" and d > 0.2:
                want = math.degrees(math.atan2(dx, dy))
            yaw_rate = 0.0 if want is None else ff + 3.0 * ((want - self.yaw + 540) % 360 - 180)
            self._step(roll, pitch, max(-1.5, min(1.5, 1.5 * (tgt[2] - self.p[2]))), yaw_rate)
        return {"reached": False, "time_s": round(self.t - t0, 2),
                "mean_xtrack_m": round(sum(xt) / max(1, len(xt)), 3), "max_xtrack_m": round(max(xt or [0]), 3)}

    def land(self, timeout=30.0):
        t_end = self.t + timeout
        while self.t < t_end and not self.landed:
            roll, pitch = self._tilt_for_velocity(0, 0)
            self._step(roll, pitch, -max(0.3, min(1.0, self.p[2])), 0)
        return self.state()

    def fly_recorded_track(self, gain=1.2):
        """Autopilot along the scene's recorded GPS track. Returns tracking errors (m) per step."""
        if not self.track:
            raise RuntimeError("this scene has no recorded track")
        T, E = self.track["t"], self.track["enu"]
        self.reset(E[0][0], E[0][1], self.track["yaw"][0])
        self.landed = False
        errs = []

        def at(t):
            i = max(0, min(len(T) - 2, bisect.bisect_right(T, t) - 1))
            a = max(0.0, min(1.0, (t - T[i]) / max(1e-6, T[i + 1] - T[i])))
            return [E[i][j] + a * (E[i + 1][j] - E[i][j]) for j in range(3)]

        while self.t < T[-1]:
            tgt, ahead = at(self.t), at(self.t + 0.5)
            vdes = [(ahead[j] - tgt[j]) / 0.5 + gain * (tgt[j] - self.p[j]) for j in range(3)]
            roll, pitch = self._tilt_for_velocity(vdes[0], vdes[1])
            self._step(roll, pitch, vdes[2], 0)
            errs.append(math.dist(tgt, self.p))
        return errs

    # ------------------------------------------------------------------ sensing
    def _cell(self, p):
        return tuple(int(math.floor(v / self.cell)) for v in p)

    def in_collision(self):
        return bool(self.occ) and self._cell(self.p) in self.occ

    def near_obstacle(self, radius=1.0):
        if not self.occ:
            return False
        c = self._cell(self.p); r = max(1, int(math.ceil(radius / self.cell)))
        return any((c[0] + i, c[1] + j, c[2] + k) in self.occ
                   for i in range(-r, r + 1) for j in range(-r, r + 1) for k in range(-r, r + 1))

    def path_is_clear(self, a, b, step=0.25):
        """True if the straight segment a->b (ENU points) stays out of occupied cells."""
        n = max(1, int(math.dist(a, b) / step))
        return all(self._cell([a[j] + (b[j] - a[j]) * i / n for j in range(3)]) not in self.occ for i in range(n + 1))

    # ------------------------------------------------------------------ output
    def report(self):
        dist = sum(math.dist(self.log[i - 1][1:4], self.log[i][1:4]) for i in range(1, len(self.log)))
        return {"sim_time_s": round(self.t, 2), "distance_m": round(dist, 2),
                "max_speed_mps": round(max((math.hypot(r[4], r[5]) for r in self.log), default=0), 2),
                "max_height_m": round(max((r[3] for r in self.log), default=0), 2),
                "collision_steps": self.collisions, "final": {k: round(v, 3) if isinstance(v, float) else v
                                                              for k, v in self.state().items()}}

    def save_log(self, path):
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "z", "vx", "vy", "vz", "roll_deg", "pitch_deg", "yaw_deg", "collision"])
            w.writerows(self.log)
        return path

    def _pace_and_stream(self):
        if self.realtime:
            lag = self.t - (time.time() - self._wall0)
            if lag > 0:
                time.sleep(lag)
        if self.viz and self.t - self._last_viz >= 0.05:
            self._last_viz = self.t
            body = json.dumps({"p": self.p, "v": self.v, "roll": self.roll, "pitch": self.pitch,
                               "yaw": self.yaw, "landed": self.landed, "t": self.t}).encode()
            try:
                req = urllib.request.Request(self.viz + "/api/pose", data=body, method="POST",
                                             headers={"Content-Type": "application/json"})
                urllib.request.build_opener(urllib.request.ProxyHandler({})).open(req, timeout=0.5).read()
            except Exception:
                pass
