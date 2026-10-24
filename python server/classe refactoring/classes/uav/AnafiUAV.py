"""Parrot ANAFI adapter with the same surface as UAV_ciccio.UAV, built on Olympe.

URI: anafi://192.168.42.1   (drone's own Wi-Fi; via SkyController 3 use 192.168.53.1)
From Godot or the HTTP API type it as  anafi:__192.168.42.1

Position: local frame, x east / y north / z up in meters, origin = first GPS fix.
Between GPS updates the position is dead-reckoned from SpeedChanged (NED), so the
twin gets updates at the speed-event rate instead of the GPS rate.
Without a GPS fix (indoors) go_to falls back to relative moveBy, which drifts.

Safety: targets outside MAX_RADIUS_M from origin or above MAX_ALT_M are refused.
Requires Linux and Python <= 3.12 (Olympe breaks on 3.13).
"""
import math
import os
import threading
import time

# Olympe imports PyOpenGL; on Wayland (Ubuntu 24.04 default) PyOpenGL picks EGL and
# Olympe's GLX import then crashes. Force GLX (works through XWayland).
os.environ.setdefault("PYOPENGL_PLATFORM", "glx")

from pymitter import EventEmitter

from classes.uav.UAVEvents import UAVEvents
from classes.uav import anafi_geo as geo

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveTo, moveBy
from olympe.messages.ardrone3.PilotingState import (
    FlyingStateChanged, PositionChanged, SpeedChanged, AltitudeChanged,
    AttitudeChanged, moveToChanged)
from olympe.messages.ardrone3.PilotingEvent import moveByEnd
from olympe.messages.common.CommonState import BatteryStateChanged
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})


class _Listener(olympe.EventListener):
    """Pushes Olympe telemetry events into the owning AnafiUAV."""

    def __init__(self, uav, *args, **kwargs):
        self.uav = uav
        super().__init__(*args, **kwargs)

    @olympe.listen_event(PositionChanged(_policy="wait"))
    def _pos(self, event, scheduler):
        a = event.args
        self.uav._on_gps(a["latitude"], a["longitude"])

    @olympe.listen_event(SpeedChanged(_policy="wait"))
    def _speed(self, event, scheduler):
        a = event.args
        self.uav._on_speed(a["speedX"], a["speedY"], a["speedZ"])

    @olympe.listen_event(AltitudeChanged(_policy="wait"))
    def _alt(self, event, scheduler):
        self.uav._on_alt(event.args["altitude"])

    @olympe.listen_event(AttitudeChanged(_policy="wait"))
    def _att(self, event, scheduler):
        self.uav.yaw = event.args["yaw"]

    @olympe.listen_event(BatteryStateChanged(_policy="wait"))
    def _bat(self, event, scheduler):
        self.uav._on_battery(event.args["percent"])

    @olympe.listen_event(FlyingStateChanged(_policy="wait"))
    def _fly(self, event, scheduler):
        self.uav.flying_state = str(event.args["state"]).split(".")[-1]


class AnafiUAV:

    DEFAULT_HEIGHT = 1.0
    MAX_RADIUS_M = 30.0     # geofence around origin
    MAX_ALT_M = 10.0
    MOVE_TIMEOUT = 60.0
    CONNECT_TIMEOUT = 15.0

    def __init__(self, uri) -> None:
        self.uri = uri
        self.ip = uri.split("://", 1)[-1]
        self.events = EventEmitter()
        self.drone = olympe.Drone(self.ip)
        self.listener = None

        self.status = "connected"
        self.connected = False
        self.positionEstimated = False
        self.scf = None
        self.mc = None
        self.flying_state = "unknown"

        self.origin = None          # (lat, lon) of first fix
        self.x = self.y = self.z = 0.0
        self.yaw = 0.0
        self.battery = 0.0          # volts-like, see _on_battery
        self.battery_percent = None
        self.starting_z = 0.0
        self.sample_t = 0.0

        self._v = (0.0, 0.0, 0.0)   # east, north, up m/s
        self._last_dr = None
        self._lock = threading.Lock()
        # event counters so we can report real telemetry rates
        self.counts = {"gps": 0, "speed": 0, "alt": 0, "battery": 0}

    # --- connection -------------------------------------------------------
    def connect(self):
        threading.Thread(target=self._connect_async, daemon=True).start()

    def _connect_async(self):
        ok = False
        try:
            ok = self.drone.connect(retry=3, timeout=self.CONNECT_TIMEOUT)
        except Exception as e:      # noqa: BLE001
            print("ANAFI connect error:", e)
        if not ok:
            self.status = "disconnected"
            # emit first: drones_collection.failed() releases its lock,
            # then connected=True lets add_drone's wait loop exit
            self.events.emit(UAVEvents.CONNECTION_FAILED, self, "connect failed")
            self.connected = True
            return
        self.listener = _Listener(self, self.drone)
        self.listener.subscribe()
        self.connected = True
        self.events.emit(UAVEvents.CONNECTED, self)
        self.positionEstimated = True
        self.events.emit(UAVEvents.POSITION_READY)
        self.mc = self
        self.events.emit(UAVEvents.CONTROLLER_READY, self)

    def disconnect(self):
        try:
            if self.listener:
                self.listener.unsubscribe()
            self.drone.disconnect()
        finally:
            self.status = "disconnected"
            self.events.emit(UAVEvents.DISCONNECTED, self)

    # --- telemetry handlers (called from Olympe's thread) -----------------
    def _on_gps(self, lat, lon):
        self.counts["gps"] += 1
        if not geo.valid_fix(lat, lon):
            return
        with self._lock:
            if self.origin is None:
                self.origin = (lat, lon)
            self.x, self.y = geo.ll_to_local(self.origin[0], self.origin[1], lat, lon)
            self._last_dr = time.time()
            self.sample_t = self._last_dr

    def _on_speed(self, vn, ve, vd):
        self.counts["speed"] += 1
        now = time.time()
        with self._lock:
            if self._last_dr is not None:
                dt = min(now - self._last_dr, 0.5)
                ve0, vn0, _ = self._v
                self.x += ve0 * dt
                self.y += vn0 * dt
            self._v = (ve, vn, -vd)
            self._last_dr = now
            self.sample_t = now

    def _on_alt(self, alt):
        self.counts["alt"] += 1
        with self._lock:
            self.z = alt
            self.sample_t = time.time()

    def _on_battery(self, percent):
        self.counts["battery"] += 1
        self.battery_percent = percent
        # Godot shows pm.vbat*1000 like a Crazyflie; map 0-100 % onto 3.0-4.2 V
        self.battery = 3.0 + 1.2 * percent / 100.0

    # --- commands (blocking, like UAV_ciccio) -----------------------------
    def take_off(self, my_height):
        self.starting_z = self.z
        ok = self.drone(
            TakeOff()
            >> FlyingStateChanged(state="hovering", _timeout=20)
        ).wait().success()
        if not ok:
            print("ANAFI takeoff failed")
            return
        h = min(float(my_height), self.MAX_ALT_M)
        dz = h - self.z
        if abs(dz) > 0.2:
            self._move_by(0.0, 0.0, -dz)

    def land(self):
        self.drone(
            Landing() >> FlyingStateChanged(state="landed", _timeout=30)
        ).wait()

    def go_to(self, x, y, z=0):
        x, y, z = float(x), float(y), float(z)
        if math.hypot(x, y) > self.MAX_RADIUS_M or z > self.MAX_ALT_M or z < 0.5:
            print(f"ANAFI refusing go_to({x:.1f},{y:.1f},{z:.1f}): outside geofence")
            return
        if self.origin is not None:
            lat, lon = geo.local_to_ll(self.origin[0], self.origin[1], x, y)
            self.drone(
                moveTo(lat, lon, z, MoveTo_Orientation_mode.NONE, 0.0)
                >> moveToChanged(status="DONE", _timeout=self.MOVE_TIMEOUT)
            ).wait()
        else:
            front, right = geo.world_to_body(x - self.x, y - self.y, self.yaw)
            self._move_by(front, right, -(z - self.z))

    def spinning(self):
        for _ in range(4):
            self._move_by(0.0, 0.0, 0.0, math.pi / 2)

    def _move_by(self, front, right, down, dpsi=0.0):
        self.drone(
            moveBy(front, right, down, dpsi)
            >> moveByEnd(_timeout=self.MOVE_TIMEOUT)
        ).wait()

    def reset_estimator(self):
        pass

    def resetPositionEstimation(self):
        pass
