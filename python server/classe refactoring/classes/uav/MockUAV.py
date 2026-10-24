"""Drop-in stand-in for UAV_ciccio.UAV so the server, routes and Godot client
run with no hardware.

URIs:
  mock://<id>          kinematic drone; flies routes via go_to/take_off/land
  mock://<id>-circle   flies a fixed circle (see mock_patterns) for benchmarking

Telemetry is produced at LOG_HZ like the Crazyflie position log (100 ms).
Each sample stores sample_t (unix time) so latency can be measured end to end.
"""
import math
import random
import threading
import time

from pymitter import EventEmitter

from classes.uav.UAVEvents import UAVEvents
from classes.uav import mock_patterns

X_THRESHOLD = 0.07
Y_THRESHOLD = 0.07
Z_THRESHOLD = 0.14


class MockUAV:

    DEFAULT_HEIGHT = 0.2
    LOG_HZ = 10.0          # matches the real 100 ms log period
    SPEED = 0.5            # m/s max for kinematic moves
    NOISE_STD = 0.0        # m, set >0 to mimic UWB jitter
    MOVE_TIMEOUT = 30.0    # s, go_to gives up after this

    def __init__(self, uri) -> None:
        self.uri = uri
        self.idx = mock_patterns.drone_index(uri)
        self.pattern = "circle" if uri.endswith("-circle") else None
        self.events = EventEmitter()

        self.status = "connected"
        self.connected = False
        self.positionEstimated = False
        self.scf = None        # no radio link; anchor calls check for this
        self.mc = None         # set once "controller" is ready, like the real UAV

        self.x, self.y, self.z = mock_patterns.home(self.idx)
        self.starting_z = self.z
        self.battery = 4.1     # volts, same unit as pm.vbat
        self.sample_t = 0.0

        self._target = (self.x, self.y, self.z)
        self._running = False
        self._thread = None

    # --- connection lifecycle -------------------------------------------
    def connect(self):
        threading.Thread(target=self._connect_async, daemon=True).start()

    def _connect_async(self):
        time.sleep(0.05 + random.random() * 0.1)   # link setup; random phase vs sender
        self.connected = True
        self.events.emit(UAVEvents.CONNECTED, self)
        self._running = True
        self._thread = threading.Thread(target=self._log_loop, daemon=True)
        self._thread.start()
        time.sleep(0.2)                       # pretend Kalman settling
        self.positionEstimated = True
        self.events.emit(UAVEvents.POSITION_READY)
        self.mc = self                        # truthy, so start_all_drones accepts it
        self.events.emit(UAVEvents.CONTROLLER_READY, self)

    def disconnect(self):
        self._running = False
        self.status = "disconnected"
        self.events.emit(UAVEvents.DISCONNECTED, self)

    # --- telemetry loop --------------------------------------------------
    def _log_loop(self):
        period = 1.0 / self.LOG_HZ
        nxt = time.perf_counter()
        last = time.time()
        while self._running:
            now = time.time()
            dt = now - last
            last = now
            if self.pattern == "circle":
                x, y, z = mock_patterns.circle(self.idx, now)
            else:
                x, y, z = self._step_toward_target(dt)
            if self.NOISE_STD > 0:
                x += random.gauss(0, self.NOISE_STD)
                y += random.gauss(0, self.NOISE_STD)
                z += random.gauss(0, self.NOISE_STD)
            self.x, self.y, self.z = x, y, z
            self.sample_t = now
            self.battery = max(3.0, self.battery - 0.00002)
            nxt += period
            time.sleep(max(0.0, nxt - time.perf_counter()))

    def _step_toward_target(self, dt):
        tx, ty, tz = self._target
        dx, dy, dz = tx - self.x, ty - self.y, tz - self.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        step = self.SPEED * dt
        if dist <= step or dist == 0:
            return tx, ty, tz
        k = step / dist
        return self.x + dx * k, self.y + dy * k, self.z + dz * k

    def _fly_to(self, x, y, z):
        self._target = (x, y, z)
        deadline = time.time() + self.MOVE_TIMEOUT
        while time.time() < deadline:
            if (abs(x - self.x) < X_THRESHOLD and abs(y - self.y) < Y_THRESHOLD
                    and abs(z - self.z) < Z_THRESHOLD):
                return True
            time.sleep(0.02)
        return False

    # --- same command surface as UAV_ciccio.UAV --------------------------
    def take_off(self, my_height):
        self.starting_z = self.z
        self._fly_to(self.x, self.y, float(my_height))

    def land(self):
        self._fly_to(self.x, self.y, self.starting_z)

    def go_to(self, x, y, z=0):
        self._fly_to(float(x), float(y), float(z))

    def spinning(self):
        time.sleep(1.0)

    def reset_estimator(self):
        pass

    def resetPositionEstimation(self):
        pass
