"""Deterministic flight patterns shared by MockUAV (server) and the bench client.

Both sides compute the same ground-truth position from wall-clock time, so the
client can measure how far the displayed twin is from where the drone really is.
"""
import math

CIRCLE_RADIUS = 1.0   # m
CIRCLE_HEIGHT = 1.0   # m
CIRCLE_SPEED = 0.5    # m/s tangential
GRID_SPACING = 3.0    # m between drone circle centers


def drone_index(uri):
    """mock://7 or mock://7-circle -> 7"""
    tail = uri.split("://", 1)[-1]
    return int(tail.split("-", 1)[0])


def home(idx):
    """Ground start position for drone idx, laid out on a 5-wide grid."""
    return (idx % 5) * GRID_SPACING, (idx // 5) * GRID_SPACING, 0.0


def circle(idx, t):
    """Ground-truth (x, y, z) at unix time t for the circle pattern."""
    cx, cy, _ = home(idx)
    w = CIRCLE_SPEED / CIRCLE_RADIUS
    phase = idx * 0.7
    a = w * t + phase
    return (cx + CIRCLE_RADIUS * math.cos(a),
            cy + CIRCLE_RADIUS * math.sin(a),
            CIRCLE_HEIGHT)
