"""Coordinate helpers for AnafiUAV. Pure math so it can be tested without a drone.

Twinflie works in a local frame in meters: x = east, y = north, z = up,
with the origin at the drone's first GPS fix (normally the takeoff spot).
"""
import math

EARTH_R = 6378137.0  # m, WGS84 equatorial radius


def ll_to_local(lat0, lon0, lat, lon):
    """(lat, lon) -> (x east, y north) meters from origin. Flat-earth, fine for <1 km."""
    y = math.radians(lat - lat0) * EARTH_R
    x = math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
    return x, y


def local_to_ll(lat0, lon0, x, y):
    lat = lat0 + math.degrees(y / EARTH_R)
    lon = lon0 + math.degrees(x / (EARTH_R * math.cos(math.radians(lat0))))
    return lat, lon


def world_to_body(de, dn, yaw):
    """East/north displacement -> (front, right) for ardrone3 moveBy.

    yaw is the drone heading in radians, 0 = north, positive clockwise (toward east).
    """
    front = dn * math.cos(yaw) + de * math.sin(yaw)
    right = -dn * math.sin(yaw) + de * math.cos(yaw)
    return front, right


def valid_fix(lat, lon):
    return abs(lat) <= 90 and abs(lon) <= 180 and not (lat == 0 and lon == 0)
