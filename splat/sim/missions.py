"""Real-world drone missions for the Hack-IO-2026 sim: lawnmower grid and orbit.

Both build a plain list of ENU waypoints (meters from the scene origin, x east, y north, z up),
so the same plan can be flown in the sim (Sim.fly_path), scored (coverage / orbit_quality), and
exported as an Olympe script for the real ANAFI (write_olympe_script).

Defaults follow what commercial missions use:
  grid:  70-80% side overlap, 80% front overlap, nadir camera (gimbal -90), lines along the
         long side of the area, constant height and speed (mapping apps fly 5-8 m/s, slower
         with a rolling shutter).
  orbit: camera locked on the center, 1.5-3 m/s, a photo every ~2 s, gimbal tilted to the target.
Camera FOV defaults are the ANAFI's (69 x 42.3 deg, read from its MP4 metadata).
"""
import math

D2R = math.pi / 180
R_EARTH = 6378137.0
ANAFI_HFOV, ANAFI_VFOV = 69.0, 42.3
PART107_MAX_ALT_M = 121.9          # 400 ft AGL


# ---------------------------------------------------------------- coordinates
def ll_to_enu(lat, lon, origin):
    """Lat/lon (deg) -> local x east, y north (m) around origin {"lat", "lon"}. Fine for < 10 km."""
    return ((lon - origin["lon"]) * D2R * R_EARTH * math.cos(origin["lat"] * D2R),
            (lat - origin["lat"]) * D2R * R_EARTH)


def enu_to_ll(x, y, origin):
    return (origin["lat"] + y / R_EARTH / D2R,
            origin["lon"] + x / (R_EARTH * math.cos(origin["lat"] * D2R)) / D2R)


def bearing_deg(frm, to):
    """Compass heading (deg clockwise from north) from point frm to point to."""
    return math.degrees(math.atan2(to[0] - frm[0], to[1] - frm[1])) % 360


def footprint(height, hfov=ANAFI_HFOV, vfov=ANAFI_VFOV):
    """Ground footprint (across-track width, along-track length) in m of a nadir camera."""
    return 2 * height * math.tan(hfov / 2 * D2R), 2 * height * math.tan(vfov / 2 * D2R)


def _inside(pt, poly):
    x, y, c = pt[0], pt[1], False
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i][:2], poly[i - 1][:2]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            c = not c
    return c


def _path_len(pts):
    return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


# ---------------------------------------------------------------- lawnmower grid
def lawnmower(polygon, height, side_overlap=0.75, front_overlap=0.8, angle_deg=None, spacing=None,
              speed=5.0, hfov=ANAFI_HFOV, vfov=ANAFI_VFOV, margin=0.0):
    """Boustrophedon (back-and-forth) survey of a polygon [(x, y), ...] in ENU meters.

    angle_deg: line direction, deg clockwise from north (None = try every 5 deg, keep the shortest).
    spacing: distance between lines (None = from camera footprint and side_overlap).
    Returns a plan dict with "waypoints" [(x, y, z)], "spacing_m", "photo_every_m", "photo_every_s",
    "angle_deg", "lines", "length_m", "est_time_s"."""
    w, l = footprint(height, hfov, vfov)
    spacing = spacing or w * (1 - side_overlap)
    if angle_deg is None:
        best = min((lawnmower(polygon, height, side_overlap, front_overlap, a, spacing, speed, hfov, vfov, margin)
                    for a in range(0, 180, 5)), key=lambda p: (p["length_m"], p["lines"]))
        return best
    a = angle_deg * D2R
    d = (math.sin(a), math.cos(a))            # along-line unit vector (east, north)
    n = (math.cos(a), -math.sin(a))           # across-line unit vector
    uv = [(p[0] * d[0] + p[1] * d[1], p[0] * n[0] + p[1] * n[1]) for p in polygon]
    vmin, vmax = min(q[1] for q in uv), max(q[1] for q in uv)
    nlines = max(1, int(math.ceil((vmax - vmin) / spacing - 1e-9)))
    v0 = (vmin + vmax) / 2 - (nlines - 1) * spacing / 2   # center the lines in the area
    wps, flip = [], False
    for k in range(nlines):
        v = v0 + k * spacing
        xs = []
        for i in range(len(uv)):
            (u1, v1), (u2, v2) = uv[i], uv[i - 1]
            if (v1 > v) != (v2 > v):
                xs.append(u1 + (v - v1) * (u2 - u1) / (v2 - v1))
        if len(xs) < 2:
            continue
        xs.sort()
        u_a, u_b = xs[0] - margin, xs[-1] + margin          # outer span (concave gaps are flown through)
        ends = [(u_a, v), (u_b, v)][::-1] if flip else [(u_a, v), (u_b, v)]
        for u, vv in ends:
            wps.append((u * d[0] + vv * n[0], u * d[1] + vv * n[1], float(height)))
        flip = not flip
    photo_m = l * (1 - front_overlap)
    length = _path_len(wps)
    return {"type": "lawnmower", "waypoints": wps, "height_m": height, "speed_mps": speed,
            "spacing_m": round(spacing, 3), "photo_every_m": round(photo_m, 3),
            "photo_every_s": round(photo_m / speed, 3), "angle_deg": angle_deg % 180,
            "lines": len(wps) // 2, "length_m": round(length, 2), "est_time_s": round(length / speed, 1),
            "polygon": [tuple(p[:2]) for p in polygon], "gimbal_pitch_deg": -90.0,
            "footprint_m": (round(w, 2), round(l, 2))}


# ---------------------------------------------------------------- orbit
def orbit(center, radius, height, laps=1.0, speed=2.0, clockwise=True, start_bearing_deg=None,
          target_height=0.0, points_per_lap=36, photo_every_s=2.0):
    """Circle a point of interest with the camera on it.

    center: (x, y) ENU. start_bearing_deg: where on the circle to start, seen from the center
    (None = 180, i.e. south of the center). target_height: height of the thing you look at.
    Each waypoint has a yaw facing the center. Returns a plan dict."""
    sb = 180.0 if start_bearing_deg is None else start_bearing_deg
    n = max(8, int(round(points_per_lap * laps)))
    sign = 1 if clockwise else -1
    wps, yaws = [], []
    for k in range(n + 1):
        b = (sb + sign * 360.0 * laps * k / n) * D2R
        p = (center[0] + radius * math.sin(b), center[1] + radius * math.cos(b), float(height))
        wps.append(p)
        yaws.append(bearing_deg(p, center))
    gimbal = -math.degrees(math.atan2(height - target_height, radius))
    length = 2 * math.pi * radius * laps
    return {"type": "orbit", "waypoints": wps, "yaws_deg": yaws, "center": tuple(center[:2]),
            "radius_m": radius, "height_m": height, "speed_mps": speed, "laps": laps,
            "clockwise": clockwise, "gimbal_pitch_deg": round(gimbal, 2), "photo_every_s": photo_every_s,
            "length_m": round(length, 2), "est_time_s": round(length / speed, 1),
            "centripetal_g": round(speed ** 2 / radius / 9.81, 3)}


# ---------------------------------------------------------------- fly + score in the sim
def fly(sim, plan, check_obstacles=True):
    """Take off, fly to the first waypoint, fly the plan, land. Returns a result dict."""
    wps = plan["waypoints"]
    blocked = []
    if check_obstacles and sim.occ:
        prev = (sim.p[0], sim.p[1], plan["height_m"])
        for i, wp in enumerate(wps):
            if not sim.path_is_clear(prev, wp):
                blocked.append(i)
            prev = wp
    sim.takeoff(plan["height_m"])
    look = plan["center"] if plan["type"] == "orbit" else None
    start_yaw = plan["yaws_deg"][0] if look else bearing_deg(wps[0], wps[1])
    sim.goto(*wps[0], yaw=start_yaw, speed=min(3.0, plan["speed_mps"]))
    sim.hover(1.0)
    i0 = len(sim.log)
    res = sim.fly_path(wps, speed=plan["speed_mps"], look_at=look, yaw="path")
    seg = sim.log[i0:]
    sim.land()
    res.update(blocked_legs=blocked, collisions=sim.collisions)
    if plan["type"] == "lawnmower":
        res.update(coverage(seg, plan))
    else:
        res.update(orbit_quality(seg, plan))
    return res


def coverage(log, plan, cell=0.5, hfov=ANAFI_HFOV, vfov=ANAFI_VFOV):
    """Fraction of the survey polygon seen by >= 1 and >= 2 simulated photos.
    Photos trigger every plan["photo_every_m"] of travel, nadir camera, footprint from height."""
    poly = plan["polygon"]
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    cells = [(x0 + cell / 2, y0 + cell / 2)
             for x0 in _frange(min(xs), max(xs), cell) for y0 in _frange(min(ys), max(ys), cell)
             if _inside((x0 + cell / 2, y0 + cell / 2), poly)]
    photos, travelled, last = [], plan["photo_every_m"], None
    for r in log:
        x, y, z, yaw = r[1], r[2], r[3], r[9]
        if last is not None:
            travelled += math.dist((x, y), last)
        last = (x, y)
        if travelled >= plan["photo_every_m"] and z > 0.5:
            photos.append((x, y, z, yaw)); travelled = 0.0
    seen = [0] * len(cells)
    for x, y, z, yaw in photos:
        w, l = footprint(z, hfov, vfov)
        a = yaw * D2R
        fwd, right = (math.sin(a), math.cos(a)), (math.cos(a), -math.sin(a))
        rr = math.hypot(w, l) / 2
        for k, (cx, cy) in enumerate(cells):
            dx, dy = cx - x, cy - y
            if abs(dx) > rr or abs(dy) > rr:
                continue
            if abs(dx * fwd[0] + dy * fwd[1]) <= l / 2 and abs(dx * right[0] + dy * right[1]) <= w / 2:
                seen[k] += 1
    n = max(1, len(cells))
    return {"photos": len(photos), "coverage_pct": round(100 * sum(s >= 1 for s in seen) / n, 1),
            "overlap2_pct": round(100 * sum(s >= 2 for s in seen) / n, 1)}


def orbit_quality(log, plan):
    """Radius error and how far the nose points off the center, over the orbit."""
    c, R = plan["center"], plan["radius_m"]
    rerr, aim = [], []
    for r in log:
        x, y, yaw = r[1], r[2], r[9]
        rerr.append(abs(math.hypot(x - c[0], y - c[1]) - R))
        aim.append(abs((bearing_deg((x, y), c) - yaw + 540) % 360 - 180))
    n = max(1, len(log))
    return {"mean_radius_err_m": round(sum(rerr) / n, 3), "max_radius_err_m": round(max(rerr or [0]), 3),
            "mean_aim_err_deg": round(sum(aim) / n, 2), "max_aim_err_deg": round(max(aim or [0]), 2),
            "photos": int(plan["est_time_s"] / plan["photo_every_s"])}


def _frange(a, b, step):
    v = a
    while v < b:
        yield v
        v += step


# ---------------------------------------------------------------- real ANAFI export
def check_limits(plan, max_alt_m=30.0, max_dist_m=60.0, takeoff_enu=(0.0, 0.0)):
    """Raise ValueError if the plan leaves the safety box around takeoff (Part 107 caps 400 ft)."""
    for x, y, z in plan["waypoints"]:
        x, y = x - takeoff_enu[0], y - takeoff_enu[1]
        if z > min(max_alt_m, PART107_MAX_ALT_M):
            raise ValueError(f"waypoint height {z:.1f} m is above the {max_alt_m} m limit")
        if math.hypot(x, y) > max_dist_m:
            raise ValueError(f"waypoint ({x:.1f}, {y:.1f}) is more than {max_dist_m} m from takeoff")
    return True


def write_olympe_script(plan, origin, path, takeoff_enu=(0.0, 0.0), max_alt_m=30.0, max_dist_m=60.0,
                        record_video=True):
    """Write a standalone Olympe script that flies this plan on a real ANAFI.

    origin: scene origin {"lat", "lon"} (scene_meta.json). takeoff_enu: where in the scene the drone
    will take off; moveTo altitude is above the takeoff point, so heights are relative to it.
    The script asks for confirmation, sets the gimbal, records video, and lands on Ctrl+C."""
    check_limits(plan, max_alt_m, max_dist_m, takeoff_enu)
    rows = []
    for i, (x, y, z) in enumerate(plan["waypoints"]):
        lat, lon = enu_to_ll(x, y, origin)
        hdg = plan["yaws_deg"][i] if plan["type"] == "orbit" else None
        rows.append((round(lat, 8), round(lon, 8), round(z, 2), None if hdg is None else round(hdg, 1)))
    tlat, tlon = enu_to_ll(takeoff_enu[0], takeoff_enu[1], origin)
    src = f'''"""Generated by splat/sim/missions.py: {plan["type"]} mission for a Parrot ANAFI.

Waypoints: {len(rows)}, length {plan["length_m"]} m, height {plan["height_m"]} m above takeoff.
Expected takeoff spot: {tlat:.7f}, {tlon:.7f}. Fly it in the sim first.
Check the area, the airspace (Part 107: 400 ft, line of sight, not over people), and the wind.

  python {path.replace(chr(92), "/").split("/")[-1]}            # DRONE_IP defaults to 192.168.42.1
"""
import os, sys, time
os.environ.setdefault("PYOPENGL_PLATFORM", "glx")
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveTo, CancelMoveTo
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, moveToChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode as OM
from olympe.messages import gimbal
from olympe.messages.camera import start_recording, stop_recording, set_camera_mode

WAYPOINTS = {rows!r}   # (lat, lon, alt above takeoff m, heading deg or None)
GIMBAL_PITCH = {plan["gimbal_pitch_deg"]}
RECORD = {record_video!r}


def main():
    drone = olympe.Drone(os.environ.get("DRONE_IP", "192.168.42.1"))
    if not drone.connect(timeout=45):
        sys.exit("could not connect to the drone")
    try:
        if not drone(GPSFixStateChanged(_policy="check")).success():
            print("waiting for GPS fix...")
            drone(GPSFixStateChanged(fixed=1, _timeout=60)).wait()
        if input(f"Fly {{len(WAYPOINTS)}} waypoints? Type FLY to start: ").strip() != "FLY":
            return
        drone(TakeOff() >> FlyingStateChanged(state="hovering", _timeout=20)).wait()
        drone(gimbal.set_target(gimbal_id=0, control_mode="position",
                                yaw_frame_of_reference="none", yaw=0.0,
                                pitch_frame_of_reference="absolute", pitch=GIMBAL_PITCH,
                                roll_frame_of_reference="none", roll=0.0)).wait()
        if RECORD:
            drone(set_camera_mode(cam_id=0, value="recording")).wait()
            drone(start_recording(cam_id=0)).wait()
        for i, (lat, lon, alt, hdg) in enumerate(WAYPOINTS):
            if hdg is None:
                cmd = moveTo(lat, lon, alt, OM.TO_TARGET, 0.0)
            else:
                cmd = moveTo(lat, lon, alt, OM.HEADING_DURING, hdg)
            ok = drone(cmd >> moveToChanged(status="DONE", _timeout=60, _policy="wait")).wait().success()
            print(f"waypoint {{i + 1}}/{{len(WAYPOINTS)}}: {{'ok' if ok else 'timeout'}}")
            if not ok:
                drone(CancelMoveTo()).wait()
                break
    except KeyboardInterrupt:
        print("stopping")
        drone(CancelMoveTo()).wait()
    finally:
        if RECORD:
            drone(stop_recording(cam_id=0)).wait()
        drone(Landing() >> FlyingStateChanged(state="landed", _timeout=60)).wait()
        drone.disconnect()


if __name__ == "__main__":
    main()
'''
    with open(path, "w", encoding="utf-8") as f:
        f.write(src)
    return path
