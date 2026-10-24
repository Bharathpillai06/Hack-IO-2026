"""Extract ANAFI per-frame telemetry (Parrot video metadata v3) from an MP4 to CSV.

Usage:
  python bench/anafi_meta.py P2150215.MP4                 # writes P2150215_meta.csv
  python bench/anafi_meta.py P2150215.MP4 -o out.csv

Needs ffmpeg/ffprobe on PATH. Layout follows Parrot's "Embedded Video Metadata" docs
(format v3, big-endian, fixed point). One row per video frame.
"""
import argparse
import csv
import math
import os
import struct
import subprocess
import sys

FLYING_STATES = {0: "landed", 1: "takingoff", 2: "hovering", 3: "flying",
                 4: "landing", 5: "emergency", 6: "usertakeoff",
                 7: "motor_ramping", 8: "emergency_landing"}


def q(v, frac):
    return v / float(1 << frac)


def quat_to_ypr(w, x, y, z):
    """NED quaternion -> yaw, pitch, roll in degrees (ZYX)."""
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.degrees(math.asin(s))
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    return yaw, pitch, roll


def parse_v3(buf):
    if len(buf) < 72 or buf[0:2] != b"P3":
        return None
    (_id, length, gd, lat, lon, alt_sv, vn, ve, vd, vair,
     dw, dx, dy, dz, bw, bx, by, bz, fw, fx, fy, fz,
     exp, gain, awb_r, awb_b, hfov, vfov, link, rssi, batt, state, mode
     ) = struct.unpack(">HHiiiihhhhhhhhhhhhhhhhHHHHHHIbBBB", buf[:72])
    r = {
        "ground_distance_m": q(gd, 16),
        "lat": q(lat, 22),
        "lon": q(lon, 22),
        "alt_m": q(alt_sv >> 8, 8),
        "gps_sv": alt_sv & 0xFF,
        "v_north": q(vn, 8), "v_east": q(ve, 8), "v_down": q(vd, 8),
        "drone_qw": q(dw, 14), "drone_qx": q(dx, 14), "drone_qy": q(dy, 14), "drone_qz": q(dz, 14),
        "cam_qw": q(fw, 14), "cam_qx": q(fx, 14), "cam_qy": q(fy, 14), "cam_qz": q(fz, 14),
        "exposure_ms": q(exp, 8), "iso": gain,
        "hfov_deg": q(hfov, 8), "vfov_deg": q(vfov, 8),
        "rssi_dbm": rssi, "battery_pct": batt,
        "state": FLYING_STATES.get(state, str(state)),
    }
    for p, qq in (("drone", (dw, dx, dy, dz)), ("cam", (fw, fx, fy, fz))):
        yaw, pitch, roll = quat_to_ypr(*(q(v, 14) for v in qq))
        r[p + "_yaw_deg"], r[p + "_pitch_deg"], r[p + "_roll_deg"] = yaw, pitch, roll
    # extensions: look for the E1 timestamp extension
    off = 72
    total = 4 + length * 4
    r["ts_us"] = ""
    while off + 4 <= min(total, len(buf)):
        eid, elen = struct.unpack(">HH", buf[off:off + 4])
        if eid == 0x4531 and off + 12 <= len(buf):
            h, l = struct.unpack(">II", buf[off + 4:off + 12])
            r["ts_us"] = (h << 32) | l
        off += 4 + elen * 4
    if not (-90 <= r["lat"] <= 90) or r["lat"] == 500 or r["gps_sv"] == 0:
        r["lat"] = r["lon"] = ""
    return r


def packets(mp4):
    """Yield (pts_seconds, bytes) for each Parrot metadata sample."""
    probe = subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "stream=index:stream_tags=handler_name",
         "-of", "csv=p=0", mp4], capture_output=True, text=True, check=True).stdout
    idx = next((line.split(",")[0] for line in probe.splitlines()
                if "ParrotVideoMetadata" in line), None)
    if idx is None:
        sys.exit("no ParrotVideoMetadata track in this file (re-exported or trimmed?)")
    times = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", idx, "-show_entries",
         "packet=pts_time,size", "-of", "csv=p=0", mp4],
        capture_output=True, text=True, check=True).stdout.split()
    data = subprocess.run(
        ["ffmpeg", "-v", "error", "-i", mp4, "-map", f"0:{idx}", "-c", "copy", "-f", "data", "-"],
        capture_output=True, check=True).stdout
    off = 0
    for line in times:
        t, size = line.split(",")[:2]
        size = int(size)
        yield float(t), data[off:off + size]
        off += size


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mp4")
    ap.add_argument("-o", "--out")
    a = ap.parse_args()
    out = a.out or os.path.splitext(a.mp4)[0] + "_meta.csv"
    rows = []
    for t, buf in packets(a.mp4):
        r = parse_v3(buf)
        if r:
            r = {"t_video_s": round(t, 4), **r}
            rows.append(r)
    if not rows:
        sys.exit("metadata track found but no v3 records decoded")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    gps = [r for r in rows if r["lat"] != ""]
    alts = [r["alt_m"] for r in rows]
    print(f"{len(rows)} frames -> {out}")
    print(f"  duration {rows[-1]['t_video_s']:.1f} s, GPS fix on {len(gps)} frames")
    if gps:
        print(f"  start lat/lon {gps[0]['lat']:.6f}, {gps[0]['lon']:.6f}")
    print(f"  altitude {min(alts):.1f} to {max(alts):.1f} m, "
          f"ground distance {min(r['ground_distance_m'] for r in rows):.1f} to "
          f"{max(r['ground_distance_m'] for r in rows):.1f} m")
    print(f"  states: {sorted(set(r['state'] for r in rows))}, "
          f"battery {rows[0]['battery_pct']}% -> {rows[-1]['battery_pct']}%")


if __name__ == "__main__":
    main()
