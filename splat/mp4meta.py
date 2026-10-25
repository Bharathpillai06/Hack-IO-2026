"""Read Parrot ANAFI per-frame telemetry straight from an MP4, no ffmpeg/ffprobe needed.

Parses the MP4 box tree, finds the timed-metadata track whose sample entry is 'mett'
(Parrot video metadata), and decodes each sample as format v3 (see bench/anafi_meta.py).
Returns [] for videos from other cameras, so callers can fall back gracefully.
"""
import math
import struct

FLYING_STATES = {0: "landed", 1: "takingoff", 2: "hovering", 3: "flying", 4: "landing",
                 5: "emergency", 6: "usertakeoff", 7: "motor_ramping", 8: "emergency_landing"}


def _boxes(f, start, end):
    pos = start
    while pos + 8 <= end:
        f.seek(pos)
        size, typ = struct.unpack(">I4s", f.read(8))
        hdr = 8
        if size == 1:
            size = struct.unpack(">Q", f.read(8))[0]; hdr = 16
        elif size == 0:
            size = end - pos
        if size < hdr:
            return
        yield typ.decode("latin-1"), pos + hdr, pos + size
        pos += size


def _find(f, start, end, path):
    for typ, s, e in _boxes(f, start, end):
        if typ == path[0]:
            return (s, e) if len(path) == 1 else _find(f, s, e, path[1:])
    return None


def _read(f, s, e):
    f.seek(s); return f.read(e - s)


def _sample_table(f, stbl):
    s, e = stbl
    tab = {t: (bs, be) for t, bs, be in _boxes(f, s, e)}
    stsd = _read(f, *tab["stsd"])
    fmt = stsd[12:16].decode("latin-1")
    stsz = _read(f, *tab["stsz"])
    fixed, n = struct.unpack(">II", stsz[4:12])
    sizes = [fixed] * n if fixed else list(struct.unpack(f">{n}I", stsz[12:12 + 4 * n]))
    if "stco" in tab:
        b = _read(f, *tab["stco"]); m = struct.unpack(">I", b[4:8])[0]
        chunks = list(struct.unpack(f">{m}I", b[8:8 + 4 * m]))
    else:
        b = _read(f, *tab["co64"]); m = struct.unpack(">I", b[4:8])[0]
        chunks = list(struct.unpack(f">{m}Q", b[8:8 + 8 * m]))
    b = _read(f, *tab["stsc"]); m = struct.unpack(">I", b[4:8])[0]
    stsc = [struct.unpack(">III", b[8 + 12 * i:20 + 12 * i]) for i in range(m)]
    b = _read(f, *tab["stts"]); m = struct.unpack(">I", b[4:8])[0]
    stts = [struct.unpack(">II", b[8 + 8 * i:16 + 8 * i]) for i in range(m)]
    # sample offsets
    offsets, si = [], 0
    for ci, off in enumerate(chunks, start=1):
        per = next(spc for first, spc, _ in reversed(stsc) if first <= ci)
        for _ in range(per):
            if si >= n:
                break
            offsets.append(off); off += sizes[si]; si += 1
    times, t = [], 0
    for count, delta in stts:
        for _ in range(count):
            times.append(t); t += delta
    return fmt, sizes, offsets, times[:n]


def q(v, frac):
    return v / float(1 << frac)


def _ypr(w, x, y, z):
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    return yaw, pitch, roll


def parse_v3(buf):
    if len(buf) < 72 or buf[0:2] != b"P3":
        return None
    v = struct.unpack(">HHiiiihhhhhhhhhhhhhhhhHHHHHHIbBBB", buf[:72])
    (_id, length, gd, lat, lon, alt_sv, vn, ve, vd, _vair, dw, dx, dy, dz, _bw, _bx, _by, _bz,
     fw, fx, fy, fz, exp, gain, _r, _b, hfov, vfov, _link, rssi, batt, state, _mode) = v
    cam = [q(fw, 14), q(fx, 14), q(fy, 14), q(fz, 14)]
    yaw, pitch, roll = _ypr(*cam)
    sv = alt_sv & 0xFF
    r = {"ground_distance_m": q(gd, 16), "lat": q(lat, 22), "lon": q(lon, 22),
         "alt_m": q(alt_sv >> 8, 8), "gps_sv": sv,
         "v_north": q(vn, 8), "v_east": q(ve, 8), "v_down": q(vd, 8),
         "cam_q": cam, "cam_yaw_deg": yaw, "cam_pitch_deg": pitch, "cam_roll_deg": roll,
         "hfov_deg": q(hfov, 8), "vfov_deg": q(vfov, 8), "battery_pct": batt,
         "state": FLYING_STATES.get(state, str(state))}
    r["gps_ok"] = sv > 0 and abs(r["lat"]) <= 90 and not (r["lat"] == 0 and r["lon"] == 0)
    return r


def read_telemetry(path):
    """[{t_video_s, ...decoded fields}] for an ANAFI MP4, or [] if it has no Parrot metadata."""
    with open(path, "rb") as f:
        f.seek(0, 2); end = f.tell()
        moov = _find(f, 0, end, ["moov"])
        if not moov:
            return []
        for typ, s, e in _boxes(f, *moov):
            if typ != "trak":
                continue
            mdia = _find(f, s, e, ["mdia"])
            if not mdia:
                continue
            hdlr = _find(f, *mdia, ["hdlr"])
            mdhd = _find(f, *mdia, ["mdhd"])
            stbl = _find(f, *mdia, ["minf", "stbl"])
            if not (hdlr and mdhd and stbl):
                continue
            if _read(f, *hdlr)[8:12] != b"meta":
                continue
            md = _read(f, *mdhd)
            timescale = struct.unpack(">I", md[12:16] if md[0] == 0 else md[20:24])[0]
            fmt, sizes, offsets, times = _sample_table(f, stbl)
            if fmt != "mett":
                continue
            rows = []
            for size, off, t in zip(sizes, offsets, times):
                f.seek(off)
                r = parse_v3(f.read(size))
                if r:
                    r["t_video_s"] = t / timescale
                    rows.append(r)
            return rows
    return []


if __name__ == "__main__":
    import sys
    rows = read_telemetry(sys.argv[1])
    print(len(rows), "telemetry rows")
    if rows:
        print({k: rows[0][k] for k in ("t_video_s", "lat", "lon", "alt_m", "ground_distance_m", "hfov_deg", "state")})
