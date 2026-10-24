"""Match each extracted frame to its telemetry row, compute local ENU, and write
COLMAP pair list + per-image telemetry. Frames are named <video>_<second>.jpg (1 fps)."""
import csv, glob, json, math, os

EARTH_R = 6378137.0
imgs = sorted(os.path.basename(p) for p in glob.glob("splatprep/images/*.jpg"))
meta = {v: list(csv.DictReader(open(f"{v}_meta.csv"))) for v in ("P2150215", "P2160216")}
lat0 = float(meta["P2150215"][0]["lat"]); lon0 = float(meta["P2150215"][0]["lon"])
alt0 = float(meta["P2150215"][0]["alt_m"]) - float(meta["P2150215"][0]["ground_distance_m"])

tele = {}
for name in imgs:
    v, sec = name[:-4].split("_")
    t = int(sec)
    rows = meta[v]
    r = min(rows, key=lambda x: abs(float(x["t_video_s"]) - t))
    la, lo = float(r["lat"]), float(r["lon"])
    e = math.radians(lo - lon0) * EARTH_R * math.cos(math.radians(lat0))
    n = math.radians(la - lat0) * EARTH_R
    u = float(r["alt_m"]) - alt0
    tele[name] = {"video": v, "t": float(r["t_video_s"]), "enu": [e, n, u],
                  "ground_distance": float(r["ground_distance_m"]),
                  "cam_q": [float(r[k]) for k in ("cam_qw", "cam_qx", "cam_qy", "cam_qz")],
                  "cam_yaw": float(r["cam_yaw_deg"]), "cam_pitch": float(r["cam_pitch_deg"]),
                  "state": r["state"]}

pairs = set()
for i, a in enumerate(imgs):
    for b in imgs[i + 1:]:
        ta, tb = tele[a], tele[b]
        same = ta["video"] == tb["video"]
        if same and abs(ta["t"] - tb["t"]) <= 6:
            pairs.add((a, b)); continue
        d = math.dist(ta["enu"][:2], tb["enu"][:2])
        dyaw = abs((ta["cam_yaw"] - tb["cam_yaw"] + 180) % 360 - 180)
        if d < 6 and dyaw < 45:
            pairs.add((a, b))
with open("pairs.txt", "w") as f:
    for a, b in sorted(pairs):
        f.write(f"{a} {b}\n")
json.dump({"origin": {"lat": lat0, "lon": lon0, "alt_ground": alt0}, "frames": tele},
          open("telemetry.json", "w"), indent=1)
print(len(imgs), "images,", len(pairs), "pairs")
