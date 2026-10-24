"""Recorded ANAFI flight (P2150215) -> site/track.json in the same ENU frame as the splat."""
import csv, json, math
EARTH_R = 6378137.0
o = json.load(open("telemetry.json"))["origin"]
rows = list(csv.DictReader(open("P2150215_meta.csv")))
out = {"t": [], "enu": [], "yaw": []}
last = -1
for r in rows:
    t = float(r["t_video_s"])
    if t - last < 0.1 or r["lat"] == "":
        continue
    last = t
    e = math.radians(float(r["lon"]) - o["lon"]) * EARTH_R * math.cos(math.radians(o["lat"]))
    n = math.radians(float(r["lat"]) - o["lat"]) * EARTH_R
    out["t"].append(round(t, 2))
    out["enu"].append([round(e, 3), round(n, 3), round(float(r["ground_distance_m"]), 3)])
    out["yaw"].append(round(float(r["cam_yaw_deg"]) % 360, 1))
# light smoothing of GPS jitter (5-sample moving average on E/N)
E = [p[0] for p in out["enu"]]; N = [p[1] for p in out["enu"]]
for i in range(len(E)):
    a, b = max(0, i - 2), min(len(E), i + 3)
    out["enu"][i][0] = round(sum(E[a:b]) / (b - a), 3); out["enu"][i][1] = round(sum(N[a:b]) / (b - a), 3)
json.dump(out, open("site/track.json", "w"))
print(len(out["t"]), "samples")
