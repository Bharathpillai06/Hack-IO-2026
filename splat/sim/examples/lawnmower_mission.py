"""Lawnmower survey (mapping / search-and-rescue sweep) over the Tuttle Park flight area.

  python splat/sim/examples/lawnmower_mission.py                          # headless report
  python splat/sim/examples/lawnmower_mission.py --viz http://127.0.0.1:8800
  python splat/sim/examples/lawnmower_mission.py --export anafi_lawnmower.py   # Olympe script for the real drone
"""
import argparse, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from hackio_sim import Sim  # noqa: E402
import missions  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--scene", default="tuttle-park")
ap.add_argument("--viz", help="URL of a running Twinfield window")
ap.add_argument("--height", type=float, default=5.0, help="m above takeoff")
ap.add_argument("--speed", type=float, default=3.0, help="m/s (mapping apps use 5-8)")
ap.add_argument("--side-overlap", type=float, default=0.7)
ap.add_argument("--export", help="write an Olympe script for the real ANAFI to this file")
a = ap.parse_args()

# Area the real flight covered (ENU m from the takeoff spot). Swap in your own polygon,
# or build one from GPS: [missions.ll_to_enu(lat, lon, sim.meta["origin"]) for lat, lon in corners]
AREA = [(-4, 0), (10, 0), (10, 18), (-4, 18)]

sim = Sim(a.scene, viz=a.viz)
plan = missions.lawnmower(AREA, a.height, side_overlap=a.side_overlap, speed=a.speed)
print({k: plan[k] for k in ("lines", "spacing_m", "angle_deg", "length_m", "est_time_s",
                            "photo_every_m", "photo_every_s", "footprint_m")})
res = missions.fly(sim, plan)
print(json.dumps(res, indent=1))
sim.save_log("lawnmower_log.csv")
if a.export:
    print("wrote", missions.write_olympe_script(plan, sim.meta.get("origin", {"lat": 0, "lon": 0}), a.export))
