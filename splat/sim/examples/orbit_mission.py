"""Orbit a point of interest with the camera locked on it (inspection / 3D capture).

  python splat/sim/examples/orbit_mission.py                          # headless report
  python splat/sim/examples/orbit_mission.py --viz http://127.0.0.1:8800
  python splat/sim/examples/orbit_mission.py --export anafi_orbit.py  # Olympe script for the real drone
"""
import argparse, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from hackio_sim import Sim  # noqa: E402
import missions  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--scene", default="tuttle-park")
ap.add_argument("--viz", help="URL of a running Twinfield window")
ap.add_argument("--center", type=float, nargs=2, default=(3.0, 9.0), metavar=("X", "Y"),
                help="point to circle, m east/north of takeoff")
ap.add_argument("--radius", type=float, default=6.0)
ap.add_argument("--height", type=float, default=3.0)
ap.add_argument("--speed", type=float, default=2.0, help="m/s (1.5-3 is typical for orbits)")
ap.add_argument("--laps", type=float, default=1.0)
ap.add_argument("--ccw", action="store_true", help="counter-clockwise")
ap.add_argument("--export", help="write an Olympe script for the real ANAFI to this file")
a = ap.parse_args()

sim = Sim(a.scene, viz=a.viz)
plan = missions.orbit(a.center, a.radius, a.height, laps=a.laps, speed=a.speed, clockwise=not a.ccw)
print({k: plan[k] for k in ("radius_m", "height_m", "length_m", "est_time_s", "gimbal_pitch_deg", "centripetal_g")})
res = missions.fly(sim, plan)
print(json.dumps(res, indent=1))
sim.save_log("orbit_log.csv")
if a.export:
    # the real ANAFI stops briefly at each moveTo, so export fewer points per lap
    real = missions.orbit(a.center, a.radius, a.height, laps=a.laps, speed=a.speed,
                          clockwise=not a.ccw, points_per_lap=12)
    print("wrote", missions.write_olympe_script(real, sim.meta.get("origin", {"lat": 0, "lon": 0}), a.export))
