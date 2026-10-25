"""Fly a 6 m square at 3 m height, check every leg against the splat, and report.

  python splat/sim/examples/square_mission.py                         # headless, prints a report
  python splat/sim/examples/square_mission.py --viz http://127.0.0.1:8800   # watch it in the sim app
"""
import argparse, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from hackio_sim import Sim  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--scene", default="tuttle-park")
ap.add_argument("--viz", help="URL of a running HackIO-Sim window")
a = ap.parse_args()

sim = Sim(a.scene, viz=a.viz)
waypoints = [(0, 6, 3), (6, 6, 3), (6, 0, 3), (0, 0, 3)]
sim.takeoff(3.0)
prev = (sim.p[0], sim.p[1], sim.p[2])
for wp in waypoints:
    clear = sim.path_is_clear(prev, wp)
    ok = sim.goto(*wp, speed=2.0)
    print(f"leg to {wp}: {'reached' if ok else 'TIMEOUT'}, path {'clear' if clear else 'BLOCKED'}, t={sim.t:.1f}s")
    prev = wp
sim.land()
print(sim.report())
print("log:", sim.save_log("square_mission_log.csv"))
