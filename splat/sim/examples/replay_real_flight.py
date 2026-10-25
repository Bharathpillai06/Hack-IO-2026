"""Fly the scene's recorded ANAFI GPS track with the autopilot and report how well the model tracks it."""
import argparse, os, statistics, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from hackio_sim import Sim  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--scene", default="tuttle-park")
ap.add_argument("--viz")
a = ap.parse_args()
sim = Sim(a.scene, viz=a.viz)
errs = sim.fly_recorded_track()
print(f"tracked {sim.t:.0f} s of real flight: mean error {statistics.mean(errs):.2f} m, "
      f"p95 {sorted(errs)[int(0.95 * len(errs))]:.2f} m, max {max(errs):.2f} m")
print(sim.report())
