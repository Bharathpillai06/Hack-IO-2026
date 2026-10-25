"""pytest splat/sim/tests  -- regression checks for the drone model and mission helpers."""
import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from hackio_sim import Sim  # noqa: E402


def test_takeoff_reaches_height():
    sim = Sim()
    s = sim.takeoff(2.0)
    assert abs(s["z"] - 2.0) < 0.1 and not s["landed"]


def test_goto_reaches_waypoint():
    sim = Sim()
    sim.takeoff(2.0)
    assert sim.goto(5, 5, 3)
    assert math.dist(sim.p, (5, 5, 3)) < 0.2


def test_velocity_mode_respects_tilt_limit():
    sim = Sim()
    sim.takeoff(2.0)
    sim.velocity(vx=20, seconds=3)
    assert max(abs(r[8]) for r in sim.log) <= 25.01      # pitch_deg column


def test_land_stops():
    sim = Sim()
    sim.takeoff(2.0)
    s = sim.land()
    assert s["landed"] and s["z"] == 0


def test_tuttle_park_scene_and_replay():
    sim = Sim("tuttle-park")
    errs = sim.fly_recorded_track()
    assert len(errs) > 1000 and sum(errs) / len(errs) < 1.0


# ---------------------------------------------------------------- missions
import missions  # noqa: E402

SCENE = "tuttle-park"

AREA = [(-4, 0), (10, 0), (10, 18), (-4, 18)]


def test_lawnmower_plan_geometry():
    plan = missions.lawnmower(AREA, 5.0, side_overlap=0.7, angle_deg=0)
    w, _ = missions.footprint(5.0)
    assert abs(plan["spacing_m"] - w * 0.3) < 1e-3
    assert plan["lines"] == len(plan["waypoints"]) // 2
    xs = sorted({round(p[0], 3) for p in plan["waypoints"]})          # north-south lines
    assert all(abs((xs[i + 1] - xs[i]) - plan["spacing_m"]) < 1e-3 for i in range(len(xs) - 1))
    assert all(p[2] == 5.0 for p in plan["waypoints"])


def test_lawnmower_auto_angle_runs_along_long_side():
    plan = missions.lawnmower([(0, 0), (40, 0), (40, 10), (0, 10)], 5.0)   # wide east-west strip
    assert plan["angle_deg"] in (90, 270 % 180)


def test_lawnmower_full_coverage_and_gaps_detected():
    sim = Sim(SCENE)
    good = missions.fly(sim, missions.lawnmower(AREA, 5.0, side_overlap=0.7, speed=3.0))
    assert good["reached"] and good["collisions"] == 0 and good["coverage_pct"] > 98
    sim = Sim(SCENE)
    sparse = missions.fly(sim, missions.lawnmower(AREA, 5.0, spacing=12.0, speed=3.0, angle_deg=0))
    assert sparse["coverage_pct"] < good["coverage_pct"] - 10


def test_orbit_keeps_radius_and_camera_on_target():
    sim = Sim(SCENE)
    res = missions.fly(sim, missions.orbit((3, 9), 6.0, 3.0, speed=2.0))
    assert res["reached"] and res["collisions"] == 0
    assert res["mean_radius_err_m"] < 0.2 and res["mean_aim_err_deg"] < 3


def test_olympe_export_compiles_and_respects_limits(tmp_path):
    origin = {"lat": 40.0, "lon": -83.0}
    p = missions.write_olympe_script(missions.orbit((3, 9), 6.0, 3.0, points_per_lap=12), origin,
                                     str(tmp_path / "orbit.py"))
    compile(open(p).read(), p, "exec")
    lat, lon = missions.enu_to_ll(*missions.ll_to_enu(40.0001, -83.0002, origin), origin)
    assert abs(lat - 40.0001) < 1e-9 and abs(lon + 83.0002) < 1e-9
    try:
        missions.write_olympe_script(missions.orbit((0, 0), 5.0, 150.0), origin, str(tmp_path / "x.py"))
        assert False, "should refuse 150 m"
    except ValueError:
        pass
