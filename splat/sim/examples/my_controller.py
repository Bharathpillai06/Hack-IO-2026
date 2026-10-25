"""Template: test your own controller in the loop. Edit control() and run it.

The sim calls control() every 10 ms with the drone state; return a velocity command.
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from hackio_sim import Sim  # noqa: E402

TARGET = (8.0, 12.0, 4.0)   # meters east, north, up


def control(s):
    """Simple proportional controller. Replace with your own."""
    k = 0.8
    return {"vx": k * (TARGET[0] - s["x"]), "vy": k * (TARGET[1] - s["y"]), "vz": k * (TARGET[2] - s["z"]), "yaw_rate": 0}


sim = Sim("tuttle-park", viz=sys.argv[sys.argv.index("--viz") + 1] if "--viz" in sys.argv else None)
sim.takeoff(1.0)
for _ in range(1500):                     # 15 s
    cmd = control(sim.state())
    sim.velocity(cmd["vx"], cmd["vy"], cmd["vz"], cmd["yaw_rate"], seconds=sim.dt)
s = sim.state()
err = ((s["x"] - TARGET[0]) ** 2 + (s["y"] - TARGET[1]) ** 2 + (s["z"] - TARGET[2]) ** 2) ** 0.5
print(f"final error {err:.2f} m, near obstacle: {s['near_obstacle']}, collisions: {sim.collisions}")
