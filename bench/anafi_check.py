"""Ground check for the ANAFI: connects, listens to telemetry, never flies.

Usage (laptop joined to the drone's Wi-Fi, drone powered on, props can stay off):
  python bench/anafi_check.py                 # 192.168.42.1, 20 s
  python bench/anafi_check.py --ip 192.168.53.1 --secs 30

Prints how often each telemetry event really arrives. That rate is the ceiling
for how fresh the digital twin can be, whatever we do to the pipeline.
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "python server", "classe refactoring"))

from classes.uav.AnafiUAV import AnafiUAV  # noqa: E402
from classes.uav.UAVEvents import UAVEvents  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.42.1")
    ap.add_argument("--secs", type=float, default=20)
    a = ap.parse_args()

    uav = AnafiUAV("anafi://" + a.ip)
    failed = []
    uav.events.on(UAVEvents.CONNECTION_FAILED, lambda u, e: failed.append(e))
    print(f"connecting to {a.ip} ...", flush=True)
    uav.connect()
    t0 = time.time()
    while not uav.connected and time.time() - t0 < 30:
        time.sleep(0.1)
    if failed or not uav.connected:
        print("could not connect. Is this laptop on the drone's ANAFI-xxxx Wi-Fi?")
        sys.exit(1)

    print(f"connected, listening for {a.secs:.0f} s (drone stays on the ground)")
    start = dict(uav.counts)
    t0 = time.time()
    while time.time() - t0 < a.secs:
        time.sleep(2)
        print(f"  t={time.time()-t0:4.0f}s  pos=({uav.x:6.2f},{uav.y:6.2f},{uav.z:5.2f}) m"
              f"  gps_origin={'yes' if uav.origin else 'no'}"
              f"  battery={uav.battery_percent}%  state={uav.flying_state}", flush=True)
    el = time.time() - t0

    print("\nevent rates (Hz):")
    for k, v in uav.counts.items():
        print(f"  {k:8s} {(v - start[k]) / el:6.2f}")
    if not uav.origin:
        print("\nno GPS fix: go_to will use relative moves (drift). Fly outdoors with sky view.")
    uav.disconnect()


if __name__ == "__main__":
    main()
