"""Baseline benchmark: Twinflie server + N mock drones + emulated Godot client.

Usage (from the repo root):
  python bench/run_bench.py                    # N = 1 5 10 20, 15 s each
  python bench/run_bench.py --n 1 5 --secs 10 --label baseline

For each N it starts a fresh server, connects N mock://i-circle drones over the
real HTTP API, runs the emulator, and reads server-side tx bytes from /stats.
Results are appended to bench/results.csv.
"""
import argparse
import csv
import json
import os
import subprocess
import sys
import time
import urllib.request

import psutil

HERE = os.path.dirname(os.path.abspath(__file__))
SERVER_DIR = os.path.join(HERE, "..", "python server", "classe refactoring")
sys.path.insert(0, HERE)
import godot_emulator  # noqa: E402

HTTP = "http://127.0.0.1:5000"


# Ignore http_proxy/https_proxy env vars: campus machines often set them, and urllib
# would send 127.0.0.1 requests to the proxy, so the server looks like it never came up.
_OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def get(path, timeout=10):
    with _OPENER.open(HTTP + path, timeout=timeout) as r:
        return r.read().decode()


def wait_http(proc, timeout=45):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if proc.poll() is not None:
            raise RuntimeError("server exited early, see bench/server.log")
        try:
            get("/stats", timeout=1)
            return
        except Exception:
            time.sleep(0.2)
    raise RuntimeError("server did not come up")


def one_run(n, secs, label):
    log = open(os.path.join(HERE, "server.log"), "w")
    proc = subprocess.Popen([sys.executable, "-u", "main.py"], cwd=SERVER_DIR,
                            stdout=log, stderr=subprocess.STDOUT)
    try:
        wait_http(proc)
        names = [f"mock://{i}-circle" for i in range(n)]
        for name in names:
            r = get("/connect_to/url/" + name.replace("/", "_"))
            if "ok" not in r:
                raise RuntimeError(f"connect {name}: {r}")
        p = psutil.Process(proc.pid)
        p.cpu_percent(None)
        get("/stats/reset")
        res = godot_emulator.run(names, secs)
        cpu = p.cpu_percent(None)
        st = json.loads(get("/stats"))
        res["server_tx_kbps"] = st["bytes"] * 8 / 1000.0 / st["elapsed"]
        res["server_cpu_pct"] = cpu
        res["label"] = label
        return res
    finally:
        proc.terminate()
        try:
            proc.wait(5)
        except subprocess.TimeoutExpired:
            proc.kill()
        log.close()
        time.sleep(0.5)   # let UDP ports free up


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, nargs="+", default=[1, 5, 10, 20])
    ap.add_argument("--secs", type=float, default=15)
    ap.add_argument("--label", default="baseline")
    a = ap.parse_args()

    out = os.path.join(HERE, "results.csv")
    rows = []
    for n in a.n:
        print(f"[{a.label}] N={n} ...", flush=True)
        r = one_run(n, a.secs, a.label)
        rows.append(r)
        print("   " + ", ".join(f"{k}={v:.2f}" if isinstance(v, float) else f"{k}={v}"
                                for k, v in r.items()), flush=True)

    keys = ["label", "drones"] + [k for k in rows[0] if k not in ("label", "drones")]
    new = not os.path.exists(out)
    with open(out, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        if new:
            w.writeheader()
        w.writerows(rows)
    print("wrote", out)


if __name__ == "__main__":
    main()
