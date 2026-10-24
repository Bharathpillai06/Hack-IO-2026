# Sim bench (no hardware)

## Setup
```
pip install flask gevent pymitter cflib psutil
```

## Run the baseline
From the repo root:
```
python bench/run_bench.py --n 1 5 10 20 40 --secs 15 --label baseline
```
Each N starts a fresh server, connects N `mock://i-circle` drones over the real HTTP API, and runs `godot_emulator.py`, which reproduces what `TouchObject.gd` does with telemetry. Results are appended to `bench/results.csv` and the server log goes to `bench/server.log`.

If the server fails with "Address family not supported", set `TWINFLIE_HTTP_HOST=127.0.0.1`.

## Fly mocks in the Godot client
Start `python main.py` in `python server/classe refactoring`. Open `godot client` in **Godot 3.5**. The paper says Godot 4, but the project is Godot 3. Add a drone with the URL `mock:__0` (underscores stand for slashes). Use `mock:__0-circle` for one that flies a circle by itself.

## Metrics
| column | meaning |
|---|---|
| transport_p50/p95_ms | packet arrival minus when the sample was produced |
| age_p50/p95_ms | how old the position each drone node is holding is, sampled every frame |
| rate_per_drone_hz | updates each drone node applied per second |
| rx_kbps / server_tx_kbps | telemetry bandwidth |
| frame_cost_ms | telemetry work per 60 fps frame |
| twin_err_ghost | distance between the ghost the user sees and the true position (circle drones) |
| twin_err_latest | distance between the latest packet and the true position (data staleness only) |
| server_cpu_pct | CPU used by the server process during the run |

## Files changed
- `classes/uav/MockUAV.py` and `mock_patterns.py` are new
- `drones_collection.py` picks `MockUAV` for `mock://` URLs, adds a `t` timestamp per drone, and returns fake anchors for mocks
- `main.py` adds `/stats` and `/stats/reset` (UDP tx counters) and a configurable HTTP bind host
- `UAV_ciccio.py` drops unused imports (`turtle` pulled in tkinter)
