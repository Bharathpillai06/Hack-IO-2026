# Hack-IO-2026

A quick simulator for drone missions, built from drone video of the place you want to fly.

Record a video of an area, and Hack-IO-2026 rebuilds it as a 3D Gaussian splat in real meters, drops a simple 6-DOF drone model into it, and lets you fly missions with velocity or attitude commands. With an ANAFI, it also replays the real flight from the drone's own GPS log, so you can compare the simulated drone against the real one.

Demo scene: Tuttle Park, Columbus OH, rebuilt from two ANAFI flights on 24 Oct 2026.

## What it does

| Part | What you get |
|---|---|
| Video to 3D world | `splat/build_scene.py`: frames, camera poses (COLMAP), Gaussian splat training on CPU. Works for any location. |
| Real-world scale | ANAFI GPS read straight from the MP4 metadata (camera path fits GPS to 0.67 m median at Tuttle Park), or a printed ArUco tag for any other camera. |
| Drone model | Position + roll, pitch, yaw. Tilt-driven acceleration, first-order attitude and climb response, drag, 25° tilt and speed limits. |
| Control | Velocity mode (x/y/z velocity + yaw rate), attitude mode (roll, pitch, yaw rate, climb; same inputs as ANAFI manual control), and an autopilot that flies the recorded GPS track and reports tracking error. |
| Views | Chase, FPV (drone camera inside the splat), free orbit. |
| Collision warning | Occupancy grid built from the splat's opaque Gaussians. |
| Desktop app | `HackIO-Sim.exe` runs offline and lists every scene you build. |
| Real drone link | `AnafiUAV` drives a Parrot ANAFI through Olympe; `MockUAV` and a benchmark measure the digital-twin telemetry path. |

## Quick start (Windows)

1. Build a scene: drag your videos onto `build_scene.bat` (first run installs the tools, about 1 GB).
2. Build the app: double-click `build_exe.bat`, then run `dist\HackIO-Sim.exe`.
3. Fly: `W/S` forward/back, `A/D` left/right, `R/F` climb/descend, `Q/E` yaw, `T` take off, `C` change camera.

Linux/macOS: `python splat/build_scene.py video.mp4 --name my-field`, then `python splat/app/sim_launcher.py`.

## Capturing a good scene

Fly or walk a full slow circle around the area with the camera aimed at the center, at two heights, in steady light, with nothing moving through the shot. Lock exposure and keep the zoom at 1x. Add a printed ArUco tag (DICT_4X4_50, id 0) if the camera has no GPS.

## Layout

- `splat/`: scene builder (`build_scene.py`, `mp4meta.py`, `train_gs.py`), sim page (`site/`), desktop launcher (`app/`)
- `scenes/`: built scenes (Tuttle Park included)
- `bench/`: ANAFI telemetry extractor, ANAFI ground check, digital-twin latency benchmark
- `python server/`, `godot client/`: the Twinflie orchestrator this project extends, with `MockUAV` and `AnafiUAV` added

## Built on

- [Twinflie](https://github.com/UniCT-ARSLab/Twinflie) (Conti, Santoro, Santoro, IEEE DASC 2023), the digital-twin UAV orchestrator this repo started from. Its original README is in `docs/TWINFLIE_README.md`.
- FiGS / SOUS VIDE (Stanford MSL) for the video-to-splat-to-drone-sim approach, and Splat-Nav for splat-based collision ideas.
- [COLMAP](https://colmap.github.io/) via pycolmap, [three.js](https://threejs.org/), [GaussianSplats3D](https://github.com/mkkellogg/GaussianSplats3D), Parrot [Olympe](https://github.com/Parrot-Developers/olympe).
