# Splat simulator pipeline (CPU)

Builds a Gaussian splat of the flying area from ANAFI video and serves a drone sim on top of it.

1. Extract frames at 1 fps, 1280 px (`ffmpeg -vf "fps=1,scale=1280:-2" -frame_pts 1 images/<video>_%06d.jpg`) and the telemetry CSVs (`bench/anafi_meta.py`).
2. `python prep.py` matches each frame to its telemetry row (ENU position, camera orientation) and builds a GPS-guided image pair list.
3. `python sfm.py --fresh` runs COLMAP (pycolmap, CPU): SIFT, matching on the pair list, incremental mapping.
4. `python train_gs.py` aligns COLMAP to GPS (similarity fit), trains a small Gaussian splat on CPU, and exports `out/scene.splat`, `out/occupancy.json` and `out/scene_meta.json`.
   Env: `ITERS`, `KMAX` (compositing chunk), `MAXN`, `DENSIFY_AT`, `RESUME=1` to continue from `out/ckpt.pt`.
5. `python make_track.py` writes the recorded flight as `site/track.json`.
6. Serve `site/` with `scene.splat`, `occupancy.json`, `scene_meta.json`, `track.json` beside `index.html`.

The sim page uses three.js and GaussianSplats3D. The drone model takes velocity or attitude commands (like ANAFI PCMD), with first-order attitude and climb response, tilt-driven horizontal acceleration and linear drag.
