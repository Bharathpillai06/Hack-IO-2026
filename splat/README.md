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

## Any location, one command
`splat/build_scene.py` runs the whole pipeline on video from anywhere:

```
python splat/build_scene.py clip1.mp4 clip2.mp4 --name my-field        # ANAFI: GPS scale and up, automatic
python splat/build_scene.py walkaround.mov --name lab --tag-size 0.18   # any camera + printed ArUco tag
python splat/build_scene.py anything.mp4 --name test                    # no GPS/tag: arbitrary units
```
On Windows, drag videos onto `build_scene.bat`. The scene lands in `scenes/<name>/`. Steps are cached in `scenes/<name>/work/`, so rerunning resumes.
For a good scene, fly or walk a full circle around the area with the camera aimed at the center, slowly, in steady light.

## Desktop app
`build_exe.bat` builds `dist/HackIO-Sim.exe` (PyInstaller). The exe serves the sim offline (three.js and the splat viewer are bundled in `splat/app/web/vendor`) and opens it in your browser. Scenes live in `scenes/` next to the exe; pick one from the Scene menu. `run_sim.bat` runs the same thing from Python without building.
