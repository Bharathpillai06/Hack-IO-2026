"""Turn drone or phone video of ANY location into a scene for the flight sim.

  python splat/build_scene.py my_field.mp4 [more.mp4 ...] --name my-field
  (Windows: drag videos onto build_scene.bat)
  python splat/build_scene.py walk.mov --name lab --tag-size 0.18   # phone video + printed ArUco tag

Output: scenes/<name>/ with scene_splat.b64.txt, scene_meta.json, occupancy.json, track.json
(the sim launcher and the web page load these). Work files go in scenes/<name>/work/.

How the scene gets real-world scale and "up":
  1. ANAFI video  -> per-frame GPS + height in the MP4 metadata (automatic)
  2. --tag-size   -> a printed ArUco tag (DICT_4X4_50, id 0) lying flat on the ground
  3. neither      -> up = average camera up, ground = lowest 5% of points, units arbitrary
Steps are cached: rerun with the same --name to resume (delete work/ to start over).
Needs: pip install pycolmap torch numpy pillow scipy imageio-ffmpeg  (opencv-python for --tag-size)
"""
import argparse, glob, json, math, os, shutil, subprocess, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from mp4meta import read_telemetry  # noqa: E402

EARTH_R = 6378137.0


def log(*a):
    print(time.strftime("[%H:%M:%S]"), *a, flush=True)


def ffmpeg_exe():
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        exe = shutil.which("ffmpeg")
        if not exe:
            sys.exit("ffmpeg not found: pip install imageio-ffmpeg")
        return exe


def umeyama(X, Y):
    mx, my = X.mean(0), Y.mean(0)
    Xc, Yc = X - mx, Y - my
    U, S, Vt = np.linalg.svd(Yc.T @ Xc / len(X))
    D = np.eye(3); D[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ D @ Vt
    s = np.trace(np.diag(S) @ D) / (Xc ** 2).sum(1).mean()
    return s, R, my - s * R @ mx


# ------------------------------------------------------------------ 1. frames + telemetry
def extract(videos, work, fps, width):
    img_dir = os.path.join(work, "images")
    if os.path.isdir(img_dir) and os.listdir(img_dir):
        log("frames: cached"); return
    os.makedirs(img_dir, exist_ok=True)
    ff = ffmpeg_exe()
    for i, v in enumerate(videos):
        tag = f"v{i}"
        subprocess.run([ff, "-v", "error", "-i", v, "-vf", f"fps={fps},scale={width}:-2", "-q:v", "2",
                        "-frame_pts", "1", os.path.join(img_dir, f"{tag}_%06d.jpg")], check=True)
    log("frames:", len(os.listdir(img_dir)))


def frame_telemetry(videos, work, fps):
    path = os.path.join(work, "telemetry.json")
    if os.path.exists(path):
        return json.load(open(path))
    per = {}
    origin = None
    for i, v in enumerate(videos):
        rows = [r for r in read_telemetry(v) if r["gps_ok"]]
        log(f"{os.path.basename(v)}: {len(rows)} telemetry rows" + ("" if rows else " (no ANAFI metadata)"))
        if not rows:
            continue
        if origin is None:
            origin = {"lat": rows[0]["lat"], "lon": rows[0]["lon"]}
        for name in sorted(glob.glob(os.path.join(work, "images", f"v{i}_*.jpg"))):
            base = os.path.basename(name)
            t = int(base[:-4].split("_")[1]) / fps
            r = min(rows, key=lambda x: abs(x["t_video_s"] - t))
            e = math.radians(r["lon"] - origin["lon"]) * EARTH_R * math.cos(math.radians(origin["lat"]))
            n = math.radians(r["lat"] - origin["lat"]) * EARTH_R
            per[base] = {"enu": [e, n, r["ground_distance_m"]], "yaw": r["cam_yaw_deg"],
                         "hfov": r["hfov_deg"], "t": t, "video": i}
    out = {"origin": origin, "frames": per}
    json.dump(out, open(path, "w"))
    return out


# ------------------------------------------------------------------ 2. pairs + SfM
def make_pairs(work, tele):
    names = sorted(os.listdir(os.path.join(work, "images")))
    fr = tele["frames"]
    pairs = set()
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            va, vb = a.split("_")[0], b.split("_")[0]
            ia, ib = int(a[:-4].split("_")[1]), int(b[:-4].split("_")[1])
            if va == vb and abs(ia - ib) <= 6:
                pairs.add((a, b)); continue
            if a in fr and b in fr:
                d = math.dist(fr[a]["enu"][:2], fr[b]["enu"][:2])
                dyaw = abs((fr[a]["yaw"] - fr[b]["yaw"] + 180) % 360 - 180)
                if d < 6 and dyaw < 45:
                    pairs.add((a, b))
            elif (ia % 4 == 0 and ib % 4 == 0):          # no GPS: sparse all-to-all for loop closure
                pairs.add((a, b))
    with open(os.path.join(work, "pairs.txt"), "w") as f:
        for a, b in sorted(pairs):
            f.write(f"{a} {b}\n")
    log("image pairs:", len(pairs))


def sfm(work, tele, width):
    import pycolmap as p
    sparse = os.path.join(work, "colmap", "sparse", "0")
    if os.path.exists(os.path.join(sparse, "images.bin")):
        log("camera poses: cached"); return
    db = os.path.join(work, "colmap", "database.db")
    os.makedirs(os.path.dirname(db), exist_ok=True)
    if os.path.exists(db):
        os.remove(db)
    reader = p.ImageReaderOptions(); reader.camera_model = "SIMPLE_RADIAL"
    hf = next((v["hfov"] for v in tele["frames"].values()), None)
    if hf:   # ANAFI reports its field of view per frame
        h = round(width * 9 / 16)
        reader.camera_params = f"{(width / 2) / math.tan(math.radians(hf) / 2):.1f},{width / 2},{h / 2},0"
    ext = p.FeatureExtractionOptions(); ext.num_threads = -1
    ext.sift.max_num_features = 6000; ext.sift.peak_threshold = 0.004
    log("camera poses: extracting features")
    p.extract_features(db, os.path.join(work, "images"), camera_mode=p.CameraMode.SINGLE,
                       reader_options=reader, extraction_options=ext, device=p.Device.auto)
    log("camera poses: matching")
    po = p.ImportedPairingOptions(); po.match_list_path = os.path.join(work, "pairs.txt")
    p.match_image_pairs(db, pairing_options=po, device=p.Device.auto)
    log("camera poses: solving")
    out = os.path.join(work, "colmap", "sparse")
    os.makedirs(out, exist_ok=True)
    opts = p.IncrementalPipelineOptions(); opts.mapper.init_min_tri_angle = 4.0
    recs = p.incremental_mapping(db, os.path.join(work, "images"), out, options=opts)
    if not recs:
        sys.exit("COLMAP could not solve camera poses. Use slower, sharper, more textured footage.")
    best = max(recs, key=lambda k: recs[k].num_reg_images())
    if best != 0:   # keep the largest model as sparse/0
        shutil.rmtree(sparse, ignore_errors=True)
        shutil.move(os.path.join(out, str(best)), sparse)
    log(f"camera poses: {recs[best].num_reg_images()} images, {recs[best].num_points3D()} points")


# ------------------------------------------------------------------ 3. alignment to a metric, z-up world
def align(work, tele, tag_size, name):
    import pycolmap as p
    rec = p.Reconstruction(os.path.join(work, "colmap", "sparse", "0"))
    cams = {}
    for im in rec.images.values():
        if im.has_pose:
            T = im.cam_from_world().matrix() if callable(im.cam_from_world) else im.cam_from_world.matrix()
            R, t = np.array(T[:, :3]), np.array(T[:, 3])
            cams[im.name] = (R, t, -R.T @ t)
    fr = tele["frames"]
    gps = [n for n in cams if n in fr]
    result = {"name": name, "origin": tele.get("origin")}
    if len(gps) >= 5:
        C = np.array([cams[n][2] for n in gps]); G = np.array([fr[n]["enu"] for n in gps])
        s, R, t = umeyama(C, G)
        r = np.linalg.norm((s * (R @ C.T)).T + t - G, axis=1)
        result.update(method="gps", units="m", residual_median_m=float(np.median(r)))
        log(f"align: GPS, median camera-vs-GPS error {np.median(r):.2f} m")
    elif tag_size:
        s, R, t, med = align_aruco(work, cams, rec, tag_size)
        result.update(method="aruco", units="m", residual_median_m=med)
    else:
        ups = np.array([-c[0][1] for c in cams.values()])      # camera -y axis in COLMAP world = image up
        up = ups.mean(0); up /= np.linalg.norm(up)
        z = up; x = np.cross([0, 1, 0] if abs(up[1]) < 0.9 else [1, 0, 0], z); x /= np.linalg.norm(x)
        y = np.cross(z, x)
        R = np.stack([x, y, z]); s = 1.0
        P = np.array([pt.xyz for pt in rec.points3D.values()]) @ R.T
        t = -np.array([P[:, 0].mean(), P[:, 1].mean(), np.percentile(P[:, 2], 5)])
        result.update(method="camera-up", units="arbitrary")
        log("align: no GPS or tag, using camera up direction; units are arbitrary")
    result.update(scale=float(s), R=np.asarray(R).tolist(), t=np.asarray(t).tolist())
    json.dump(result, open(os.path.join(work, "align.json"), "w"), indent=1)
    return result, cams


def align_aruco(work, cams, rec, tag_size):
    import cv2
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                  cv2.aruco.DetectorParameters())
    h = tag_size / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)
    C_col, C_tag = [], []
    for name, (R, t, C) in cams.items():
        im = cv2.imread(os.path.join(work, "images", name), cv2.IMREAD_GRAYSCALE)
        corners, ids, _ = det.detectMarkers(im)
        if ids is None or 0 not in ids.ravel():
            continue
        cam = rec.cameras[next(i.camera_id for i in rec.images.values() if i.name == name)]
        f, cx, cy = cam.params[:3]
        K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]])
        ok, rv, tv = cv2.solvePnP(obj, corners[list(ids.ravel()).index(0)][0], K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            Rt, _ = cv2.Rodrigues(rv)
            C_tag.append((-Rt.T @ tv).ravel()); C_col.append(C)
    if len(C_col) < 3:
        sys.exit(f"ArUco tag (DICT_4X4_50 id 0) seen in only {len(C_col)} frames; need 3+.")
    s, R, t = umeyama(np.array(C_col), np.array(C_tag))   # tag frame: z = tag normal = up
    r = np.linalg.norm((s * (R @ np.array(C_col).T)).T + t - np.array(C_tag), axis=1)
    log(f"align: ArUco tag in {len(C_col)} frames, median error {np.median(r):.2f} m")
    return s, R, t, float(np.median(r))


# ------------------------------------------------------------------ 4. train + export
def train(work, iters):
    if os.path.exists(os.path.join(work, "out", "scene.splat")):
        log("splat: cached"); return
    env = dict(os.environ, ITERS=str(iters), KMAX="128", MAXN="90000",
               DENSIFY_AT=",".join(str(int(iters * f)) for f in (0.15, 0.35, 0.55)))
    log(f"splat: training {iters} steps (CPU: about 1.5 s per step)")
    subprocess.run([sys.executable, os.path.join(HERE, "train_gs.py")], cwd=work, env=env, check=True)


def export(work, scene_dir, tele, al, cams):
    import base64
    out = os.path.join(work, "out")
    raw = open(os.path.join(out, "scene.splat"), "rb").read()
    open(os.path.join(scene_dir, "scene_splat.b64.txt"), "w").write(base64.b64encode(raw).decode())
    for f in ("scene_meta.json", "occupancy.json"):
        shutil.copy(os.path.join(out, f), scene_dir)
    # recorded path: GPS track if we have one, otherwise the solved camera path
    fr = tele["frames"]
    track = {"t": [], "enu": [], "yaw": []}
    if fr:
        items = sorted(fr.values(), key=lambda v: (v["video"], v["t"]))
        items = [v for v in items if v["video"] == items[0]["video"]]
        for v in items:
            track["t"].append(round(v["t"] - items[0]["t"], 2)); track["enu"].append([round(x, 3) for x in v["enu"]])
            track["yaw"].append(round(v["yaw"] % 360, 1))
    else:
        s, R, t = al["scale"], np.array(al["R"]), np.array(al["t"])
        for i, name in enumerate(sorted(cams)):
            Rc, tc, C = cams[name]
            p = s * R @ C + t
            fwd = R @ (Rc.T @ np.array([0, 0, 1.0]))
            track["t"].append(float(i)); track["enu"].append([round(float(x), 3) for x in p])
            track["yaw"].append(round(math.degrees(math.atan2(fwd[0], fwd[1])) % 360, 1))
    json.dump(track, open(os.path.join(scene_dir, "track.json"), "w"))
    log(f"scene ready: {scene_dir}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("videos", nargs="+")
    ap.add_argument("--name", help="scene name, e.g. tuttle-park (default: first video's file name)")
    ap.add_argument("--scenes", default=os.path.join(os.path.dirname(HERE), "scenes"))
    ap.add_argument("--fps", type=float, default=1.0, help="frames per second to sample (1 = 1 frame/s)")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--iters", type=int, default=1200)
    ap.add_argument("--tag-size", type=float, help="edge of the printed ArUco tag in meters, e.g. 0.18")
    a = ap.parse_args()
    a.name = a.name or os.path.splitext(os.path.basename(a.videos[0]))[0].lower().replace(" ", "-")
    scene_dir = os.path.join(a.scenes, a.name)
    work = os.path.join(scene_dir, "work")
    os.makedirs(work, exist_ok=True)
    videos = [os.path.abspath(v) for v in a.videos]
    extract(videos, work, a.fps, a.width)
    tele = frame_telemetry(videos, work, a.fps)
    make_pairs(work, tele)
    sfm(work, tele, a.width)
    al, cams = align(work, tele, a.tag_size, a.name)
    train(work, a.iters)
    export(work, scene_dir, tele, al, cams)


if __name__ == "__main__":
    main()
