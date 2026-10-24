import os, shutil, sys, time
import pycolmap as p

os.makedirs("colmap", exist_ok=True)
db = "colmap/database.db"
if "--fresh" in sys.argv and os.path.exists(db):
    os.remove(db)
t0 = time.time()
reader = p.ImageReaderOptions()
reader.camera_model = "SIMPLE_RADIAL"
# ANAFI 4K video at 1280x720: HFOV 69.0 deg -> f = 640/tan(34.5 deg) = 931.2 px
reader.camera_params = "931.2,640,360,0"
ext = p.FeatureExtractionOptions()
ext.num_threads = 2
ext.sift.max_num_features = 6000
ext.sift.peak_threshold = 0.004      # grass is low contrast; keep more features
if not os.path.exists(db):
    p.extract_features(db, "splatprep/images", camera_mode=p.CameraMode.SINGLE,
                       reader_options=reader, extraction_options=ext, device=p.Device.cpu)
    print(f"features done {time.time()-t0:.0f}s", flush=True)
    m = p.FeatureMatchingOptions(); m.num_threads = 2
    po = p.ImportedPairingOptions(); po.match_list_path = "pairs.txt"
    p.match_image_pairs(db, matching_options=m, pairing_options=po, device=p.Device.cpu)
    print(f"matching done {time.time()-t0:.0f}s", flush=True)
shutil.rmtree("colmap/sparse", ignore_errors=True)
os.makedirs("colmap/sparse")
opts = p.IncrementalPipelineOptions()
opts.num_threads = 2
opts.mapper.init_min_tri_angle = 4.0     # forward motion: allow smaller baselines
recs = p.incremental_mapping(db, "splatprep/images", "colmap/sparse", options=opts)
for k, r in recs.items():
    print(k, r.summary(), flush=True)
print(f"total {time.time()-t0:.0f}s")
