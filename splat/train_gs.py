"""Minimal CPU Gaussian splatting trainer (PyTorch, tile-based alpha compositing).

Input : colmap/sparse/0 (pycolmap), splatprep/images, telemetry.json
Output: out/scene.splat (antimatter15 .splat, three.js frame: x=E, y=Up, z=-N),
        out/align.json (COLMAP -> ENU similarity), out/preview_*.png

Kept deliberately small: DC color only, L1 loss, simple clone/split/prune.
Runs at low resolution so it trains on a 2-core CPU.
"""
import json, math, os, sys, time
import numpy as np
import torch
import pycolmap
from PIL import Image
from scipy.spatial import cKDTree

torch.set_num_threads(2)
torch.manual_seed(0)
ITERS = int(os.environ.get("ITERS", 1500))
DOWN = int(os.environ.get("DOWN", 4))          # 1280x720 -> 320x180
TILE = 16
KMAX = int(os.environ.get("KMAX", 192))
MAXN = int(os.environ.get("MAXN", 120000))
DENSIFY_AT = tuple(int(x) for x in os.environ.get("DENSIFY_AT", "250,500,750,1000,1250").split(","))
os.makedirs("out", exist_ok=True)

# ---------------------------------------------------------------- data
rec = pycolmap.Reconstruction("colmap/sparse/0")
tele = json.load(open("telemetry.json"))["frames"]
imgs = []
for im in rec.images.values():
    if not im.has_pose:
        continue
    cam = rec.cameras[im.camera_id]
    f, cx, cy = cam.params[0], cam.params[1], cam.params[2]
    T = im.cam_from_world().matrix() if callable(im.cam_from_world) else im.cam_from_world.matrix()
    imgs.append({"name": im.name, "R": np.array(T[:, :3]), "t": np.array(T[:, 3]),
                 "f": f, "cx": cx, "cy": cy, "w": cam.width, "h": cam.height})
imgs.sort(key=lambda d: d["name"])
print(len(imgs), "registered images,", len(rec.points3D), "points", flush=True)

# --- similarity COLMAP -> ENU (x east, y north, z = height above lawn)
C = np.array([-d["R"].T @ d["t"] for d in imgs])
G = np.array([[tele[d["name"]]["enu"][0], tele[d["name"]]["enu"][1],
               tele[d["name"]]["ground_distance"]] for d in imgs])
def umeyama(X, Y):
    mx, my = X.mean(0), Y.mean(0)
    Xc, Yc = X - mx, Y - my
    U, S, Vt = np.linalg.svd(Yc.T @ Xc / len(X))
    D = np.eye(3); D[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ D @ Vt
    s = np.trace(np.diag(S) @ D) / (Xc ** 2).sum(1).mean()
    return s, R, my - s * R @ mx
s, Ra, ta = umeyama(C, G)
res = np.linalg.norm((s * (Ra @ C.T)).T + ta - G, axis=1)
print(f"align: scale {s:.3f}, camera-vs-GPS residual median {np.median(res):.2f} m, max {res.max():.2f} m", flush=True)
json.dump({"scale": s, "R": Ra.tolist(), "t": ta.tolist(), "residual_median_m": float(np.median(res)),
           "residual_max_m": float(res.max())}, open("out/align.json", "w"), indent=1)

# express cameras in ENU: X_enu = s Ra X + ta  ->  X = Ra^T (X_enu - ta)/s
for d in imgs:
    R_new = d["R"] @ Ra.T
    t_new = s * d["t"] - R_new @ ta
    d["R"], d["t"] = R_new, t_new          # cam = R_new X_enu + t_new (camera units = meters)
    sc = 1.0 / DOWN
    img = Image.open(f"splatprep/images/{d['name']}").convert("RGB")
    W, H = int(round(d["w"] * sc)), int(round(d["h"] * sc))
    d["img"] = torch.from_numpy(np.asarray(img.resize((W, H), Image.LANCZOS))).float() / 255.0
    d["W"], d["H"] = W, H
    d["f"], d["cx"], d["cy"] = d["f"] * sc, d["cx"] * sc, d["cy"] * sc
    d["Rt"] = torch.tensor(d["R"], dtype=torch.float32)
    d["tt"] = torch.tensor(d["t"], dtype=torch.float32)

# ---------------------------------------------------------------- init gaussians
P = np.array([p.xyz for p in rec.points3D.values()])
RGB = np.array([p.color for p in rec.points3D.values()]) / 255.0
P = (s * (Ra @ P.T)).T + ta
keep = (np.abs(P[:, 0]) < 400) & (np.abs(P[:, 1]) < 400) & (P[:, 2] > -3) & (P[:, 2] < 60)
P, RGB = P[keep], RGB[keep]
camc = np.array([-d["R"].T @ d["t"] for d in imgs])
lo, hi = camc.min(0) - 25, camc.max(0) + 25
# seed the lawn: the footage is mostly grass, which has few sparse points
ng = 15000
gp = np.stack([np.random.uniform(lo[0], hi[0], ng), np.random.uniform(lo[1], hi[1], ng), np.zeros(ng)], 1)
P = np.concatenate([P, gp]); RGB = np.concatenate([RGB, np.tile([[0.25, 0.4, 0.2]], (ng, 1))])
dist, _ = cKDTree(P).query(P, k=4)
sc0 = np.clip(dist[:, 1:].mean(1), 0.02, 3.0)
N = len(P)
print("init gaussians:", N, flush=True)

def inv_sigmoid(x): return math.log(x / (1 - x))
means = torch.tensor(P, dtype=torch.float32, requires_grad=True)
log_s = torch.tensor(np.log(np.stack([sc0] * 3, 1)), dtype=torch.float32, requires_grad=True)
quats = torch.zeros(N, 4); quats[:, 0] = 1; quats.requires_grad_(True)
opac = torch.full((N,), inv_sigmoid(0.3), requires_grad=True)
col = torch.tensor(np.log(np.clip(RGB, 0.02, 0.98) / (1 - np.clip(RGB, 0.02, 0.98))), dtype=torch.float32, requires_grad=True)
sky = torch.tensor([[0.8, 0.85, 0.95], [0.55, 0.65, 0.75]], requires_grad=True)   # horizon, zenith

def make_opt():
    return torch.optim.Adam([
        {"params": [means], "lr": 0.004}, {"params": [log_s], "lr": 0.005},
        {"params": [quats], "lr": 0.001}, {"params": [opac], "lr": 0.05},
        {"params": [col], "lr": 0.02}, {"params": [sky], "lr": 0.01}])
if os.environ.get("RESUME"):
    ck = torch.load("out/ckpt.pt")
    means, log_s, quats, opac, col, sky = [ck[k].detach().requires_grad_(True) for k in ("means", "log_s", "quats", "opac", "col", "sky")]
    print("resumed from checkpoint with", len(means), "gaussians", flush=True)
opt = make_opt()

# ---------------------------------------------------------------- renderer
def quat_to_rot(q):
    q = q / q.norm(dim=1, keepdim=True)
    w, x, y, z = q.unbind(1)
    return torch.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
                        2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
                        2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], 1).view(-1, 3, 3)

def render(d, want_grad2d=False):
    W, H, f, cx, cy = d["W"], d["H"], d["f"], d["cx"], d["cy"]
    Xc = means @ d["Rt"].T + d["tt"]
    z = Xc[:, 2]
    vis = z > 0.3
    idx = vis.nonzero().squeeze(1)
    Xc, z = Xc[idx], z[idx]
    u = f * Xc[:, 0] / z + cx
    v = f * Xc[:, 1] / z + cy
    mean2d = torch.stack([u, v], 1)
    if want_grad2d:
        mean2d.retain_grad()
    Rq = quat_to_rot(quats[idx])
    S = torch.exp(log_s[idx])
    M = Rq * S[:, None, :]
    cov3 = M @ M.transpose(1, 2)
    Wc = d["Rt"]
    J = torch.zeros(len(idx), 2, 3)
    J[:, 0, 0] = f / z; J[:, 0, 2] = -f * Xc[:, 0] / z ** 2
    J[:, 1, 1] = f / z; J[:, 1, 2] = -f * Xc[:, 1] / z ** 2
    T = J @ Wc
    cov2 = T @ cov3 @ T.transpose(1, 2)
    a = cov2[:, 0, 0] + 0.3; b = cov2[:, 0, 1]; c = cov2[:, 1, 1] + 0.3
    det = (a * c - b * b).clamp(min=1e-6)
    conic = torch.stack([c / det, -b / det, a / det], 1)
    mid = 0.5 * (a + c)
    lam = mid + torch.sqrt((mid * mid - det).clamp(min=0.01))
    rad = (3 * torch.sqrt(lam)).detach().clamp(max=140)
    ud, vd = u.detach(), v.detach()
    on = (ud + rad > 0) & (ud - rad < W) & (vd + rad > 0) & (vd - rad < H) & (rad > 0.3)
    sel = on.nonzero().squeeze(1)
    ntx, nty = math.ceil(W / TILE), math.ceil(H / TILE)
    x0 = ((ud[sel] - rad[sel]) / TILE).floor().clamp(0, ntx - 1).long()
    x1 = ((ud[sel] + rad[sel]) / TILE).floor().clamp(0, ntx - 1).long()
    y0 = ((vd[sel] - rad[sel]) / TILE).floor().clamp(0, nty - 1).long()
    y1 = ((vd[sel] + rad[sel]) / TILE).floor().clamp(0, nty - 1).long()
    nx, ny = x1 - x0 + 1, y1 - y0 + 1
    cnt = nx * ny
    gid = torch.repeat_interleave(sel, cnt)
    start = torch.repeat_interleave(torch.cumsum(cnt, 0) - cnt, cnt)
    k = torch.arange(len(gid)) - start
    nxr = torch.repeat_interleave(nx, cnt)
    tx = torch.repeat_interleave(x0, cnt) + k % nxr
    ty = torch.repeat_interleave(y0, cnt) + torch.div(k, nxr, rounding_mode="floor")
    tid = ty * ntx + tx
    key = tid.double() * 1e6 + z.detach()[gid].double().clamp(max=9.9e5)
    order = torch.argsort(key)
    tid, gid = tid[order], gid[order]
    ntile = ntx * nty
    tcount = torch.bincount(tid, minlength=ntile)
    tstart = torch.cumsum(tcount, 0) - tcount
    rank = torch.arange(len(tid)) - tstart[tid]
    # pixel centers per tile
    py, px = torch.meshgrid(torch.arange(TILE), torch.arange(TILE), indexing="ij")
    tyy, txx = torch.div(torch.arange(ntile), ntx, rounding_mode="floor"), torch.arange(ntile) % ntx
    PX = (txx[:, None] * TILE + px.reshape(1, -1)).float() + 0.5     # (T, 256)
    PY = (tyy[:, None] * TILE + py.reshape(1, -1)).float() + 0.5
    op_all = torch.sigmoid(opac[idx]); rgb_all = torch.sigmoid(col[idx])
    # front-to-back compositing in chunks of KMAX gaussians, so crowded tiles are not truncated
    out = torch.zeros(ntile, TILE * TILE, 3)
    Trun = torch.ones(ntile, TILE * TILE)
    kcap = min(int(tcount.max()), 6 * KMAX) if len(tid) else 0
    for k0 in range(0, kcap, KMAX):
        act = (tcount > k0).nonzero().squeeze(1)
        m = (rank >= k0) & (rank < k0 + KMAX)
        pos = torch.full((ntile,), -1, dtype=torch.long); pos[act] = torch.arange(len(act))
        table = torch.full((len(act), KMAX), -1, dtype=torch.long)
        table[pos[tid[m]], rank[m] - k0] = gid[m]
        valid = table >= 0
        tg = table.clamp(min=0)
        mu = mean2d[tg]; cn = conic[tg]
        dx = PX[act][:, :, None] - mu[:, None, :, 0]
        dy = PY[act][:, :, None] - mu[:, None, :, 1]
        power = -0.5 * (cn[:, None, :, 0] * dx * dx + cn[:, None, :, 2] * dy * dy) - cn[:, None, :, 1] * dx * dy
        alpha = (op_all[tg][:, None, :] * torch.exp(power.clamp(max=0))).clamp(max=0.99)
        alpha = alpha * valid[:, None, :] * (alpha > 1 / 255)
        Tl = torch.cumprod(torch.cat([torch.ones_like(alpha[:, :, :1]), 1 - alpha[:, :, :-1]], 2), 2)
        T0 = Trun[act]
        w = alpha * Tl * T0[:, :, None]
        out = out.index_add(0, act, torch.einsum("tpk,tkc->tpc", w, rgb_all[tg]))
        Trun = Trun.index_copy(0, act, T0 * Tl[:, :, -1] * (1 - alpha[:, :, -1]))
    Tfin = Trun
    # sky: blend horizon->zenith by how far up the pixel ray points
    ray = torch.stack([(PX - cx) / f, (PY - cy) / f, torch.ones_like(PX)], -1) @ d["Rt"]  # cam->world dirs
    up = (ray[..., 2] / ray.norm(dim=-1)).clamp(0, 1)[..., None]
    bg = sky[0] * (1 - up) + sky[1] * up
    out = out + Tfin[..., None] * bg
    # tiles -> image
    img = out.view(nty, ntx, TILE, TILE, 3).permute(0, 2, 1, 3, 4).reshape(nty * TILE, ntx * TILE, 3)[:H, :W]
    return img, idx, mean2d, sel

# ---------------------------------------------------------------- train
grad_acc = torch.zeros(len(means)); grad_n = torch.zeros(len(means))
t0 = time.time()
def save_preview(it):
    with torch.no_grad():
        for j in (0, len(imgs) // 2):
            img, *_ = render(imgs[j])
            both = torch.cat([imgs[j]["img"], img.clamp(0, 1)], 1)
            Image.fromarray((both.numpy() * 255).astype(np.uint8)).save(f"out/preview_{j}.png")

def densify():
    global means, log_s, quats, opac, col, opt, grad_acc, grad_n
    g = grad_acc / grad_n.clamp(min=1)
    with torch.no_grad():
        big = torch.exp(log_s).max(1).values
        # densify the top 4% by view-space gradient (absolute thresholds depend on units/resolution)
        thr = torch.quantile(g[grad_n > 0], 0.96) if (grad_n > 0).any() else float("inf")
        hot = (g >= thr) & (grad_n > 0)
        clone = hot & (big < 0.15)
        split = hot & ~clone
        room = MAXN - len(means)
        if room <= 0:
            clone[:] = False; split[:] = False
        newm = [means[clone]]
        if split.any():
            samp = torch.randn(split.sum(), 3) * torch.exp(log_s[split])
            newm.append(means[split] + (quat_to_rot(quats[split]) @ samp[:, :, None]).squeeze(-1))
        nm = torch.cat(newm)
        ns = torch.cat([log_s[clone], log_s[split] - math.log(1.6)])
        nq = torch.cat([quats[clone], quats[split]]); no = torch.cat([opac[clone], opac[split]])
        nc = torch.cat([col[clone], col[split]])
        if split.any():
            log_s[split] -= math.log(1.6)
        keep = torch.sigmoid(opac) > 0.01
        M_ = torch.cat([means[keep], nm]); S_ = torch.cat([log_s[keep], ns]); Q_ = torch.cat([quats[keep], nq])
        O_ = torch.cat([opac[keep], no]); C_ = torch.cat([col[keep], nc])
    means = M_.detach().requires_grad_(True); log_s = S_.detach().requires_grad_(True)
    quats = Q_.detach().requires_grad_(True); opac = O_.detach().requires_grad_(True)
    col = C_.detach().requires_grad_(True)
    opt = make_opt()
    grad_acc = torch.zeros(len(means)); grad_n = torch.zeros(len(means))
    print(f"  densify: clone {int(clone.sum())} split {int(split.sum())} prune {int((~keep).sum())} -> {len(means)}", flush=True)

order = np.random.permutation(len(imgs))
for it in range(1, ITERS + 1):
    d = imgs[order[it % len(order)]]
    if it % len(order) == 0:
        order = np.random.permutation(len(imgs))
    img, idx, mean2d, sel = render(d, want_grad2d=True)
    loss = (img - d["img"]).abs().mean()
    opt.zero_grad()
    loss.backward()
    opt.step()
    with torch.no_grad():
        quats /= quats.norm(dim=1, keepdim=True)
        if mean2d.grad is not None:
            gn = mean2d.grad.norm(dim=1)
            grad_acc[idx] += gn; grad_n[idx] += (gn > 0).float()
    if it in DENSIFY_AT:
        densify()
    if it % 50 == 0:
        print(f"it {it} loss {loss.item():.4f} N {len(means)} {(time.time()-t0)/it:.2f}s/it", flush=True)
    if it % 300 == 0:
        save_preview(it)
        torch.save({"means": means, "log_s": log_s, "quats": quats, "opac": opac, "col": col, "sky": sky},
                   "out/ckpt.pt")

# ---------------------------------------------------------------- export .splat (three.js frame)
with torch.no_grad():
    o = torch.sigmoid(opac)
    keep = o > 0.05
    m = means[keep].numpy(); sc = torch.exp(log_s[keep]).numpy(); q = quats[keep].numpy()
    c = torch.sigmoid(col[keep]).numpy(); o = o[keep].numpy()
order = np.argsort(-(o * sc.prod(1)))
# ENU -> three.js (x=E, y=U, z=-N): rotation Rx(-90deg)
Rm = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=np.float64)
qr = np.array([math.cos(-math.pi / 4), math.sin(-math.pi / 4), 0, 0])   # quat of Rm (w,x,y,z)
def qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b.T
    return np.stack([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2], 1)
buf = bytearray()
mt = (Rm @ m.T).T
qt = qmul(qr, q)
qt /= np.linalg.norm(qt, axis=1, keepdims=True)
for i in order:
    buf += np.asarray(mt[i], np.float32).tobytes() + np.asarray(sc[i], np.float32).tobytes()
    buf += bytes([int(c[i, 0]*255), int(c[i, 1]*255), int(c[i, 2]*255), int(o[i]*255)])
    buf += bytes(np.clip(qt[i] * 128 + 128, 0, 255).astype(np.uint8))
open("out/scene.splat", "wb").write(buf)
json.dump({"sky": sky.detach().tolist(), "count": len(order),
           "align_median_m": float(np.median(res)), "align_scale": float(s)}, open("out/scene_meta.json", "w"))
# occupancy for collision warnings: opaque gaussians above the lawn, 0.5 m cells (ENU indices)
cell = 0.5
solid = (o > 0.5) & (m[:, 2] > 0.4) & (m[:, 2] < 30)
cells = sorted({tuple(int(v) for v in np.floor(p / cell)) for p in m[solid]})
json.dump({"cell": cell, "cells": cells}, open("out/occupancy.json", "w"))
print("occupied cells:", len(cells))
print(f"exported {len(order)} gaussians, {len(buf)/1e6:.1f} MB, total {time.time()-t0:.0f}s")
