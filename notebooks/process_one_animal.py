"""End-to-end single-animal pipeline: raw 10 mkv -> metric mesh + volume/SA.

Stages:
  1. extract  : call the k4a C++ extractor on <dir>/*.mkv -> per-view jpg/depth/intrinsics
  2. mask     : SAM3 segments each view -> <stem>_mask.png  (optional gamma for dark rigs)
  3. register : RoMa + TEASER++ + pose-graph -> per-view extrinsics into cam-20 world frame
                (or --rig <dir> to reuse a precomputed session rig -- more reliable if the
                 rig hasn't moved; single-animal recovery can mis-bridge, see manual)
  4. fuse+crop: back-project MASKED depth per view (normals oriented to each camera), merge
                under the extrinsics, then cluster_point_cloud_new drops non-cow specks
  5. visualize + write  <animal>_sam_teaser.ply
  6. poisson  : PoissonRecon -> <animal>_mesh.ply ; visualize
  7. metrics  : surface area + CloudCompare-style volume -> printed and appended to
                <out>/<session>_summary.csv  (columns: Animal ID, Surface Area, Volume)

Usage:
  python process_one_animal.py --data-path <parent> --session <animal_dir_name> \
      [--out <dir>] [--rig <dir>] [--gamma 2.0] [--poisson-depth 10] [--no-show]
  (the 10 mkv files live in <data-path>/<session>/, named <session>_nano_<view>.mkv)
"""
import argparse, csv, os, subprocess, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import open3d as o3d
from PIL import Image
from helpers import get_sam3_mask, cluster_point_cloud_new, call_cpp_program
import depth_denoise as dd
from recover_extrinsics_roma import AnimalViews, recover_animal, T_INDEX, VIEWS, WORLD

POISSON_EXE = "/home/vigir3d/Software/packages/ccPoisson/PoissonRecon/Bin/Linux/PoissonRecon"
MIN_FG = 500


# ----- your Poisson + volume (unchanged) -----
def run_poisson_reconstruction(input_ply, output_ply, depth=10, bType=3,
                               samples_per_node=1.0, point_weight=1.0):
    cmd = [POISSON_EXE, "--in", input_ply, "--out", output_ply, "--bType", str(bType),
           "--depth", str(depth), "--samplesPerNode", str(samples_per_node),
           "--pointWeight", str(point_weight)]
    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode != 0:
        print("Poisson failed:\n", result.stderr.decode())
        raise RuntimeError("Poisson reconstruction failed.")
    print("Poisson reconstruction completed.")
    return output_ply


def cloudcompare_volume(mesh):
    V = np.asarray(mesh.vertices); F = np.asarray(mesh.triangles)
    Vs = V - V.min(axis=0)
    A, B, C = Vs[F[:, 0]], Vs[F[:, 1]], Vs[F[:, 2]]
    signed = np.einsum('ij,ij->i', A, np.cross(B, C)) / 6.0
    return float(np.abs(signed.sum()))


# ----- stages -----
def load_sam3():
    from sam3.model_builder import build_sam3_image_model
    from sam3.model.sam3_image_processor import Sam3Processor
    return Sam3Processor(build_sam3_image_model())


def sam3_mask_views(session_dir, animal, gamma, processor=None):
    """Mask all views. Pass a pre-loaded `processor` (bulk: load once) to avoid re-loading
    the 3.5GB model per animal; if None, loads and frees it here (single-animal use)."""
    own = processor is None
    if own:
        processor = load_sam3()

    def gamma_img(pil):
        a = np.asarray(pil).astype(np.float32) / 255.0
        return Image.fromarray((np.power(a, 1.0 / gamma) * 255).astype(np.uint8))

    made = 0
    for v in VIEWS:
        stem = os.path.join(session_dir, f"{animal}_nano_{v}")
        jpg = stem + ".jpg"
        if not os.path.exists(jpg):
            continue
        mp = stem + "_mask.png"
        if os.path.exists(mp):
            continue
        orig = Image.open(jpg).convert("RGB")
        rgb = np.array(orig)
        det = gamma_img(orig) if gamma and gamma != 1.0 else orig
        mask = get_sam3_mask(det, processor)   # [1,H,W], 1=animal
        if int(mask.sum()) < MIN_FG:
            print(f"  [mask] view {v}: no animal detected (skipped)")
            continue
        m = (mask[0] > 0).astype(np.uint8)[..., None]
        Image.fromarray((rgb * m + 255 * (1 - m)).astype(np.uint8), "RGB").save(mp)
        made += 1
    if own:
        del processor
        import torch; torch.cuda.empty_cache()
    print(f"  masked {made} views")


def load_intr(p):
    v = np.loadtxt(p); return v[2], v[3], v[4], v[5]


def masked_colored_cloud(session_dir, animal, v, voxel=0.005, denoise=None):
    stem = os.path.join(session_dir, f"{animal}_nano_{v}")
    d = np.array(Image.open(stem + "_depth.png")).astype(np.float32) / 1000.0
    rgb = np.array(Image.open(stem + ".jpg").convert("RGB")).astype(np.float32) / 255.0
    m = np.array(Image.open(stem + "_mask.png").convert("L"))
    fx, fy, cx, cy = load_intr(stem + "_intrinsics.txt")
    sel = (d > 0) & (m < 250)
    if denoise is not None:
        # the full depth `d` is passed on purpose: once the bars are zeroed the cliff
        # they make is invisible and the flying pixels beside them look like surface
        sel = dd.clean_view(d, sel, denoise)
    vs, us = np.where(sel); z = d[vs, us]
    p = o3d.geometry.PointCloud()
    p.points = o3d.utility.Vector3dVector(
        np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], 1))
    p.colors = o3d.utility.Vector3dVector(rgb[vs, us])
    p = p.voxel_down_sample(voxel)
    # normals oriented toward THIS camera (origin) while still in camera frame -> correct
    # orientation for Poisson after we transform into the world frame
    p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
    p.orient_normals_towards_camera_location(np.zeros(3))
    return p


def ba_refine(av, tforms, matcher, depth_weight=20.0, max_tracks=4000):
    """Bundle-adjustment refinement of the per-view poses: re-match the gated pairs, build
    multi-view tracks, and jointly optimize poses (view 20 fixed, intrinsics fixed) against
    sub-pixel reprojection + weighted depth. Returns refined transforms + stats.
    NOTE: re-runs RoMa on the pairs (registration doesn't cache its matches), so --ba adds
    roughly one more registration's worth of time."""
    from sfm_ba_rgbd import match_pair_keep_kpts, build_tracks, run_ba
    from recover_extrinsics_roma import PAIRS, GATE
    views = sorted(tforms.keys())
    imgs = {v: matcher.load_image(av.jpg[v]) for v in av.views}
    gated = {}
    for (i, j) in PAIRS:
        if i in views and j in views:
            e = match_pair_keep_kpts(av, matcher, imgs, i, j)
            if e and e["gate"] >= GATE:
                gated[(i, j)] = e
    tracks = build_tracks(gated, max_tracks)
    T_ba, stats = run_ba(av, views, tforms, tracks, depth_weight)
    return T_ba, stats


def load_rig(rig_dir, animal, shared_rig_id=None):
    """Resolve a per-view rig for `animal`. Priority: per-animal A<id>_T* -> shared
    session_rig_T* -> (single-representative sessions) the sole A<id>_T* set present."""
    aid = animal.replace("Animal_", "")
    # single-representative auto-detect: no per-animal & no session_rig, exactly one A*_T0
    if shared_rig_id is None and os.path.isdir(rig_dir) \
            and not os.path.exists(os.path.join(rig_dir, f"A{aid}_T0.npy")) \
            and not os.path.exists(os.path.join(rig_dir, "session_rig_T0.npy")):
        reps = sorted({f[1:].rsplit("_T", 1)[0] for f in os.listdir(rig_dir)
                       if f.startswith("A") and f.endswith("_T0.npy")})
        if len(reps) == 1:
            shared_rig_id = reps[0]
    src = shared_rig_id or aid
    T = {}
    for v in VIEWS:
        for name in (f"A{src}_T{T_INDEX[v]}.npy", f"session_rig_T{T_INDEX[v]}.npy"):
            p = os.path.join(rig_dir, name)
            if os.path.exists(p):
                T[v] = np.load(p); break
    return T


# production TSDF params (match helpers.py ScalableTSDFVolume)
TSDF_VOXEL = 4.0 / 512.0      # ~7.8 mm
TSDF_SDF_TRUNC = 0.04
TSDF_DEPTH_TRUNC = 4.0


def fuse_union(session_dir, animal, tforms, denoise=None):
    """Raw union of masked-depth backprojections (thicker; sensitive to pose error)."""
    fused = o3d.geometry.PointCloud()
    used = []
    for v in VIEWS:
        stem = os.path.join(session_dir, f"{animal}_nano_{v}")
        if v not in tforms or not os.path.exists(stem + "_mask.png"):
            continue
        c = masked_colored_cloud(session_dir, animal, v, denoise=denoise)
        if len(c.points) == 0:
            continue
        c.transform(tforms[v])
        fused += c
        used.append(v)
    fused = fused.voxel_down_sample(0.005)
    n0 = len(fused.points)
    if denoise is not None:
        fused = dd.filter_cloud(fused, _denoise_views(session_dir, animal, tforms, denoise),
                                denoise)
    fused = cluster_point_cloud_new(fused)
    return fused, used, n0


def _denoise_views(session_dir, animal, tforms, p):
    """Package the FULL (un-masked) depth + pose of every registered view for the
    free-space test.  Un-masked is the point: a chute bar the mask discarded is still a
    valid witness that everything in front of it is empty air."""
    views = []
    for v in VIEWS:
        stem = os.path.join(session_dir, f"{animal}_nano_{v}")
        if v not in tforms or not os.path.exists(stem + "_depth.png"):
            continue
        a = np.loadtxt(stem + "_intrinsics.txt")
        views.append(dict(
            depth=np.array(Image.open(stem + "_depth.png")).astype(np.float32) / 1000.0,
            K=(int(a[0]), int(a[1]), a[2], a[3], a[4], a[5]),
            T=tforms[v], margin=p.fs_margin))
    return views


def fuse_tsdf(session_dir, animal, tforms, denoise=None):
    """Multi-view TSDF co-integration: register FIRST, then integrate every masked RGBD
    view into ONE volume with its pose -> overlapping views mutually average -> thin,
    denoised surface. Needs accurate poses (that's why registration precedes this)."""
    vol = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=TSDF_VOXEL, sdf_trunc=TSDF_SDF_TRUNC,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    used = []
    for v in VIEWS:
        stem = os.path.join(session_dir, f"{animal}_nano_{v}")
        if v not in tforms or not os.path.exists(stem + "_mask.png"):
            continue
        depth = np.array(Image.open(stem + "_depth.png")).astype(np.uint16)  # mm
        m = np.array(Image.open(stem + "_mask.png").convert("L"))
        if denoise is None:
            depth[m >= 250] = 0                                # crop to animal
        else:
            dm = depth.astype(np.float32) / 1000.0
            depth[~dd.clean_view(dm, (m < 250) & (dm > 0), denoise)] = 0
        color = o3d.io.read_image(stem + ".jpg")
        w, h, fx, fy, cx, cy = (lambda a: (int(a[0]), int(a[1]), a[2], a[3], a[4], a[5]))(
            np.loadtxt(stem + "_intrinsics.txt"))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, o3d.geometry.Image(depth), depth_scale=1000.0,
            depth_trunc=TSDF_DEPTH_TRUNC, convert_rgb_to_intensity=False)
        intr = o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy)
        # integrate wants world->camera extrinsic; our T is world_from_cam -> invert
        vol.integrate(rgbd, intr, np.linalg.inv(tforms[v]))
        used.append(v)
    pcd = vol.extract_point_cloud()   # thin, denoised, WITH normals (good for Poisson)
    n0 = len(pcd.points)
    if denoise is not None:
        pcd = dd.filter_cloud(pcd, _denoise_views(session_dir, animal, tforms, denoise),
                              denoise)
    if pcd.has_colors() and len(pcd.points):
        pcd = cluster_point_cloud_new(pcd)
    return pcd, used, n0


def fuse_and_crop(session_dir, animal, tforms, method="tsdf", denoise=None):
    return (fuse_tsdf if method == "tsdf" else fuse_union)(
        session_dir, animal, tforms, denoise=denoise)


def denoise_params_from_args(args):
    """Build a DenoiseParams from CLI args, or None when --no-denoise was passed."""
    if getattr(args, "no_denoise", False):
        return None
    return dd.DenoiseParams(
        jump_ws=args.denoise_jump_ws, erode_px=args.denoise_erode,
        fs_margin=args.fs_margin, fs_min_votes=args.fs_votes)


def add_denoise_args(ap):
    g = ap.add_argument_group("flying-pixel denoising (depth_denoise.py)")
    g.add_argument("--no-denoise", action="store_true",
                   help="skip all flying-pixel removal (pre-2026-08-31 behaviour)")
    g.add_argument("--denoise-jump-ws", type=int, default=dd.DEFAULT.jump_ws,
                   help="half-window for the max-jump mixed-pixel test; 0 disables it")
    g.add_argument("--denoise-erode", type=int, default=dd.DEFAULT.erode_px,
                   help="mask erosion in px; the silhouette is all mixed pixels")
    g.add_argument("--fs-votes", type=int, default=dd.DEFAULT.fs_min_votes,
                   help="cameras that must see THROUGH a point before it is dropped "
                        "(1 = aggressive, 2 = tuned, >10 = free-space carving off)")
    g.add_argument("--fs-margin", type=float, default=dd.DEFAULT.fs_margin,
                   help="metres a point must clear the other camera's surface by")


def show_ply(path, title="viewer"):
    """Visualize a .ply (mesh if it has triangles, else point cloud) in an ISOLATED
    subprocess -- Open3D's draw_geometries can hard-segfault on some GL/headless setups,
    and a subprocess crash can't take down the pipeline (outputs are already saved)."""
    if not os.environ.get("DISPLAY"):
        print(f"  (no DISPLAY; skipping viewer -- open {os.path.basename(path)} in CloudCompare/MeshLab)")
        return
    snippet = (
        "import open3d as o3d,sys\n"
        "p=sys.argv[1]\n"
        "m=o3d.io.read_triangle_mesh(p)\n"
        "g=m if len(m.triangles)>0 else o3d.io.read_point_cloud(p)\n"
        "if hasattr(g,'triangles') and len(g.triangles)>0: g.compute_vertex_normals()\n"
        "o3d.visualization.draw_geometries([g],window_name=sys.argv[2])\n")
    r = subprocess.run([sys.executable, "-c", snippet, path, title])
    if r.returncode != 0:
        print(f"  (viewer exited abnormally rc={r.returncode}; open {os.path.basename(path)} "
              f"in CloudCompare/MeshLab)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-path", required=True, help="parent dir containing <session>/")
    ap.add_argument("--session", required=True, help="animal dir name (= Animal ID)")
    ap.add_argument("--out", default=None)
    ap.add_argument("--rig", default=None, help="dir with precomputed A<id>_T*/session_rig_T* (skip registration)")
    ap.add_argument("--gamma", type=float, default=1.0, help="brighten for dark rigs (e.g. 2.0)")
    ap.add_argument("--fusion", choices=["tsdf", "union"], default="tsdf",
                    help="tsdf = multi-view TSDF co-integration (thin denoised, recommended); "
                         "union = raw masked backprojection union")
    ap.add_argument("--ba", action="store_true",
                    help="extra bundle-adjustment refinement of the poses before fusion "
                         "(re-runs RoMa on the pairs -> ~one more registration's time)")
    ap.add_argument("--ba-depth-weight", type=float, default=20.0)
    ap.add_argument("--poisson-depth", type=int, default=10)
    ap.add_argument("--tau-confidence", type=float, default=0.0,
                    help="normal-consistency sample weighting for Poisson (tau_confidence.py, "
                         "Lin et al. 2024). 0 = off (unchanged behaviour); 1.0 collapsed genus "
                         "43 -> 5 on Animal_57 with volume unchanged. Surface AREA does shrink "
                         "with the exponent, so do not raise it if you report area.")
    ap.add_argument("--keep-largest", action="store_true",
                    help="drop all but the largest connected component of the Poisson mesh "
                         "(~180 stray shells, <1%% of faces, volume unchanged)")
    add_denoise_args(ap)
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--skip-extract", action="store_true", help="views already extracted")
    args = ap.parse_args()

    dpath = os.path.join(args.data_path, args.session)
    animal = os.path.basename(dpath.rstrip("/"))
    out_dir = args.out or os.path.join(dpath, "result")
    os.makedirs(out_dir, exist_ok=True)
    t0 = time.time()

    # 1. extract mkv -> per-view files
    if not args.skip_extract:
        print(f"[1/7] extracting mkv in {dpath} ...", flush=True)
        if not call_cpp_program(dpath + "/"):
            print("extraction failed"); return

    # 2. SAM3 masks
    print("[2/7] SAM3 masking ...", flush=True)
    sam3_mask_views(dpath, animal, args.gamma)

    # 3. registration (or reuse a rig)
    matcher = None
    if args.rig:
        print(f"[3/7] using precomputed rig from {args.rig}", flush=True)
        tforms = load_rig(args.rig, animal)
    else:
        print("[3/7] RoMa + TEASER++ + pose-graph registration ...", flush=True)
        from vismatch import get_matcher
        matcher = get_matcher("roma", device="cuda")
        tforms, res = recover_animal(args.data_path, animal, matcher,
                                     os.path.join(out_dir, "teaser_reg_results"))
        if tforms is None:
            print(f"registration failed: {res.get('warnings')}"); return
        if res.get("warnings"):
            print("  reg warnings:", res["warnings"])
    if WORLD not in tforms or len(tforms) < 3:
        print("insufficient registered views; aborting"); return

    # 3b. optional bundle-adjustment refinement
    if args.ba:
        print("[3b] bundle-adjustment refinement ...", flush=True)
        if matcher is None:
            from vismatch import get_matcher
            matcher = get_matcher("roma", device="cuda")
        av = AnimalViews(args.data_path, animal)
        try:
            tforms, stats = ba_refine(av, tforms, matcher, depth_weight=args.ba_depth_weight)
            print(f"  BA: {stats['n_obs']} obs, reproj rmse "
                  f"{stats['reproj_rmse_px_before']:.2f} -> {stats['reproj_rmse_px_after']:.2f} px")
        except Exception as e:
            print(f"  BA skipped ({e}); using pre-BA poses")

    # 4. fuse masked views + cluster filter
    den = denoise_params_from_args(args)
    print(f"[4/7] fusing masked views ({args.fusion}) + cluster filter "
          f"[denoise {'off' if den is None else 'on'}] ...", flush=True)
    cloud, used, n0 = fuse_and_crop(dpath, animal, tforms, method=args.fusion, denoise=den)
    print(f"  {len(used)} views, {n0} -> {len(cloud.points)} pts after filter")

    # 5. save point cloud
    pcd_path = os.path.join(out_dir, f"{animal}_sam_teaser.ply")
    o3d.io.write_point_cloud(pcd_path, cloud)
    print(f"[5/7] wrote {pcd_path}")

    # 6. Poisson mesh
    print("[6/7] Poisson reconstruction ...", flush=True)
    mesh_path = os.path.join(out_dir, f"{animal}_mesh.ply")
    if args.tau_confidence > 0 or args.keep_largest:
        # tau-weighted screened Poisson: down-weights points whose normals disagree with
        # their neighbours, so the solve stops bridging across occluded gaps (udder/hind leg)
        # and drilling tunnels through noise. See tau_confidence.py for the measured ablation.
        from tau_confidence import poisson_with_confidence
        poisson_with_confidence(pcd_path, mesh_path, depth=args.poisson_depth,
                                confidence=args.tau_confidence,
                                keep_largest=args.keep_largest)
    else:
        run_poisson_reconstruction(pcd_path, mesh_path, depth=args.poisson_depth)
    mesh = o3d.io.read_triangle_mesh(mesh_path)

    # 7. metrics -- computed and SAVED before any visualization, so a viewer crash
    #    can never cost you the mesh/volume.
    sa = mesh.get_surface_area()
    vol = cloudcompare_volume(mesh)
    print(f"[7/7] {animal}:  Surface Area = {sa:.4f} m^2   Volume = {vol:.4f} m^3")
    csv_path = os.path.join(out_dir, f"{args.session}_summary.csv")
    new = not os.path.exists(csv_path)
    with open(csv_path, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["Animal ID", "Surface Area", "Volume"])
        w.writerow([animal, sa, vol])
    print(f"      appended to {csv_path}  ({time.time()-t0:.0f}s total)")

    # visualization LAST, isolated -- cloud then mesh
    if not args.no_show:
        show_ply(pcd_path, f"{animal} point cloud")
        show_ply(mesh_path, f"{animal} mesh")


if __name__ == "__main__":
    main()
