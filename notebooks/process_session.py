"""Bulk driver: process a whole session of animals with a SHARED rig, loading models once.

Speedup vs running process_one_animal.py N times: the SAM3 model (~3.5GB, ~9s) loads ONCE
and stays resident across all animals, and registration is NOT re-run per animal -- every
animal reuses the shared session rig (calibrate it once with session_consensus.py). Only
SAM3 runs on the GPU in the loop; extraction/TSDF/Poisson are CPU. Optional --ba re-runs
RoMa per animal for extra pose refinement (much slower; loads RoMa once too).

Per animal: extract mkv -> SAM3 mask -> apply shared rig -> TSDF fuse -> Poisson ->
surface area + volume -> one row in <session>/<name>_summary.csv. Robust: a failure on one
animal is logged and the loop continues.

Usage:
  python process_session.py --session <dir> --rig <dir> \
      [--fusion tsdf|union] [--gamma 2.0] [--poisson-depth 10] [--ba] [--skip-extract] \
      [--animals A,B,...]   # default: all Animal_* dirs
  --rig points at a dir with session_rig_T*.npy (shared) and/or per-animal A<id>_T*.npy.
"""
import argparse, csv, fnmatch, glob, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import open3d as o3d
from helpers import call_cpp_program
from recover_extrinsics_roma import AnimalViews, WORLD, VIEWS, T_INDEX, recover_animal
from process_one_animal import (load_sam3, sam3_mask_views, fuse_and_crop, load_rig,
                                ba_refine, run_poisson_reconstruction, cloudcompare_volume,
                                add_denoise_args, denoise_params_from_args)


def has_views(session, animal, n=3):
    d = os.path.join(session, animal)
    c = 0
    for v in VIEWS:
        if os.path.exists(os.path.join(d, f"{animal}_nano_{v}.jpg")) or \
           os.path.exists(os.path.join(d, f"{animal}_nano_{v}.mkv")):
            c += 1
    return c >= n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True)
    ap.add_argument("--rig", default=None,
                    help="dir with session_rig_T*/A<id>_T* (shared rig). OMIT to register every "
                         "animal on its own with RoMa+TEASER -- more expensive (~60 s/animal) and "
                         "a little less reliable than a rig that has not moved, but it is the only "
                         "option when no session rig exists, and it gives genuinely per-animal poses")
    ap.add_argument("--tform-dirname", default="transformation_matrices",
                    help="per-animal registration output, written to <session>/<animal>/<this>/")
    ap.add_argument("--pcd-out", default=None,
                    help="one dir for every animal's cleaned registered cloud "
                         "(default: <animal>/result/)")
    ap.add_argument("--mesh-out", default=None,
                    help="one dir for every animal's mesh (default: <animal>/result/)")
    ap.add_argument("--exclude", default=None,
                    help="comma-separated fnmatch patterns to skip, e.g. 'Animal_84*,Animal_test'")
    ap.add_argument("--prepare-only", action="store_true",
                    help="extract mkv + SAM3 mask every animal, then stop. Run this before "
                         "session_consensus.py, which needs the views and masks to already exist")
    ap.add_argument("--force", action="store_true",
                    help="rebuild animals whose mesh already exists (default: skip them, so an "
                         "interrupted batch resumes where it stopped)")
    ap.add_argument("--fusion", choices=["tsdf", "union"], default="tsdf")
    ap.add_argument("--gamma", type=float, default=1.0)
    ap.add_argument("--poisson-depth", type=int, default=10)
    ap.add_argument("--ba", action="store_true", help="per-animal BA refinement (slow)")
    ap.add_argument("--ba-depth-weight", type=float, default=20.0)
    ap.add_argument("--skip-extract", action="store_true")
    ap.add_argument("--animals", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--tau-confidence", type=float, default=0.0,
                    help="normal-consistency Poisson weighting (tau_confidence.py); 1.0 with "
                         "--poisson-depth 8 --keep-largest gives a single-shell mesh")
    ap.add_argument("--keep-largest", action="store_true",
                    help="drop all but the largest connected component of the Poisson mesh")
    add_denoise_args(ap)
    ap.add_argument("--result-root", default=None,
                    help="write meshes/pcds here (per-animal <root>/<animal>/result/) instead "
                         "of onto the source drive -- use when the source drive is near-full")
    args = ap.parse_args()
    den = denoise_params_from_args(args)
    print(f"flying-pixel denoising: {'OFF' if den is None else den}", flush=True)

    session = args.session.rstrip("/")
    name = os.path.basename(session)
    result_root = os.path.join(args.result_root, name) if args.result_root else None
    out_dir = args.out or (result_root if result_root else session)
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, f"{name}_summary.csv")
    csv_new = not os.path.exists(csv_path)

    if args.animals:
        animals = args.animals.split(",")
    else:
        animals = sorted(os.path.basename(d) for d in glob.glob(os.path.join(session, "Animal_*"))
                         if os.path.isdir(d))
    if args.exclude:
        pats = [p.strip() for p in args.exclude.split(",") if p.strip()]
        dropped = [a for a in animals if any(fnmatch.fnmatch(a, p) for p in pats)]
        animals = [a for a in animals if a not in dropped]
        print(f"excluded {len(dropped)}: {', '.join(dropped)}", flush=True)
    pcd_out = args.pcd_out
    mesh_out = args.mesh_out
    for d in (pcd_out, mesh_out):
        if d:
            os.makedirs(d, exist_ok=True)
    print(f"{len(animals)} animals in {name}", flush=True)
    print(f"  clouds -> {pcd_out or '<animal>/result/'}", flush=True)
    print(f"  meshes -> {mesh_out or '<animal>/result/'}", flush=True)
    print(f"  poses  -> <animal>/{args.tform_dirname}/"
          f"{'  (registering per animal)' if not args.rig else '  (shared rig: ' + args.rig + ')'}",
          flush=True)

    # --- load models ONCE ---
    print("loading SAM3 (once) ...", flush=True)
    sam = load_sam3()
    matcher = None
    if args.prepare_only:
        for k, animal in enumerate(animals):
            dpath = os.path.join(session, animal)
            if not os.path.isdir(dpath):
                continue
            t0 = time.time()
            try:
                if not args.skip_extract and not has_views(session, animal, 1):
                    print(f"[{k+1}/{len(animals)}] {animal}: no mkv/views, skip", flush=True)
                    continue
                if not args.skip_extract:
                    call_cpp_program(dpath + "/")
                sam3_mask_views(dpath, animal, args.gamma, processor=sam)
                n = len(glob.glob(os.path.join(dpath, f"{animal}_nano_*_mask.png")))
                print(f"[{k+1}/{len(animals)}] {animal}: {n} masked views "
                      f"({time.time()-t0:.0f}s)", flush=True)
            except Exception as e:
                print(f"[{k+1}/{len(animals)}] {animal}: FAILED {type(e).__name__}: {e}", flush=True)
        print("\nprepare-only done", flush=True)
        return

    if args.ba or not args.rig:
        why = "for registration" if not args.rig else "for --ba"
        print(f"loading RoMa (once, {why}) ...", flush=True)
        from vismatch import get_matcher
        matcher = get_matcher("roma", device="cuda")

    done = failed = skipped = 0
    t_start = time.time()
    with open(csv_path, "a", newline="") as cf:
        w = csv.writer(cf)
        if csv_new:
            w.writerow(["Animal ID", "Surface Area", "Volume"])
        for k, animal in enumerate(animals):
            dpath = os.path.join(session, animal)
            if not os.path.isdir(dpath):
                continue
            t0 = time.time()
            tdir = os.path.join(dpath, args.tform_dirname)
            out_a = os.path.join(result_root, animal, "result") if result_root \
                else os.path.join(dpath, "result")
            pcd_path = os.path.join(pcd_out or out_a, f"{animal}_sam_teaser.ply")
            mesh_path = os.path.join(mesh_out or out_a, f"{animal}_mesh.ply")
            try:
                if os.path.exists(mesh_path) and not args.force:
                    print(f"[{k+1}/{len(animals)}] {animal}: mesh exists, skip (--force to rebuild)",
                          flush=True); skipped += 1; continue
                if not args.skip_extract and not has_views(session, animal, 1):
                    print(f"[{k+1}/{len(animals)}] {animal}: no mkv/views, skip"); skipped += 1; continue
                if not args.skip_extract:
                    call_cpp_program(dpath + "/")
                sam3_mask_views(dpath, animal, args.gamma, processor=sam)

                if args.rig:
                    tforms = load_rig(args.rig, animal)
                else:
                    # reuse a previous run's poses when they are already complete, so an
                    # interrupted batch does not pay for registration twice
                    tforms = load_rig(tdir, animal)
                    if WORLD not in tforms or len(tforms) < len(VIEWS):
                        tforms, qual = recover_animal(session, animal, matcher, tdir)
                        if tforms is None:
                            print(f"[{k+1}/{len(animals)}] {animal}: registration failed "
                                  f"{qual.get('warnings')}", flush=True); failed += 1; continue
                        if qual.get("warnings"):
                            print(f"    reg warnings: {qual['warnings']}", flush=True)
                        if qual.get("bridged_views"):
                            print(f"    bridged views (weaker poses): {qual['bridged_views']}",
                                  flush=True)
                    else:
                        print("    reusing existing transformation_matrices", flush=True)
                if WORLD not in tforms or len(tforms) < 3:
                    print(f"[{k+1}/{len(animals)}] {animal}: rig missing/short, skip"); skipped += 1; continue
                if args.ba:
                    av = AnimalViews(session, animal)
                    try:
                        tforms, _ = ba_refine(av, tforms, matcher, depth_weight=args.ba_depth_weight)
                    except Exception as e:
                        print(f"    BA skipped ({e})")

                # the poses that actually built this animal, stored with the animal --
                # whether they came from per-animal recovery or from a session rig
                os.makedirs(tdir, exist_ok=True)
                aid = animal.replace("Animal_", "")
                for v, T in tforms.items():
                    np.save(os.path.join(tdir, f"A{aid}_T{T_INDEX[v]}.npy"), T)

                os.makedirs(os.path.dirname(pcd_path), exist_ok=True)
                os.makedirs(os.path.dirname(mesh_path), exist_ok=True)
                cloud, used, n0 = fuse_and_crop(dpath, animal, tforms, method=args.fusion,
                                                denoise=den)
                if len(cloud.points) < 500:
                    print(f"[{k+1}/{len(animals)}] {animal}: empty cloud, skip"); skipped += 1; continue
                o3d.io.write_point_cloud(pcd_path, cloud)
                if args.tau_confidence > 0 or args.keep_largest:
                    from tau_confidence import poisson_with_confidence
                    poisson_with_confidence(pcd_path, mesh_path, depth=args.poisson_depth,
                                            confidence=args.tau_confidence,
                                            keep_largest=args.keep_largest, verbose=False)
                else:
                    run_poisson_reconstruction(pcd_path, mesh_path, depth=args.poisson_depth)
                mesh = o3d.io.read_triangle_mesh(mesh_path)
                sa = mesh.get_surface_area()
                vol = cloudcompare_volume(mesh)
                w.writerow([animal, sa, vol]); cf.flush()
                done += 1
                print(f"[{k+1}/{len(animals)}] {animal}: SA={sa:.3f} V={vol:.3f} "
                      f"({len(used)} views, {len(cloud.points)} pts, {time.time()-t0:.0f}s)", flush=True)
            except Exception as e:
                failed += 1
                print(f"[{k+1}/{len(animals)}] {animal}: FAILED {type(e).__name__}: {e}", flush=True)

    dt = time.time() - t_start
    print(f"\nDONE {done} ok / {skipped} skipped / {failed} failed in {dt/60:.1f} min "
          f"({dt/max(done,1):.0f}s per animal). Summary: {csv_path}", flush=True)


if __name__ == "__main__":
    main()
