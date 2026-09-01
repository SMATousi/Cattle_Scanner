"""Session QC: topology, metrics, body dimensions, and test-retest repeatability.

Reads the layout process_session.py writes:
  <session>/<name>_summary.csv                Animal ID, Surface Area, Volume
  <session>/reconstruction_results/<a>_mesh.ply
  <session>/teaser_reg_results/<a>_sam_teaser.ply
  <session>/<rig>/session_quality.jsonl       per-animal registration fitness

Reports per animal: components / genus / watertight, volume, area, point count, body
length-height-width in the PCA body frame, and registration fitness. Then the two checks
that actually arbitrate quality:

  * TEST-RETEST -- repeat scans of one animal (Animal_X, Animal_X_b, Animal_X_c ...) are
    independent captures, independently registered and meshed, so their spread is a direct
    measure of pipeline repeatability. This beats comparing against a corpus median because
    it controls for the animal.
  * OUTLIER BAND -- volume/area far from the session median, which on this pipeline means a
    broken registration rather than an unusual animal (the failure signature is a volume
    roughly a third of normal, with the surface area collapsing with it).

Body frame follows the corpus convention: PC1 head = the narrower end, PC2 up = negative
third moment (the legs form a sparse downward tail), PC3 = PC1 x PC2.

Usage:
  python qc_session.py --session <dir> [--rig extrinsics_session_rig] [--csv qc.csv]
"""
import argparse, csv, glob, json, os, re, sys
import numpy as np


def body_frame_dims(V):
    """Length/height/width (m) in a canonical PCA body frame."""
    c = V.mean(0)
    X = V - c
    _, _, Vt = np.linalg.svd(X, full_matrices=False)
    pc1, pc2, pc3 = Vt[0], Vt[1], Vt[2]
    p1 = X @ pc1
    # PC1 points head-ward: the head end is the narrower one
    half = p1 > np.median(p1)
    spread = lambda m: np.sqrt((X[m] @ pc2) ** 2 + (X[m] @ pc3) ** 2).mean()
    if spread(half) > spread(~half):
        pc1 = -pc1
        p1 = -p1
    # PC2 up: legs make a sparse tail on the downward side -> negative third moment
    p2 = X @ pc2
    if ((p2 - p2.mean()) ** 3).mean() > 0:
        pc2 = -pc2
        p2 = -p2
    pc3 = np.cross(pc1, pc2)
    p3 = X @ pc3
    return float(p1.ptp()), float(p2.ptp()), float(p3.ptp())


def topology(path):
    """components / genus / watertight without paying for open3d's self-intersection test."""
    import trimesh
    m = trimesh.load(path, process=False)
    out = {"verts": len(m.vertices), "faces": len(m.faces),
           "volume": float(abs(m.volume)), "area": float(m.area),
           "watertight": bool(m.is_watertight)}
    comps = m.split(only_watertight=False)
    out["components"] = len(comps)
    big = max(comps, key=lambda c: len(c.faces)) if len(comps) else m
    out["genus"] = int((2 - big.euler_number) // 2)
    L, H, W = body_frame_dims(np.asarray(m.vertices))
    out["length"], out["height"], out["width"] = L, H, W
    return out


def base_id(a):
    """Animal_31_b -> Animal_31. Suffix is one trailing letter (or a bare trailing
    underscore -- Animal_N5127_ is a typo for the same animal as Animal_N5127)."""
    return re.sub(r"_[a-zA-Z]?$", "", a)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True)
    ap.add_argument("--rig", default="extrinsics_session_rig")
    ap.add_argument("--mesh-dir", default="reconstruction_results")
    ap.add_argument("--pcd-dir", default="teaser_reg_results")
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()

    session = args.session.rstrip("/")
    name = os.path.basename(session)
    mesh_dir = os.path.join(session, args.mesh_dir)

    fit = {}
    qp = os.path.join(session, args.rig, "session_quality.jsonl")
    if os.path.exists(qp):
        for line in open(qp):
            try:
                r = json.loads(line)
            except Exception:
                continue
            # session_consensus writes min_fitness/mean_fitness (not fitness_after)
            fit[r["animal"]] = (r.get("mean_fitness"), r.get("min_fitness"),
                                r.get("flagged_views"), r.get("epoch"))

    rows = []
    meshes = sorted(glob.glob(os.path.join(mesh_dir, "*_mesh.ply")))
    print(f"{len(meshes)} meshes in {mesh_dir}", flush=True)
    for k, mp in enumerate(meshes):
        a = os.path.basename(mp)[:-len("_mesh.ply")]
        try:
            t = topology(mp)
        except Exception as ex:
            print(f"  {a}: FAILED {type(ex).__name__}: {ex}", flush=True)
            continue
        npts = 0
        pp = os.path.join(session, args.pcd_dir, f"{a}_sam_teaser.ply")
        if os.path.exists(pp):
            import open3d as o3d
            npts = len(o3d.io.read_point_cloud(pp).points)
        mf, mnf, flags, ep = fit.get(a, (None, None, None, None))
        rows.append({"animal": a, "epoch": ep, **t, "points": npts,
                     "mean_fit": mf, "min_fit": mnf,
                     "flagged": ",".join(map(str, flags)) if flags else ""})
        if (k + 1) % 25 == 0:
            print(f"  ... {k+1}/{len(meshes)}", flush=True)

    if not rows:
        print("no meshes found"); return
    vol = np.array([r["volume"] for r in rows])
    area = np.array([r["area"] for r in rows])
    med, mad = np.median(vol), np.median(np.abs(vol - np.median(vol)))
    print(f"\n=== {name}: {len(rows)} animals ===")
    print(f"volume  median {med:.4f} m3  MAD {mad:.4f}  range {vol.min():.4f}-{vol.max():.4f}")
    print(f"area    median {np.median(area):.4f} m2  range {area.min():.4f}-{area.max():.4f}")
    comps = np.array([r["components"] for r in rows])
    genus = np.array([r["genus"] for r in rows])
    print(f"single-shell: {(comps == 1).sum()}/{len(rows)}   "
          f"components max {comps.max()}   genus median {int(np.median(genus))} max {genus.max()}")
    L = np.array([r["length"] for r in rows])
    print(f"body length median {np.median(L):.3f} m  range {L.min():.3f}-{L.max():.3f}")

    print("\n--- volume outliers (|v - median| > 4 MAD) ---")
    bad = [r for r in rows if abs(r["volume"] - med) > 4 * max(mad, 1e-6)]
    for r in sorted(bad, key=lambda r: r["volume"]):
        print(f"  {r['animal']:<28} V={r['volume']:.4f} A={r['area']:.3f} "
              f"comps={r['components']} genus={r['genus']} meanfit={r['mean_fit']} "
              f"flagged={r['flagged']}")
    if not bad:
        print("  none")

    print("\n--- test-retest (repeat scans of the same animal) ---")
    groups = {}
    for r in rows:
        groups.setdefault(base_id(r["animal"]), []).append(r)
    reps = {g: v for g, v in groups.items() if len(v) > 1}
    dv, da = [], []
    for g in sorted(reps):
        v = np.array([r["volume"] for r in reps[g]])
        a_ = np.array([r["area"] for r in reps[g]])
        pv = 100 * (v.max() - v.min()) / v.mean()
        pa = 100 * (a_.max() - a_.min()) / a_.mean()
        dv.append(pv); da.append(pa)
        mark = "  <-- " if pv > 5 else ""
        print(f"  {g:<24} n={len(v)}  V={' '.join(f'{x:.4f}' for x in v)}  "
              f"spread {pv:5.2f}%   A spread {pa:5.2f}%{mark}")
    if dv:
        print(f"\n  {len(dv)} repeat groups: volume spread mean {np.mean(dv):.2f}% "
              f"median {np.median(dv):.2f}% worst {np.max(dv):.2f}%")
        print(f"                    area   spread mean {np.mean(da):.2f}% "
              f"median {np.median(da):.2f}% worst {np.max(da):.2f}%")

    out = args.csv or os.path.join(session, f"{name}_qc.csv")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
