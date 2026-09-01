"""Session-consensus extrinsics: whole-session rig from multi-animal agreement.

Rationale (validated on beef_farm_03_11_2026): the rig is static within a session, but
per-animal left-cluster BRIDGING is unreliable — plausible-looking wrong bridges pass
coherence certification (observed 17 deg and 114 deg errors at coherence 0.19-0.29),
while animals with naturally-gated full connectivity recover the rig to 1-2 deg.
Consensus across many animals lets correct solutions cluster and outvotes bad bridges.

Stages:
  1. RoMa/TEASER recovery (recover_extrinsics_roma) on a SAMPLE of animals spread
     across the session (default 30) -> per-animal rigs + bridged-view bookkeeping.
  2. Robust per-view consensus: inlier clustering (ROT_IN deg / TRANS_IN m), winning
     cluster must contain at least one naturally-connected (unbridged) contributor;
     chordal-mean rotation + mean translation of inliers.
  3. Apply the consensus rig to EVERY complete animal; per-view ANIMAL-ONLY ICP polish
     (masked cow only, guarded: max POLISH_ROT deg / POLISH_TRANS m from consensus,
     kept only if masked-cow fitness improves); save final transforms + quality row.

Outputs in <session>/<out>/:
  session_rig_T<n>.npy            consensus rig (T0<->view20/world, T1..T9<->views 11..19)
  session_rig_report.json         consensus contributors/inliers per view
  A<id>_T<n>.npy                  final per-animal (consensus + polish)
  session_quality.jsonl           one row per animal: fitness before/after polish, flags

Usage:
  python session_consensus.py --session <dir> [--sample 30] [--out teaser_reg_results_session]
"""
import argparse, json, os, sys, time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import open3d as o3d
from recover_extrinsics_roma import (
    AnimalViews, T_INDEX, VIEWS, WORLD, ICP_DIST, VOXEL, recover_animal)

ROT_IN = 5.0        # deg, consensus inlier threshold
TRANS_IN = 0.15     # m
MIN_INLIERS = 3
FIT_MIN_CONTRIB = 0.30   # per-view fitness needed to contribute to consensus
FIT_VALIDATE = 0.40      # median cow-assembly fitness needed to ACCEPT a consensus view
POLISH_ROT = 5.0    # deg, max polish deviation from consensus
POLISH_TRANS = 0.10  # m
POLISH_GAIN = 0.005  # min fitness improvement to accept polish
FLAG_FITNESS = 0.25  # per-view final fitness below this -> flag animal


def rot_diff_deg(A, B):
    Rd = A[:3, :3] @ B[:3, :3].T
    return float(np.degrees(np.arccos(np.clip((np.trace(Rd) - 1) / 2, -1, 1))))


def chordal_mean(Ts):
    Rm = np.mean([T[:3, :3] for T in Ts], axis=0)
    U, _, Vt = np.linalg.svd(Rm)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.mean([t[:3, 3] for t in Ts], axis=0)
    return T


def complete_animals(session, min_views=8):
    out = []
    for a in sorted(os.listdir(session)):
        adir = os.path.join(session, a)
        if not (a.startswith("Animal_") and os.path.isdir(adir)):
            continue
        have = []
        for v in VIEWS:
            stem = os.path.join(adir, f"{a}_nano_{v}")
            if all(os.path.exists(stem + s) for s in
                   [".jpg", "_depth.png", "_mask.png", "_intrinsics.txt"]):
                have.append(v)
        if len(have) >= min_views and WORLD in have:
            out.append(a)
    return out


def build_consensus(recovered, session=None):
    """recovered: {animal: (T_dict, result)}. Returns rig {view: T}, report.

    Two-stage: (1) per view, chordal mean of the largest tight cluster (>= MIN_INLIERS);
    (2) validate each candidate view by median cow-assembly fitness across sample animals
    (GT-free, direct correctness test) -- drops views below FIT_VALIDATE. This replaces the
    old '>=1 natural contributor' proxy, which false-rejected correct-but-bridged views on
    rigs where every view is bridge-dependent (e.g. thompson4). A wrong bridge assembles a
    scrambled cow -> low fitness -> still rejected, so anti-alias protection is retained."""
    rig, report = {WORLD: np.eye(4)}, {}
    for v in VIEWS:
        if v == WORLD:
            continue
        cands = []
        for a, (T, res) in recovered.items():
            if v not in T:
                continue
            ff = res["final_fitness_per_view"]
            if ff.get(v, ff.get(str(v), 0)) < FIT_MIN_CONTRIB:
                continue
            natural = v not in res.get("bridged_views", [])
            cands.append((a, T[v], natural))
        best = None
        for _, Tk, _ in cands:
            inl = [(a, T, nat) for (a, T, nat) in cands
                   if rot_diff_deg(T, Tk) < ROT_IN and
                   np.linalg.norm(T[:3, 3] - Tk[:3, 3]) < TRANS_IN]
            if best is None or len(inl) > len(best[1]):
                best = ((len(inl), sum(n for _, _, n in inl)), inl)
        if best is None or len(best[1]) < MIN_INLIERS:
            report[v] = {"status": "no consensus",
                         "best_cluster": len(best[1]) if best else 0,
                         "candidates": len(cands)}
            continue
        (_, n_nat), inl = best
        rig[v] = chordal_mean([T for (_, T, _) in inl])
        spread = max(rot_diff_deg(T, rig[v]) for (_, T, _) in inl)
        report[v] = {"status": "candidate", "inliers": len(inl), "natural": n_nat,
                     "candidates": len(cands), "max_spread_deg": round(spread, 2),
                     "contributors": [a for (a, _, _) in inl]}

    # ---- stage 2: validate candidate views by cow-assembly fitness ----
    if session is not None:
        val_animals = list(recovered)[:min(8, len(recovered))]
        avs = {}
        for a in val_animals:
            try:
                avs[a] = AnimalViews(session, a)
            except Exception:
                pass
        for v in [x for x in rig if x != WORLD]:
            fits = []
            for a, av in avs.items():
                views = [u for u in rig if u in av.views]
                if v not in views or len(views) < 3:
                    continue
                asm = o3d.geometry.PointCloud()
                for u in views:
                    if u == v:
                        continue
                    c = o3d.geometry.PointCloud(av.cloud(u, "animal"))
                    c.transform(rig[u])
                    asm += c
                asm = asm.voxel_down_sample(VOXEL)
                fits.append(o3d.pipelines.registration.evaluate_registration(
                    av.cloud(v, "animal"), asm, ICP_DIST, rig[v]).fitness)
            med = float(np.median(fits)) if fits else 0.0
            report[v]["validate_fitness"] = round(med, 3)
            if med >= FIT_VALIDATE:
                report[v]["status"] = "ok"
            else:
                report[v]["status"] = "rejected (low assembly fitness)"
                del rig[v]
    return rig, report


def polish_animal(av, rig, views):
    """Animal-only ICP polish per view, guarded. Returns T_final, per-view info."""
    T = {v: rig[v].copy() for v in views}
    info = {}
    # assembly under consensus (all other views)
    clouds = {v: av.cloud(v, "animal") for v in views}
    for v in views:
        if v == WORLD:
            info[v] = {"fit_before": None, "fit_after": None, "polished": False}
            continue
        asm = o3d.geometry.PointCloud()
        for u in views:
            if u == v:
                continue
            c = o3d.geometry.PointCloud(clouds[u])
            c.transform(T[u])
            asm += c
        asm = asm.voxel_down_sample(VOXEL)
        f0 = o3d.pipelines.registration.evaluate_registration(
            clouds[v], asm, ICP_DIST, T[v]).fitness
        icp = o3d.pipelines.registration.registration_icp(
            clouds[v], asm, 0.02, T[v],
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))
        Tp = np.array(icp.transformation)
        dev_r = rot_diff_deg(Tp, rig[v])
        dev_t = float(np.linalg.norm(Tp[:3, 3] - rig[v][:3, 3]))
        f1 = o3d.pipelines.registration.evaluate_registration(
            clouds[v], asm, ICP_DIST, Tp).fitness
        ok = dev_r <= POLISH_ROT and dev_t <= POLISH_TRANS and f1 >= f0 + POLISH_GAIN
        if ok:
            T[v] = Tp
        info[v] = {"fit_before": round(f0, 3), "fit_after": round(f1 if ok else f0, 3),
                   "polished": bool(ok), "dev_rot_deg": round(dev_r, 2),
                   "dev_trans_m": round(dev_t, 3)}
    return T, info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True)
    ap.add_argument("--sample", type=int, default=30)
    ap.add_argument("--out", default="teaser_reg_results_session")
    ap.add_argument("--recover-out", default="teaser_reg_results_roma")
    ap.add_argument("--min-views", type=int, default=8)
    args = ap.parse_args()

    animals = complete_animals(args.session, args.min_views)
    print(f"{len(animals)} complete animals in session", flush=True)
    step = max(1, len(animals) // args.sample)
    sample = animals[::step][:args.sample]
    print(f"recovery sample ({len(sample)}): {sample}", flush=True)

    out_dir = os.path.join(args.session, args.out)
    rec_dir = os.path.join(args.session, args.recover_out)
    os.makedirs(out_dir, exist_ok=True)

    from vismatch import get_matcher
    matcher = get_matcher("roma", device="cuda")

    # ---- stage 1: per-animal recovery on the sample ----
    recovered = {}
    for k, a in enumerate(sample):
        qpath = os.path.join(rec_dir, f"A{a.replace('Animal_', '')}_quality.json")
        if os.path.exists(qpath):
            res = json.load(open(qpath))
            if "bridged_views" in res:   # only reuse runs made with current bookkeeping
                aid = a.replace("Animal_", "")
                T = {}
                for v in VIEWS:
                    p = os.path.join(rec_dir, f"A{aid}_T{T_INDEX[v]}.npy")
                    if os.path.exists(p):
                        T[v] = np.load(p)
                if T:
                    res["final_fitness_per_view"] = {int(k2): v2 for k2, v2 in res["final_fitness_per_view"].items()}
                    recovered[a] = (T, res)
                    print(f"[{k+1}/{len(sample)}] {a}: reused existing recovery", flush=True)
                    continue
        try:
            T, res = recover_animal(args.session, a, matcher, rec_dir)
        except Exception as ex:
            print(f"[{k+1}/{len(sample)}] {a}: FAILED {ex}", flush=True)
            continue
        if T is None:
            print(f"[{k+1}/{len(sample)}] {a}: skipped {res['warnings']}", flush=True)
            continue
        recovered[a] = (T, res)
        print(f"[{k+1}/{len(sample)}] {a}: {len(T)} views, "
              f"bridged={res['bridged_views']}, {res['runtime_s']}s", flush=True)

    # ---- stage 2: consensus ----
    rig, rig_report = build_consensus(recovered, session=args.session)
    print("\nconsensus rig:", flush=True)
    for v in VIEWS:
        r = rig_report.get(v, {"status": "world"})
        print(f"  view {v}: {r}", flush=True)
    for v in VIEWS:
        if v in rig:
            np.save(os.path.join(out_dir, f"session_rig_T{T_INDEX[v]}.npy"), rig[v])
    with open(os.path.join(out_dir, "session_rig_report.json"), "w") as f:
        json.dump({str(v): rig_report.get(v, {}) for v in VIEWS}, f, indent=1)

    usable_views = sorted(rig.keys())
    if len(usable_views) < 6:
        print("consensus too weak; aborting application stage", flush=True)
        return

    # ---- stage 3: apply + animal-only polish for ALL animals ----
    qpath = os.path.join(out_dir, "session_quality.jsonl")
    n_flag = 0
    with open(qpath, "w") as qf:
        for k, a in enumerate(animals):
            t0 = time.time()
            try:
                av = AnimalViews(args.session, a)
                views = [v for v in usable_views if v in av.views]
                T, info = polish_animal(av, rig, views)
            except Exception as ex:
                qf.write(json.dumps({"animal": a, "error": str(ex)}) + "\n")
                print(f"[{k+1}/{len(animals)}] {a}: ERROR {ex}", flush=True)
                continue
            aid = a.replace("Animal_", "")
            for v in views:
                np.save(os.path.join(out_dir, f"A{aid}_T{T_INDEX[v]}.npy"), T[v])
            fits = [i["fit_after"] for i in info.values() if i["fit_after"] is not None]
            flags = [v for v, i in info.items()
                     if i["fit_after"] is not None and i["fit_after"] < FLAG_FITNESS]
            if flags:
                n_flag += 1
            row = {"animal": a, "views": views, "per_view": {str(v): info[v] for v in views},
                   "min_fitness": min(fits) if fits else None,
                   "mean_fitness": round(float(np.mean(fits)), 3) if fits else None,
                   "flagged_views": flags, "runtime_s": round(time.time() - t0, 1)}
            qf.write(json.dumps(row) + "\n")
            print(f"[{k+1}/{len(animals)}] {a}: mean_fit="
                  f"{row['mean_fitness']} flags={flags} ({row['runtime_s']}s)", flush=True)

    print(f"\nDONE. {len(animals)} animals processed, {n_flag} flagged. "
          f"Outputs in {out_dir}", flush=True)


if __name__ == "__main__":
    main()
