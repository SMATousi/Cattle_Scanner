"""Time-aware session rig: test a long session for rig drift, then build one or many rigs.

`session_consensus.py` assumes the rig is static for the whole session. That holds for a
45-minute session; beef_farm_08_26_2026 runs 7.6 hours with nine multi-minute gaps, and a
bumped camera in the middle would be averaged into a rig that is wrong for everybody
(the failure mode that forced per-epoch rigs on the pig rig).

Stages:
  1. recover  : RoMa/TEASER per-animal rigs on a TIME-STRATIFIED sample (recover_animal),
                cached in <session>/<recover-out>/ and reused on re-runs.
  2. drift    : per view, cluster the recovered rotations/translations and report whether
                cluster membership tracks capture time. Writes drift_report.json + a
                per-view table. `--analyze-only` stops here.
  3. rig      : one consensus rig for the whole session (default), or one per epoch when
                --epochs is given (comma-separated HH:MM boundaries, or "auto" to use the
                boundaries the drift stage proposes).
  4. apply    : consensus + guarded animal-only ICP polish for EVERY complete animal, using
                the rig of the epoch that animal was captured in. Same outputs and same
                quality bookkeeping as session_consensus.py.

Capture time comes from the mkv ctime (the camera clocks are unset, so mkv mtime is junk;
ctime is when the file was copied off the camera and is monotone in capture order).

Usage:
  python session_rig_drift.py --session <dir> --sample 40 --analyze-only
  python session_rig_drift.py --session <dir> --sample 40 --out extrinsics_session_rig
  python session_rig_drift.py --session <dir> --epochs auto --out extrinsics_session_rig
"""
import argparse, fnmatch, glob, json, os, sys, time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import open3d as o3d
from recover_extrinsics_roma import AnimalViews, T_INDEX, VIEWS, WORLD, recover_animal
from session_consensus import (build_consensus, complete_animals, polish_animal,
                               chordal_mean, rot_diff_deg, FLAG_FITNESS)

# a view is called drifted when its recovered poses split into time-contiguous groups
# further apart than this; well below the 5 deg consensus inlier radius, so a split this
# size would already be quietly corrupting the averaged rig
DRIFT_ROT = 1.5      # deg
DRIFT_TRANS = 0.03   # m


def capture_times(session):
    """{animal: unix ctime of its first mkv}. ctime survives extraction; mtime does not."""
    t = {}
    for d in sorted(glob.glob(os.path.join(session, "Animal_*"))):
        if not os.path.isdir(d):
            continue
        mkvs = sorted(glob.glob(os.path.join(d, "*.mkv")))
        if mkvs:
            t[os.path.basename(d)] = os.stat(mkvs[0]).st_ctime
    return t


def stratified_sample(animals, times, n):
    """n animals spread evenly over capture time, not over the alphabet."""
    ordered = sorted(animals, key=lambda a: times.get(a, 0))
    if n >= len(ordered):
        return ordered
    idx = np.linspace(0, len(ordered) - 1, n).round().astype(int)
    return [ordered[i] for i in sorted(set(idx.tolist()))]


def load_cached(rec_dir, animal):
    """Reuse a previous recover_animal run (same check session_consensus makes)."""
    aid = animal.replace("Animal_", "")
    qp = os.path.join(rec_dir, f"A{aid}_quality.json")
    if not os.path.exists(qp):
        return None
    try:
        res = json.load(open(qp))
    except Exception:
        return None
    if "bridged_views" not in res:
        return None
    T = {}
    for v in VIEWS:
        p = os.path.join(rec_dir, f"A{aid}_T{T_INDEX[v]}.npy")
        if os.path.exists(p):
            T[v] = np.load(p)
    if not T:
        return None
    res["final_fitness_per_view"] = {int(k): x for k, x in res["final_fitness_per_view"].items()}
    return T, res


def recover_sample(session, sample, rec_dir, times):
    os.makedirs(rec_dir, exist_ok=True)
    matcher = None
    recovered = {}
    for k, a in enumerate(sample):
        hit = load_cached(rec_dir, a)
        if hit is not None:
            recovered[a] = hit
            print(f"[{k+1}/{len(sample)}] {a}: reused cached recovery", flush=True)
            continue
        if matcher is None:
            print("loading RoMa (once) ...", flush=True)
            from vismatch import get_matcher
            matcher = get_matcher("roma", device="cuda")
        try:
            T, res = recover_animal(session, a, matcher, rec_dir)
        except Exception as ex:
            print(f"[{k+1}/{len(sample)}] {a}: FAILED {type(ex).__name__}: {ex}", flush=True)
            continue
        if T is None:
            print(f"[{k+1}/{len(sample)}] {a}: skipped {res['warnings']}", flush=True)
            continue
        recovered[a] = (T, res)
        print(f"[{k+1}/{len(sample)}] {a}: {len(T)} views, bridged={res['bridged_views']}, "
              f"{res['runtime_s']}s  (t={time.strftime('%H:%M', time.localtime(times.get(a,0)))})",
              flush=True)
    return recovered


def cluster_views(Ts, rot_tol, trans_tol):
    """Greedy tight clustering of poses; returns a label per input, largest cluster = 0."""
    n = len(Ts)
    labels = [-1] * n
    lab = 0
    remaining = list(range(n))
    while remaining:
        best = []
        for k in remaining:
            inl = [i for i in remaining
                   if rot_diff_deg(Ts[i], Ts[k]) < rot_tol
                   and np.linalg.norm(Ts[i][:3, 3] - Ts[k][:3, 3]) < trans_tol]
            if len(inl) > len(best):
                best = inl
        for i in best:
            labels[i] = lab
        remaining = [i for i in remaining if i not in set(best)]
        lab += 1
    return labels


def drift_report(recovered, times, rot_tol=DRIFT_ROT, trans_tol=DRIFT_TRANS):
    """Per view: does the recovered pose split into time-contiguous clusters?

    A bump gives clusters that are contiguous in capture time. Scattered clusters mean
    noisy per-animal recovery, not a moved camera -- consensus handles those already.
    """
    rep, boundaries = {}, []
    for v in VIEWS:
        if v == WORLD:
            continue
        items = [(a, T[v]) for a, (T, _) in recovered.items() if v in T]
        items.sort(key=lambda x: times.get(x[0], 0))
        if len(items) < 4:
            rep[v] = {"n": len(items), "status": "too few"}
            continue
        names = [a for a, _ in items]
        Ts = [T for _, T in items]
        labels = cluster_views(Ts, rot_tol, trans_tol)
        # contiguity: how many label changes along the time axis? A single bump gives 1.
        switches = [i for i in range(1, len(labels)) if labels[i] != labels[i - 1]]
        n_lab = len(set(labels))
        sizes = {l: labels.count(l) for l in sorted(set(labels))}
        big = [l for l, s in sizes.items() if s >= 3]
        contiguous = n_lab > 1 and len(switches) <= (n_lab - 1) + 1 and len(big) > 1
        # spread of the dominant cluster, for reference against ROT_IN=5 deg
        dom = max(sizes, key=lambda l: sizes[l])
        domTs = [Ts[i] for i in range(len(Ts)) if labels[i] == dom]
        m = chordal_mean(domTs)
        spread = max(rot_diff_deg(T, m) for T in domTs)
        rep[v] = {"n": len(items), "clusters": n_lab, "sizes": sizes,
                  "label_switches": len(switches), "time_contiguous": bool(contiguous),
                  "dom_spread_deg": round(spread, 2),
                  "status": "DRIFT" if contiguous else "stable",
                  "timeline": [{"animal": names[i],
                                "t": time.strftime("%H:%M", time.localtime(times.get(names[i], 0))),
                                "label": labels[i]} for i in range(len(items))]}
        if contiguous:
            for i in switches:
                boundaries.append((times.get(names[i - 1], 0) + times.get(names[i], 0)) / 2)
    return rep, sorted(boundaries)


def assign_epochs(times, bounds):
    """{animal: epoch index} from sorted unix-time boundaries."""
    return {a: int(np.searchsorted(bounds, t)) for a, t in times.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True)
    ap.add_argument("--sample", type=int, default=40)
    ap.add_argument("--out", default="extrinsics_session_rig")
    ap.add_argument("--recover-out", default="extrinsics_per_animal")
    ap.add_argument("--min-views", type=int, default=8)
    ap.add_argument("--exclude", default=None,
                    help="comma-separated fnmatch patterns to drop, e.g. 'Animal_test*'")
    ap.add_argument("--analyze-only", action="store_true")
    ap.add_argument("--epochs", default=None,
                    help="'auto' to use the boundaries the drift stage proposes, or a "
                         "comma-separated list of HH:MM cut points. Default: one rig")
    ap.add_argument("--rot-tol", type=float, default=DRIFT_ROT)
    ap.add_argument("--trans-tol", type=float, default=DRIFT_TRANS)
    args = ap.parse_args()

    session = args.session.rstrip("/")
    animals = complete_animals(session, args.min_views)
    if args.exclude:
        pats = [p.strip() for p in args.exclude.split(",") if p.strip()]
        drop = [a for a in animals if any(fnmatch.fnmatch(a, p) for p in pats)]
        animals = [a for a in animals if a not in drop]
        print(f"excluded {len(drop)}: {', '.join(drop)}", flush=True)
    times = capture_times(session)
    print(f"{len(animals)} complete animals; capture span "
          f"{time.strftime('%H:%M', time.localtime(min(times[a] for a in animals)))}-"
          f"{time.strftime('%H:%M', time.localtime(max(times[a] for a in animals)))}", flush=True)

    sample = stratified_sample(animals, times, args.sample)
    print(f"time-stratified sample ({len(sample)}): {sample}", flush=True)
    rec_dir = os.path.join(session, args.recover_out)
    recovered = recover_sample(session, sample, rec_dir, times)
    print(f"\nrecovered {len(recovered)}/{len(sample)}", flush=True)

    rep, bounds = drift_report(recovered, times, args.rot_tol, args.trans_tol)
    out_dir = os.path.join(session, args.out)
    os.makedirs(out_dir, exist_ok=True)
    print("\n--- drift report (tol %.1f deg / %.0f mm) ---" % (args.rot_tol, args.trans_tol * 1000),
          flush=True)
    for v in VIEWS:
        if v == WORLD or v not in rep:
            continue
        r = rep[v]
        print(f"  view {v}: n={r['n']:<3} {r.get('status'):<9} clusters={r.get('clusters')} "
              f"sizes={r.get('sizes')} switches={r.get('label_switches')} "
              f"dom_spread={r.get('dom_spread_deg')} deg", flush=True)
    if bounds:
        print("  proposed epoch boundaries: " +
              ", ".join(time.strftime("%H:%M", time.localtime(b)) for b in bounds), flush=True)
    else:
        print("  no time-contiguous splits -> a single session rig is justified", flush=True)
    with open(os.path.join(out_dir, "drift_report.json"), "w") as f:
        json.dump({"tol_rot_deg": args.rot_tol, "tol_trans_m": args.trans_tol,
                   "per_view": {str(v): rep.get(v, {}) for v in VIEWS},
                   "proposed_boundaries": [time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(b))
                                           for b in bounds]}, f, indent=1)
    if args.analyze_only:
        print(f"\nanalyze-only; report in {out_dir}/drift_report.json", flush=True)
        return

    # ---- epochs ----
    if args.epochs == "auto":
        ep_bounds = bounds
    elif args.epochs:
        day = time.localtime(min(times.values()))
        ep_bounds = []
        for s in args.epochs.split(","):
            hh, mm = s.strip().split(":")
            ep_bounds.append(time.mktime((day.tm_year, day.tm_mon, day.tm_mday,
                                          int(hh), int(mm), 0, 0, 0, -1)))
        ep_bounds.sort()
    else:
        ep_bounds = []
    ep_of = assign_epochs(times, ep_bounds)
    n_ep = len(ep_bounds) + 1
    print(f"\nbuilding {n_ep} rig(s)", flush=True)

    rigs = {}
    for e in range(n_ep):
        sub = {a: v for a, v in recovered.items() if ep_of.get(a, 0) == e}
        if len(sub) < 3:
            print(f"  epoch {e}: only {len(sub)} recovered animals -- too few, will fall back",
                  flush=True)
            rigs[e] = None
            continue
        rig, rig_rep = build_consensus(sub, session=session)
        rigs[e] = rig
        print(f"  epoch {e} ({len(sub)} animals): {len(rig)} views", flush=True)
        for v in VIEWS:
            r = rig_rep.get(v, {"status": "world"})
            print(f"    view {v}: {r.get('status')} inliers={r.get('inliers')} "
                  f"nat={r.get('natural')} spread={r.get('max_spread_deg')} "
                  f"valfit={r.get('validate_fitness')}", flush=True)
        pre = "session_rig" if n_ep == 1 else f"epoch{e}_rig"
        for v in VIEWS:
            if v in rig:
                np.save(os.path.join(out_dir, f"{pre}_T{T_INDEX[v]}.npy"), rig[v])
        with open(os.path.join(out_dir, f"{pre}_report.json"), "w") as f:
            json.dump({str(v): rig_rep.get(v, {}) for v in VIEWS}, f, indent=1)

    # a global fallback rig for epochs too thin to solve on their own
    glob_rig, glob_rep = (build_consensus(recovered, session=session) if n_ep > 1
                          else (rigs[0], None))
    if n_ep > 1:
        for v in VIEWS:
            if v in glob_rig:
                np.save(os.path.join(out_dir, f"session_rig_T{T_INDEX[v]}.npy"), glob_rig[v])
        with open(os.path.join(out_dir, "session_rig_report.json"), "w") as f:
            json.dump({str(v): glob_rep.get(v, {}) for v in VIEWS}, f, indent=1)
    for e in range(n_ep):
        if rigs[e] is None:
            rigs[e] = glob_rig

    if max(len(r) for r in rigs.values()) < 6:
        print("consensus too weak; aborting application stage", flush=True)
        return

    # ---- apply + polish, every animal on its own epoch's rig ----
    qpath = os.path.join(out_dir, "session_quality.jsonl")
    n_flag = 0
    t_start = time.time()
    with open(qpath, "w") as qf:
        for k, a in enumerate(animals):
            t0 = time.time()
            e = ep_of.get(a, 0)
            rig = rigs[e]
            try:
                av = AnimalViews(session, a)
                views = [v for v in sorted(rig) if v in av.views]
                T, info = polish_animal(av, rig, views)
            except Exception as ex:
                qf.write(json.dumps({"animal": a, "epoch": e, "error": str(ex)}) + "\n")
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
            qf.write(json.dumps(
                {"animal": a, "epoch": e, "views": views,
                 "per_view": {str(v): info[v] for v in views},
                 "min_fitness": min(fits) if fits else None,
                 "mean_fitness": round(float(np.mean(fits)), 3) if fits else None,
                 "flagged_views": flags, "runtime_s": round(time.time() - t0, 1)}) + "\n")
            print(f"[{k+1}/{len(animals)}] {a}: ep={e} mean_fit="
                  f"{round(float(np.mean(fits)), 3) if fits else None} flags={flags} "
                  f"({time.time()-t0:.1f}s)", flush=True)

    print(f"\nDONE. {len(animals)} animals, {n_flag} flagged, {n_ep} rig(s), "
          f"{(time.time()-t_start)/60:.1f} min. Outputs in {out_dir}", flush=True)


if __name__ == "__main__":
    main()
