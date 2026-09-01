"""Recover per-view rig extrinsics for a scanned animal via RoMa 2D matching.

Pipeline (validated on southwest_12_03/Animal_b291 against production TEASER extrinsics,
final agreement ~1 deg / ~5 cm):
  1. RoMa (vismatch) matches physically-sensible view pairs in RGB.
  2. Matched pixels are lifted to 3D through each view's own depth + intrinsics
     (5x5 median window to survive depth holes).
  3. TEASER++ solves each pair from the 3D-3D correspondences; ICP refines.
  4. Edges are accepted GT-free when min(background_fitness, animal_fitness) >= GATE.
     (kills bilateral-symmetry aliases: a false left<->right transform can't align
     both the asymmetric chute background AND the actual 3D animal.)
  5. Clusters not connected to the world view get bridged by the best remaining
     candidate edge, certified by global animal coherence (cluster's animal clouds
     must overlap the already-registered animal assembly).
  6. Pose-graph optimization (Open3D) distributes residual drift.

Output follows the CONFIRMED production convention:
  T0 <-> view 20 (world frame, identity);  T1..T9 <-> views 11..19.
Files: <out_dir>/A<id>_T<n>.npy + A<id>_quality.json (per-view/per-edge metrics).

Usage:
  python recover_extrinsics_roma.py --session <session_dir> --animals Animal_X,Animal_Y
  python recover_extrinsics_roma.py --session <session_dir> --all
"""
import argparse, itertools, json, os, sys, time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import open3d as o3d
from helpers import get_teaser_solver, Rt2T
from PIL import Image

VIEWS = list(range(11, 21))
WORLD = 20
# T-index convention (confirmed empirically, mean fitness 0.978 on southwest_12_03):
# T0 <-> view 20, T1..T9 <-> views 11..19
T_INDEX = {WORLD: 0, **{v: i + 1 for i, v in enumerate(range(11, 20))}}

# Physically sensible pairs (validated set + backup bridges + cheap extra attempts)
PAIRS = [
    (11, 12), (13, 14), (16, 17), (18, 19), (15, 20),   # intra-pair backbone
    (12, 13), (11, 13), (17, 18), (17, 19), (16, 19),   # within-cluster
    (15, 18), (18, 20),                                 # overhead anchors
    (13, 17), (14, 17), (14, 16),                       # cross-side head bridges
    (12, 15), (11, 20), (12, 20), (19, 20), (14, 20),   # extra attempts (often fail, cheap)
]

GATE = 0.15            # min(bg_fitness, animal_fitness) acceptance
MIN_CORRS = 30
COHERENCE_MIN = 0.15   # bridge certification threshold (aliases ~0.03-0.04; raised from
                       # 0.12 after two barely-over-threshold false bridges on bf_03_11_2026)
VOXEL = 0.02
ICP_DIST = 0.03
TEASER_NOISE = 0.03


def load_intr(path):
    v = np.loadtxt(path)
    return v[2], v[3], v[4], v[5]


def window_depth(d, us, vs, r=2):
    out = np.zeros(len(us), dtype=np.float32)
    for i, (u, v) in enumerate(zip(us, vs)):
        w = d[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1]
        w = w[w > 0]
        out[i] = np.median(w) if len(w) else 0
    return out


class AnimalViews:
    def __init__(self, session_dir, animal):
        self.animal = animal
        self.adir = os.path.join(session_dir, animal)
        self.views = []
        self.depth, self.intr, self.mask, self.jpg = {}, {}, {}, {}
        for v in VIEWS:
            stem = os.path.join(self.adir, f"{animal}_nano_{v}")
            if all(os.path.exists(stem + s) for s in [".jpg", "_depth.png", "_mask.png", "_intrinsics.txt"]):
                self.views.append(v)
                self.jpg[v] = stem + ".jpg"
                self.depth[v] = np.array(Image.open(stem + "_depth.png"))
                self.mask[v] = np.array(Image.open(stem + "_mask.png").convert("L"))
                self.intr[v] = load_intr(stem + "_intrinsics.txt")
        self._clouds = {}

    def cloud(self, v, region="full"):
        key = (v, region)
        if key not in self._clouds:
            d = self.depth[v].astype(np.float32) / 1000.0
            fx, fy, cx, cy = self.intr[v]
            sel = d > 0
            if region == "animal":
                sel &= self.mask[v] < 250
            elif region == "bg":
                sel &= self.mask[v] >= 250
            vs, us = np.where(sel)
            z = d[vs, us]
            p = o3d.geometry.PointCloud()
            p.points = o3d.utility.Vector3dVector(
                np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], 1))
            self._clouds[key] = p.voxel_down_sample(VOXEL)
        return self._clouds[key]

    def lift(self, k, v):
        d = self.depth[v]
        fx, fy, cx, cy = self.intr[v]
        H, W = d.shape
        us = np.clip(np.round(k[:, 0]).astype(int), 0, W - 1)
        vs = np.clip(np.round(k[:, 1]).astype(int), 0, H - 1)
        z = window_depth(d, us, vs) / 1000.0
        return np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], 1), z > 0


def register_pair(av, matcher, imgs, i, j):
    """RoMa -> lift -> TEASER -> ICP. Returns dict or None."""
    res = matcher(imgs[i], imgs[j])
    k_i = np.asarray(res["matched_kpts0"])
    k_j = np.asarray(res["matched_kpts1"])
    p_i, v_i = av.lift(k_i, i)
    p_j, v_j = av.lift(k_j, j)
    sel = v_i & v_j
    n = int(sel.sum())
    if n < MIN_CORRS:
        return None
    solver = get_teaser_solver(TEASER_NOISE)
    solver.solve(p_i[sel].T, p_j[sel].T)
    s = solver.getSolution()
    icp = o3d.pipelines.registration.registration_icp(
        av.cloud(i), av.cloud(j), ICP_DIST, Rt2T(s.rotation, s.translation),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
    T = np.array(icp.transformation)
    fit_an = o3d.pipelines.registration.evaluate_registration(
        av.cloud(i, "animal"), av.cloud(j, "animal"), ICP_DIST, T).fitness
    fit_bg = o3d.pipelines.registration.evaluate_registration(
        av.cloud(i, "bg"), av.cloud(j, "bg"), ICP_DIST, T).fitness
    return {"T": T, "n": n, "fit_full": icp.fitness,
            "fit_animal": fit_an, "fit_bg": fit_bg,
            "gate": min(fit_an, fit_bg)}


def components(nodes, edges):
    seen, comps = set(), []
    adj = {v: set() for v in nodes}
    for (i, j) in edges:
        adj[i].add(j)
        adj[j].add(i)
    for start in nodes:
        if start in seen:
            continue
        comp, stack = set(), [start]
        while stack:
            u = stack.pop()
            if u in comp:
                continue
            comp.add(u)
            stack.extend(adj[u] - comp)
        seen |= comp
        comps.append(comp)
    return comps


def recover_animal(session_dir, animal, matcher, out_dir):
    t0 = time.time()
    av = AnimalViews(session_dir, animal)
    result = {"animal": animal, "views_present": av.views, "edges": {}, "warnings": []}
    if WORLD not in av.views:
        result["warnings"].append("view 20 (world) missing - cannot anchor")
        return None, result

    imgs = {v: matcher.load_image(av.jpg[v]) for v in av.views}
    E = {}
    for (i, j) in PAIRS:
        if i not in av.views or j not in av.views:
            continue
        try:
            e = register_pair(av, matcher, imgs, i, j)
        except Exception as ex:
            result["warnings"].append(f"pair ({i},{j}) failed: {ex}")
            continue
        if e is None:
            continue
        E[(i, j)] = e
        result["edges"][f"{i}-{j}"] = {k: round(float(x), 4) for k, x in e.items() if k != "T"}

    accepted = {k: e for k, e in E.items() if e["gate"] >= GATE}

    def eT(i, j):
        if (i, j) in E:
            return E[(i, j)]["T"]
        return np.linalg.inv(E[(j, i)]["T"])

    # connectivity from world
    comps = components(av.views, accepted.keys())
    world_comp = next(c for c in comps if WORLD in c)

    # compose accepted edges within reach of world (Dijkstra, cost = 1/gate)
    import heapq

    def dijkstra_tree(edge_set):
        adj = {v: [] for v in av.views}
        for (i, j) in edge_set:
            c = 1.0 / max(E[(i, j)]["gate"], 1e-3)
            adj[i].append((j, c))
            adj[j].append((i, c))
        dist = {v: np.inf for v in av.views}
        dist[WORLD] = 0
        prev, pq = {}, [(0.0, WORLD)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            for w, c in adj[u]:
                if d + c < dist[w]:
                    dist[w] = d + c
                    prev[w] = u
                    heapq.heappush(pq, (dist[w], w))
        T = {WORLD: np.eye(4)}
        for v in sorted([x for x in av.views if np.isfinite(dist[x]) and x != WORLD],
                        key=lambda x: dist[x]):
            T[v] = T[prev[v]] @ eT(v, prev[v])
        return T, prev

    Tworld, prev = dijkstra_tree(accepted.keys())

    # bridge unreached clusters via best candidate edge + animal-coherence certification
    bridges_used = []
    for comp in comps:
        if WORLD in comp or not (comp & set(Tworld)):
            pass
        if WORLD in comp:
            continue
        # local chain within the cluster (accepted edges only)
        anchor = min(comp)
        candidates = [(k, e) for k, e in E.items()
                      if k not in accepted
                      and ((k[0] in comp) != (k[1] in comp))
                      and (k[0] in Tworld or k[1] in Tworld or k[0] in comp or k[1] in comp)]
        candidates = [(k, e) for k, e in candidates
                      if (k[0] in comp and k[1] in Tworld) or (k[1] in comp and k[0] in Tworld)]
        candidates.sort(key=lambda ke: -(ke[1]["fit_animal"] + ke[1]["fit_bg"]))
        # local transforms to anchor
        local_edges = [k for k in accepted if k[0] in comp and k[1] in comp]
        Tlocal = {anchor: np.eye(4)}
        changed = True
        while changed:
            changed = False
            for (i, j) in local_edges:
                # Tlocal[v] maps view-v points into the anchor frame:
                # v -> neighbor (eT(v, n)) then neighbor -> anchor (Tlocal[n])
                if i in Tlocal and j not in Tlocal:
                    Tlocal[j] = Tlocal[i] @ eT(j, i)
                    changed = True
                elif j in Tlocal and i not in Tlocal:
                    Tlocal[i] = Tlocal[j] @ eT(i, j)
                    changed = True
        # world-side animal assembly
        assembly = o3d.geometry.PointCloud()
        for v in Tworld:
            c = o3d.geometry.PointCloud(av.cloud(v, "animal"))
            c.transform(Tworld[v])
            assembly += c
        assembly = assembly.voxel_down_sample(VOXEL)

        # evaluate ALL candidates, keep the best-coherence one (first-over-threshold
        # previously picked mediocre bridges ahead of good ones)
        best = None
        for (k, e) in candidates:
            ci, wj = (k[0], k[1]) if k[0] in comp else (k[1], k[0])
            T_bridge = (Tworld[wj] @ eT(ci, wj)) @ np.linalg.inv(Tlocal[ci])
            cands_T = {v: T_bridge @ Tlocal[v] for v in Tlocal}
            fits = []
            for v in cands_T:
                r = o3d.pipelines.registration.evaluate_registration(
                    av.cloud(v, "animal"), assembly, ICP_DIST, cands_T[v])
                fits.append(r.fitness)
            score = float(np.mean(fits))
            if best is None or score > best[0]:
                best = (score, k, e, cands_T)
        if best is not None and best[0] >= COHERENCE_MIN:
            score, k, e, cands_T = best
            for v in cands_T:
                Tworld[v] = cands_T[v]
                prev[v] = None  # bridged, not tree
            accepted[k] = e
            bridges_used.append({"edge": f"{k[0]}-{k[1]}",
                                 "coherence": round(score, 3)})
        else:
            result["warnings"].append(f"cluster {sorted(comp)} unreachable - no certified bridge"
                                      + (f" (best coherence {best[0]:.3f} via {best[1]})" if best else ""))

    result["bridges"] = bridges_used

    # pose-graph optimization over accepted edges
    reach = [v for v in av.views if v in Tworld]
    idx = {v: i for i, v in enumerate(reach)}
    pg = o3d.pipelines.registration.PoseGraph()
    for v in reach:
        pg.nodes.append(o3d.pipelines.registration.PoseGraphNode(Tworld[v].copy()))
    tree_pairs = {(v, p) for v, p in prev.items() if p is not None}
    for (i, j), e in accepted.items():
        if i not in idx or j not in idx:
            continue
        uncertain = not ((i, j) in tree_pairs or (j, i) in tree_pairs)
        info = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            av.cloud(i), av.cloud(j), ICP_DIST, e["T"])
        pg.edges.append(o3d.pipelines.registration.PoseGraphEdge(
            idx[i], idx[j], e["T"], info, uncertain=uncertain))
    o3d.pipelines.registration.global_optimization(
        pg,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=ICP_DIST,
            edge_prune_threshold=0.25,
            reference_node=idx[WORLD]))
    norm = np.linalg.inv(pg.nodes[idx[WORLD]].pose)
    Topt = {v: norm @ np.array(pg.nodes[idx[v]].pose) for v in reach}

    # final GT-free quality: each view vs assembly of all OTHER views
    quality = {}
    for v in reach:
        assembly = o3d.geometry.PointCloud()
        for u in reach:
            if u == v:
                continue
            c = o3d.geometry.PointCloud(av.cloud(u, "animal"))
            c.transform(Topt[u])
            assembly += c
        assembly = assembly.voxel_down_sample(VOXEL)
        r = o3d.pipelines.registration.evaluate_registration(
            av.cloud(v, "animal"), assembly, ICP_DIST, Topt[v])
        quality[v] = round(float(r.fitness), 3)
    result["final_fitness_per_view"] = quality
    result["bridged_views"] = sorted(v for v in reach if v != WORLD and prev.get(v, "t") is None)
    result["unreached_views"] = sorted(set(av.views) - set(reach))
    result["runtime_s"] = round(time.time() - t0, 1)

    # save
    os.makedirs(out_dir, exist_ok=True)
    aid = animal.replace("Animal_", "")
    for v in reach:
        np.save(os.path.join(out_dir, f"A{aid}_T{T_INDEX[v]}.npy"), Topt[v])
    with open(os.path.join(out_dir, f"A{aid}_quality.json"), "w") as f:
        json.dump(result, f, indent=1)
    return Topt, result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True)
    ap.add_argument("--animals", default=None, help="comma-separated Animal_ dirs")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--out", default="teaser_reg_results_roma")
    args = ap.parse_args()

    if args.all:
        animals = sorted(a for a in os.listdir(args.session)
                         if a.startswith("Animal_") and
                         os.path.isdir(os.path.join(args.session, a)))
    else:
        animals = args.animals.split(",")

    out_dir = os.path.join(args.session, args.out)
    from vismatch import get_matcher
    matcher = get_matcher("roma", device="cuda")

    for animal in animals:
        print(f"\n===== {animal} =====", flush=True)
        try:
            Topt, res = recover_animal(args.session, animal, matcher, out_dir)
        except Exception as ex:
            print(f"  FAILED: {type(ex).__name__}: {ex}", flush=True)
            continue
        if Topt is None:
            print(f"  skipped: {res['warnings']}", flush=True)
            continue
        fits = res["final_fitness_per_view"]
        print(f"  {len(Topt)}/{len(res['views_present'])} views registered "
              f"in {res['runtime_s']}s; fitness min={min(fits.values()):.3f} "
              f"mean={np.mean(list(fits.values())):.3f}", flush=True)
        if res["bridges"]:
            print(f"  bridges: {res['bridges']}", flush=True)
        for w in res["warnings"]:
            print(f"  WARNING: {w}", flush=True)


if __name__ == "__main__":
    main()
