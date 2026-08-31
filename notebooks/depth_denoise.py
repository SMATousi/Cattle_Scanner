"""Flying-pixel / mixed-pixel removal for the Kinect cattle rig.

Why this exists
---------------
The Azure Kinect is a ToF sensor: a pixel whose footprint straddles a depth
discontinuity returns a *blend* of the near and far return, so its depth lands
somewhere in between and the back-projected point floats in mid-air.  In the chute
the animal is imaged through a cage of steel bars ~0.5 m in front of it, so every
bar edge manufactures a curtain of these "flying pixels" strung between the bar and
the animal.  `transformation_depth_image_to_color_camera` (the k4a call that warps
depth into the 1920x1080 colour frame) smears them further, because it splats depth
triangles and the triangles that span an occlusion boundary get stretched.

SAM3 is *not* the problem -- overlaying the production masks on RGB shows the mask
carving correctly around the bars.  The problem is that the depth under a correct
mask is already contaminated, and the production fuse path
(`process_one_animal.fuse_tsdf` / `fuse_union`) applies **no depth cleaning at all**:
it goes straight from `depth[mask==0]=0` to TSDF integration.  Statistical outlier
removal cannot fix it either -- a flying-pixel curtain is locally *dense*, so SOR
sees a legitimate surface.

What this module does
---------------------
Two stages, in the order they must run:

  A. per-view, image space, on the FULL (un-masked) depth
     A1 `jump_mask`      reject pixels sitting on a depth cliff.  Must see the
                         un-masked depth: once the bars are zeroed out, the pixel
                         next to a bar has no valid neighbour and the cliff is
                         invisible -- which is why the existing
                         `helpers.numba_flying_pixel_maxjump` cannot help the
                         `process_one_animal` path.
     A2 `erode_mask`     shave the silhouette, where every pixel is a mixed pixel.
     A3 `depth_band_mask` drop connected components whose depth is nowhere near the
                         animal -- background leaking through mask slop, and chute
                         bars caught in front of it.  Runs *after* A1 so the mixed-
                         pixel ramp that bridges animal and background is already
                         cut and the leak is a separate component.

  B. on the fused cloud, 3D
     B1 `freespace_filter`  multi-view visibility.  A point that another camera can
                            see *through* -- i.e. that camera measured a surface
                            behind it along the same ray -- cannot be real.  This is
                            the one test that keys on the actual defect rather than
                            on local density, and with 10 cameras it has a lot of
                            evidence to work with.
     B2 `sor_filter`        mop up isolated specks.

Every stage is off unless you pass it a config; `DenoiseParams()` holds the tuned
defaults and `clean_view()` / `filter_cloud()` are the two entry points.
"""
from dataclasses import dataclass
import numpy as np

try:
    from scipy.ndimage import (maximum_filter, minimum_filter, binary_erosion,
                               label as cc_label)
    _HAVE_SCIPY = True
except ImportError:                                          # pragma: no cover
    _HAVE_SCIPY = False


# ---------------------------------------------------------------------------
@dataclass
class DenoiseParams:
    """Tuned on beef_farm_08_26_2026/Animal_N5122 (10-view chute rig, ~1.3 m standoff)."""
    # --- A1 max-jump -------------------------------------------------------
    jump_ws:      int   = 2      # half-window, px.  5x5 -> shaves ~2 px per cliff
    jump_rel:     float = 0.03   # 3 % of the pixel's own depth
    jump_abs:     float = 0.03   # metres; floor, so near surfaces aren't over-cut
    # --- A2 mask erosion ---------------------------------------------------
    erode_px:     int   = 2      # silhouette pixels are mixed pixels by construction
    # --- A3 depth-band component gate --------------------------------------
    band_conn:    float = 0.05   # m; two 4-neighbours join a component if |dz| < this
    band_min_px:  int   = 150    # components smaller than this are dropped outright
    band_near:    float = 0.60   # m in front of the anchor depth still counted animal
    band_far:     float = 0.90   # m behind it
    # --- B1 free-space carving --------------------------------------------
    fs_margin:    float = 0.05   # m; must clear registration error (~9 mm) + ToF noise
    fs_min_votes: int   = 2      # this many cameras must see through before we cut
    # --- B2 statistical outlier removal ------------------------------------
    sor_neighbors: int  = 20
    sor_std_ratio: float = 2.0


DEFAULT = DenoiseParams()


# ---------------------------------------------------------------------------
# A1 -- max jump to a valid neighbour
# ---------------------------------------------------------------------------
def jump_mask(depth, ws=DEFAULT.jump_ws, rel=DEFAULT.jump_rel, abs_floor=DEFAULT.jump_abs):
    """True where the pixel sits on a depth cliff (i.e. is a candidate mixed pixel).

    `depth` is metres, 0 = no return, and should be the FULL depth image -- not one
    that has already had the background zeroed, or the cliffs disappear.

    Scale-aware (threshold grows with depth) and blind to invalid neighbours, so it
    does not carve the silhouette against a hole the way a sum-of-abs-diffs score does.
    """
    if not _HAVE_SCIPY:
        raise RuntimeError("depth_denoise needs scipy")
    d = np.asarray(depth, np.float32)
    valid = d > 0
    size = 2 * ws + 1
    hi = maximum_filter(np.where(valid, d, -np.inf), size=size)
    lo = minimum_filter(np.where(valid, d, +np.inf), size=size)
    has = maximum_filter(valid, size=size)                # any valid neighbour at all
    jump = np.where(has, np.maximum(hi - d, d - lo), 0.0)
    jump[~np.isfinite(jump)] = 0.0
    thr = np.maximum(abs_floor, rel * d)
    return valid & (jump > thr)


# ---------------------------------------------------------------------------
# A2 -- silhouette erosion
# ---------------------------------------------------------------------------
def erode_mask(mask, px=DEFAULT.erode_px):
    if px <= 0:
        return mask
    return binary_erosion(mask, np.ones((2 * px + 1, 2 * px + 1), bool), border_value=0)


# ---------------------------------------------------------------------------
# A3 -- keep only the depth-connected components that are actually the animal
# ---------------------------------------------------------------------------
def depth_band_mask(depth, mask, p=DEFAULT):
    """Drop masked components whose depth puts them off the animal.

    The bars split the animal into several strips per view, so "keep the largest
    component" is wrong.  Instead: label 4-connected runs of masked pixels that are
    also depth-continuous, anchor on the *area-weighted median* depth of the mask,
    and keep every component whose median depth is inside the band around it.
    """
    d = np.asarray(depth, np.float32)
    m = mask & (d > 0)
    if not m.any():
        return m
    anchor = float(np.median(d[m]))

    # depth-continuous connected components: label on the mask, then split each label
    # wherever a 4-neighbour disagrees by more than band_conn.
    step_y = np.zeros_like(m)
    step_x = np.zeros_like(m)
    step_y[1:, :] = m[1:, :] & m[:-1, :] & (np.abs(d[1:, :] - d[:-1, :]) > p.band_conn)
    step_x[:, 1:] = m[:, 1:] & m[:, :-1] & (np.abs(d[:, 1:] - d[:, :-1]) > p.band_conn)
    seed = m & ~(step_y | step_x)
    lab, n = cc_label(seed, structure=np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]]))
    if n == 0:
        return m

    keep = np.zeros(n + 1, bool)
    idx = lab[lab > 0]
    dv = d[lab > 0]
    order = np.argsort(idx, kind="stable")
    idx, dv = idx[order], dv[order]
    bounds = np.searchsorted(idx, np.arange(1, n + 2))
    for c in range(1, n + 1):
        a, b = bounds[c - 1], bounds[c]
        if b - a < p.band_min_px:
            continue
        med = np.median(dv[a:b])
        keep[c] = (anchor - p.band_near) <= med <= (anchor + p.band_far)
    out = keep[lab]
    # pixels that were cut out as depth steps still belong to the animal if they sit
    # inside a kept component's depth band -- put them back so we don't punch holes.
    residual = m & (lab == 0)
    if residual.any():
        out |= residual & (np.abs(d - anchor) <= max(p.band_near, p.band_far))
    return out


# ---------------------------------------------------------------------------
# A -- one call per view
# ---------------------------------------------------------------------------
def clean_view(depth, mask, p=DEFAULT, stats=None):
    """Full per-view image-space clean.  Returns the kept boolean mask.

    depth : float32 metres, FULL frame (background not zeroed)
    mask  : bool, True = animal (SAM3 foreground)
    """
    m0 = mask & (depth > 0)
    n0 = int(m0.sum())
    m = m0
    if p.jump_ws > 0:
        m = m & ~jump_mask(depth, p.jump_ws, p.jump_rel, p.jump_abs)
    n1 = int(m.sum())
    m = erode_mask(m, p.erode_px)
    n2 = int(m.sum())
    m = depth_band_mask(depth, m, p)
    n3 = int(m.sum())
    if stats is not None:
        stats.update(n_in=n0, after_jump=n1, after_erode=n2, after_band=n3)
    return m


# ---------------------------------------------------------------------------
# B1 -- multi-view free-space (visibility) carving
# ---------------------------------------------------------------------------
def freespace_votes(points, views):
    """Per-point count of cameras that measured a surface BEHIND the point.

    `views` is a list of dicts: {'depth': HxW metres (FULL frame, un-masked),
    'K': (w,h,fx,fy,cx,cy), 'T': 4x4 world_from_cam}.  Using the un-masked depth is
    the whole point: a bar the mask threw away is still a legitimate free-space
    witness for everything in front of it, and the animal behind it is a witness for
    the flying pixels hanging off its edge.
    """
    P = np.asarray(points, np.float64)
    votes = np.zeros(len(P), np.int16)
    seen = np.zeros(len(P), np.int16)
    for vw in views:
        T = vw["T"]
        Ri = T[:3, :3].T
        ti = -Ri @ T[:3, 3]
        X = P @ Ri.T + ti
        z = X[:, 2]
        w, h, fx, fy, cx, cy = vw["K"]
        good = z > 0.2
        zz = np.where(good, z, 1.0)
        u = np.rint(X[:, 0] * fx / zz + cx).astype(np.int32)
        v = np.rint(X[:, 1] * fy / zz + cy).astype(np.int32)
        good &= (u >= 0) & (u < w) & (v >= 0) & (v < h)
        dm = np.zeros(len(P), np.float32)
        dm[good] = vw["depth"][v[good], u[good]]
        ok = good & (dm > 0)
        seen += ok
        votes += ok & (z < dm - vw.get("margin", DEFAULT.fs_margin))
    return votes, seen


def freespace_filter(pcd, views, p=DEFAULT, stats=None):
    """Drop points that >= fs_min_votes cameras can see straight through."""
    import open3d as o3d
    P = np.asarray(pcd.points)
    if len(P) == 0:
        return pcd
    for vw in views:
        vw.setdefault("margin", p.fs_margin)
    votes, seen = freespace_votes(P, views)
    keep = votes < p.fs_min_votes
    if stats is not None:
        stats.update(fs_in=len(P), fs_kept=int(keep.sum()),
                     fs_votes_hist=np.bincount(votes, minlength=6)[:6].tolist())
    return pcd.select_by_index(np.flatnonzero(keep))


# ---------------------------------------------------------------------------
# B2 -- statistical outlier removal
# ---------------------------------------------------------------------------
def sor_filter(pcd, p=DEFAULT, stats=None):
    if len(pcd.points) == 0:
        return pcd
    out, _ = pcd.remove_statistical_outlier(nb_neighbors=p.sor_neighbors,
                                            std_ratio=p.sor_std_ratio)
    if stats is not None:
        stats.update(sor_in=len(pcd.points), sor_kept=len(out.points))
    return out


def filter_cloud(pcd, views, p=DEFAULT, stats=None):
    """Stage B: free-space carving then SOR."""
    pcd = freespace_filter(pcd, views, p, stats)
    pcd = sor_filter(pcd, p, stats)
    return pcd
