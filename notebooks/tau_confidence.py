"""Normal-consistency sample weighting tau(p) for screened Poisson reconstruction.

Implements Eq. (19)-(20) of Lin et al., "Calculating Volume of Pig Point Cloud Based on
Improved Poisson Reconstruction", Animals 2024, 14, 1210 (doi:10.3390/ani14081210).

WHY
    Poisson closes a watertight surface across regions it never observed. Where two real
    surfaces face each other across an occluded gap -- the udder and a hind leg, the inside
    of the legs, the muzzle -- it bridges them into one blob, and it drills tunnels through
    noisy patches. Measured on Animal_57 the production mesh has genus 43 (43 handles) and
    196 connected components.

WHAT
    tau(p) is a per-point confidence built purely from how much a point's normal agrees with
    its k nearest neighbours:

        tau(p) = (1/k) * sum_i g(cos theta_i),   g(x) = cos theta for theta in [0, 90 deg]
                                                 g(x) = 0          for theta > 90 deg

    On a clean surface patch all normals point the same way -> tau ~ 1. Across a gap the two
    facing surfaces have opposing normals -> tau -> 0, so those points stop pulling the
    isosurface through the void.

HOW IT REACHES POISSON
    PoissonRecon defines a point's confidence as the MAGNITUDE of its normal (see its
    `--confidence <exponent>` flag, weight = |n|^exponent). So we do not need to modify
    PoissonRecon at all: scale each unit normal by tau(p), write the cloud, and pass
    `--confidence 1`. `--confidence 0` reproduces the unweighted result exactly, which makes
    a free A/B control.

MEASURED (Animal_57, beef_farm_07_29_2026, depth 10, pointWeight 1.0, k=16)
    exponent   faces    comps  euler  genus   volume   area
    0.0 (ctrl) 708,520   180    -84     43    0.3808   4.952
    0.5        681,304    92    -28     15    0.3806   4.845
    1.0        661,348    50     -8      5    0.3806   4.776
    2.0        632,062    19    -10      6    0.3806   4.698
    4.0        582,258    10     -2      2    0.3806   4.605

    Genus 43 -> 5 at exponent 1.0. VOLUME IS INVARIANT across the whole sweep (0.3806), so if
    volume is the reported quantity the exponent does not matter -- use 1.0. Surface AREA
    falls monotonically (-7.7% at exponent 4), so high exponents are eroding real detail; do
    not push the exponent to "improve" genus if you also report area.

    NOT fixed by this: the vertical corrugations on the flank where the chute bars occlude
    every view. Those are missing data, and no point weighting can invent unobserved surface.

Usage (standalone):
    python tau_confidence.py --cloud <animal>_sam_teaser.ply --out <animal>_mesh.ply \
        [--k 16] [--confidence 1.0] [--depth 10] [--point-weight 1.0] [--keep-largest]

Usage (from the pipeline), replacing run_poisson_reconstruction:
    from tau_confidence import poisson_with_confidence
    poisson_with_confidence(cloud_ply, mesh_ply, depth=10, confidence=1.0, keep_largest=True)
"""

import argparse
import os
import subprocess
import sys

import numpy as np

POISSON_EXE = "/home/vigir3d/Software/packages/ccPoisson/PoissonRecon/Bin/Linux/PoissonRecon"

DEFAULT_K = 16            # neighbours used to judge normal agreement
DEFAULT_CONFIDENCE = 1.0  # PoissonRecon exponent on |n|; 0 disables (exact control)


def compute_tau(points: np.ndarray, normals: np.ndarray, k: int = DEFAULT_K) -> np.ndarray:
    """Per-point normal-consistency confidence in [0, 1]. Eq. (20) of the paper.

    Vectorised: one batched cKDTree query rather than a Python loop (0.1 s for 107 k points
    vs ~3 min).

    Args:
        points: (N, 3) point positions.
        normals: (N, 3) normals; need not be unit length, they are normalised here.
        k: number of nearest neighbours to compare against (self excluded).

    Returns:
        (N,) tau in [0, 1]. 1 = neighbours all agree, 0 = neighbours all oppose.
    """
    from scipy.spatial import cKDTree

    n = normals / np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    # k+1 because the query returns the point itself first; workers=-1 uses all cores.
    _, idx = cKDTree(points).query(points, k=k + 1, workers=-1)
    idx = idx[:, 1:]
    cos = np.einsum("ij,ikj->ik", n, n[idx])
    # g(x) = cos for theta <= 90 deg, 0 beyond: a neighbour facing away contributes nothing
    # rather than cancelling out an agreeing one.
    return np.maximum(cos, 0.0).mean(axis=1)


def write_ply_with_normals(path: str, points: np.ndarray, normals: np.ndarray) -> None:
    """Write an ASCII PLY preserving normal MAGNITUDE (that is the Poisson confidence).

    Open3D's writer is not used here because the magnitude carries the signal -- anything
    that re-normalises the normals silently discards tau.
    """
    with open(path, "w") as f:
        f.write(
            f"ply\nformat ascii 1.0\nelement vertex {len(points)}\n"
            "property float x\nproperty float y\nproperty float z\n"
            "property float nx\nproperty float ny\nproperty float nz\nend_header\n"
        )
        np.savetxt(f, np.c_[points, normals], fmt="%.6f")


def keep_largest_component(mesh_path: str, out_path: str | None = None) -> str:
    """Drop every connected component but the largest (Poisson emits ~180 stray shells).

    Cheap and near-zero risk: on Animal_57 the largest component is 98.9% of the faces and
    volume is unchanged to 4 decimal places.
    """
    import trimesh

    out_path = out_path or mesh_path
    tm = trimesh.load(mesh_path, process=False)
    cc = trimesh.graph.connected_components(tm.face_adjacency, min_len=1)
    if len(cc) > 1:
        tm = tm.submesh([list(max(cc, key=len))], append=True)
    tm.export(out_path)
    return out_path


def poisson_with_confidence(
    cloud_ply: str,
    out_ply: str,
    k: int = DEFAULT_K,
    confidence: float = DEFAULT_CONFIDENCE,
    depth: int = 10,
    b_type: int = 3,
    samples_per_node: float = 1.0,
    point_weight: float = 1.0,
    keep_largest: bool = False,
    verbose: bool = True,
):
    """tau-weighted screened Poisson. Drop-in for run_poisson_reconstruction.

    The input cloud MUST carry normals -- the TSDF path already provides them
    (`vol.extract_point_cloud()` returns oriented normals from the volumetric gradient).

    Returns:
        (out_ply, tau) so the caller can log or visualise the confidence field.
    """
    import open3d as o3d

    pcd = o3d.io.read_point_cloud(cloud_ply)
    if not pcd.has_normals():
        raise ValueError(f"{cloud_ply} has no normals; tau needs oriented normals")
    P = np.asarray(pcd.points)
    N = np.asarray(pcd.normals)
    N = N / np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-12)

    tau = compute_tau(P, N, k)
    if verbose:
        print(
            f"  tau (k={k}): median {np.median(tau):.3f} p05 {np.percentile(tau, 5):.3f} "
            f"| below 0.5: {(tau < 0.5).sum():,} ({100 * (tau < 0.5).mean():.2f}%)"
        )

    weighted = f"{os.path.splitext(out_ply)[0]}_tauCloud.ply"
    write_ply_with_normals(weighted, P, N * tau[:, None])

    cmd = [
        POISSON_EXE, "--in", weighted, "--out", out_ply,
        "--bType", str(b_type), "--depth", str(depth),
        "--samplesPerNode", str(samples_per_node),
        "--pointWeight", str(point_weight),
        "--confidence", str(confidence),
    ]
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if r.returncode != 0:
        print("Poisson failed:\n", r.stderr.decode()[-800:])
        raise RuntimeError("Poisson reconstruction failed.")

    if keep_largest:
        keep_largest_component(out_ply)
    if verbose:
        _report(out_ply)
    return out_ply, tau


def _report(mesh_path: str) -> None:
    """Print the topology/metric summary used throughout the ablation."""
    import trimesh

    tm = trimesh.load(mesh_path, process=False)
    cc = trimesh.graph.connected_components(tm.face_adjacency, min_len=1)
    largest = tm.submesh([list(max(cc, key=len))], append=True)
    print(
        f"  mesh: {len(tm.faces):,} faces | components {len(cc)} | "
        f"euler {largest.euler_number} genus {(2 - largest.euler_number) // 2} | "
        f"watertight {largest.is_watertight} | vol {largest.volume:.4f} area {largest.area:.3f}"
    )


def main():
    """CLI entry point."""
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cloud", required=True, help="input point cloud PLY (must have normals)")
    ap.add_argument("--out", required=True, help="output mesh PLY")
    ap.add_argument("--k", type=int, default=DEFAULT_K)
    ap.add_argument("--confidence", type=float, default=DEFAULT_CONFIDENCE,
                    help="PoissonRecon exponent on |n|; 0 = unweighted control")
    ap.add_argument("--depth", type=int, default=10)
    ap.add_argument("--point-weight", type=float, default=1.0)
    ap.add_argument("--keep-largest", action="store_true",
                    help="drop all but the largest connected component")
    ap.add_argument("--sweep", default=None,
                    help="comma-separated exponents to A/B, e.g. 0,0.5,1,2 "
                         "(tau is computed once and reused)")
    a = ap.parse_args()

    if a.sweep:
        for c in [float(x) for x in a.sweep.split(",")]:
            out = f"{os.path.splitext(a.out)[0]}_c{c}.ply"
            print(f"--- confidence {c}")
            poisson_with_confidence(a.cloud, out, k=a.k, confidence=c, depth=a.depth,
                                    point_weight=a.point_weight, keep_largest=a.keep_largest)
    else:
        poisson_with_confidence(a.cloud, a.out, k=a.k, confidence=a.confidence,
                                depth=a.depth, point_weight=a.point_weight,
                                keep_largest=a.keep_largest)


if __name__ == "__main__":
    sys.exit(main())
