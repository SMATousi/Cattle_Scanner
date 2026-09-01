import cv2
import numpy as np
import open3d as o3d
import json
import glob
import matplotlib.pyplot as plt
import copy
import teaserpp_python
from numpy.linalg import inv
from scipy.spatial import cKDTree
import time
from PIL import Image
import torchvision
import torch
from torchvision import transforms as T 
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor
from numba import jit, prange
import os
import csv 
# Load MaskRCNN 
from segment_anything import sam_model_registry, SamPredictor
import subprocess
from pathlib import Path
import contextlib
from collections import defaultdict

# Open3D tensor (GPU) API -- used by the optional VoxelBlockGrid TSDF path.
try:
    import open3d.core as o3c
    _O3D_CUDA_OK = o3c.cuda.is_available()
except Exception:
    o3c = None
    _O3D_CUDA_OK = False


# def gamma_correction(img, gamma=1.8):
#     inv = 1.0 / gamma
#     table = np.array([(i / 255.0) ** inv * 255
#                       for i in range(256)]).astype("uint8")
#     return cv2.LUT(img, table)

def gamma_correct_image(in_path, gamma=2.0):
    # load image (BGR)
    img = cv2.imread(in_path, cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"Could not load image: {in_path}")

    # build LUT
    inv = 1.0 / gamma
    table = np.array(
        [(i / 255.0) ** inv * 255 for i in range(256)],
        dtype="uint8"
    )

    # apply gamma correction
    img_gamma = cv2.LUT(img, table)

#     # ensure output directory exists
#     os.makedirs(os.path.dirname(out_path), exist_ok=True)

    # write to disk
    cv2.imwrite(in_path, img_gamma)

    return in_path



def reg_icp(source, target, trans_init,threshold ):
#     threshold = 0.1
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=10000))
    draw_registration_result(source, target, reg_p2p.transformation)
    return reg_p2p.transformation

def draw_geometries(pcd, title="Open3D Viewer"):
    vis = o3d.visualization.Visualizer()

    # Create window with custom title
    vis.create_window(window_name=title)

    vis.add_geometry(pcd)

    # Rendering options
    opt = vis.get_render_option()
    opt.point_size = 1.0            # Your point size
    opt.background_color = [1, 1, 1]  # White background (RGB in [0,1])

    vis.run()
    vis.destroy_window()

def save_transforms(transforms, save_dir, prefix="T"):
    """
    Saves a list of 4x4 transformation matrices to .npy files.

    Parameters:
        transforms (list of np.ndarray): Each matrix must be shape (4,4).
        save_dir (str): Path to output directory.
        prefix (str): Prefix to use for filenames, e.g. "T" → T_0.npy, T_1.npy.
    """
    # Ensure directory exists
    os.makedirs(save_dir, exist_ok=True)

    for i, T in enumerate(transforms):
        if not isinstance(T, np.ndarray):
            raise TypeError(f"Transform {i} is not a numpy array")

        if T.shape != (4,4):
            raise ValueError(f"Transform {i} must be shape (4,4), got {T.shape}")

        filename = os.path.join(save_dir, f"{prefix}{i}.npy")
        np.save(filename, T)

    print(f"Saved {len(transforms)} transforms → {save_dir}")

def load_transforms(load_dir, prefix="T"):
    """
    Loads a list of 4x4 transformation matrices from .npy files.

    Parameters:
        load_dir (str): Path to directory containing transform files.
        prefix (str): Prefix used for filenames, e.g. "T" -> T0.npy, T1.npy.
    """
    file_list = sorted(glob.glob(os.path.join(load_dir, prefix + "*.npy")))
    transforms = []

    for filename in file_list:
        print(f"Loading {filename}")
        transform = np.load(filename)
        if transform.shape != (4, 4):
            raise ValueError(f"Transform {filename} must be shape (4,4), got {transform.shape}")
        transforms.append(transform)

    print(f"Loaded {len(transforms)} transforms from → {load_dir}")
    return transforms

# ---------------------------------------------------------------------------
# SAM 3 text-prompted segmentation
# ---------------------------------------------------------------------------
# "cow" under-scores very dark / black cattle: SAM 3 still detects them (right
# shape, ~210k px) but at score ~0.10, so the default 0.5 threshold filters them
# out and you get an all-white mask. The lighter / brown animals score 0.86-0.94
# and pass fine. Fix: also try "the animal" (fires on the same black cattle at
# 0.70-0.89) and pick the highest-scoring detection across prompts, scanning at a
# low confidence so nothing is pre-filtered. Scanning prompts + picking by score
# is more robust than just lowering the global threshold and keeping "cow".
SAM3_PROMPTS = ["cow", "the animal", "cattle"]  # tried in order; "the animal" rescues black cattle
SAM3_SCAN_CONF = 0.1     # low, so low-scoring dark-cow dets aren't pre-filtered
SAM3_MIN_PIXELS = 2000   # reject specks
SAM3_ACCEPT_CONF = 0.5   # early-exit: once a det scores this high, stop scanning further prompts


def segment_animal(processor, pil_image):
    """Return the best-scoring animal mask as a bool [H, W] array (or None).

    Scans SAM3_PROMPTS in order and, across all of them, keeps the single
    highest-scoring detection whose area >= SAM3_MIN_PIXELS.
    processor = Sam3Processor(model).
    """
    state = processor.set_image(pil_image)
    best_mask, best_score = None, -1.0
    for prompt in SAM3_PROMPTS:
        # reset between prompts so results don't accumulate, and set the (low)
        # threshold *before* set_text_prompt so the forward pass keeps everything.
        processor.reset_all_prompts(state)
        processor.set_confidence_threshold(SAM3_SCAN_CONF, state)
        state = processor.set_text_prompt(prompt=prompt, state=state)

        masks, scores = state.get("masks"), state.get("scores")
        if masks is None or masks.shape[0] == 0:
            continue

        masks = masks.squeeze(1).bool().cpu().numpy()        # [N, H, W]
        scores = scores.detach().float().cpu().numpy()       # NOTE: bfloat16 -> float FIRST
        areas = masks.reshape(masks.shape[0], -1).sum(1)

        for i in scores.argsort()[::-1]:                     # highest score first
            if areas[i] >= SAM3_MIN_PIXELS:
                if scores[i] > best_score:
                    best_mask, best_score = masks[i], float(scores[i])
                break

        # Early-exit: a confident detection is almost always the right animal, so
        # skip the remaining prompts (the common light/brown case exits on "cow").
        # Dark/black cattle score ~0.10 on "cow" (below this), so they still fall
        # through to "the animal" as intended.
        if best_score >= SAM3_ACCEPT_CONF:
            break

    # Optional robustness (not needed in practice -- "the animal" handled every
    # case): if best_mask is None, SAM 3 will still take a box even when the text
    # concept won't fire, e.g.
    #     from sam3.model.box_ops import box_xywh_to_cxcywh
    #     from sam3.visualization_utils import normalize_bbox
    #     b = normalize_bbox(box_xywh_to_cxcywh(torch.tensor([x,y,w,h]).view(-1,4)), W, H).flatten().tolist()
    #     processor.reset_all_prompts(state)
    #     state = processor.add_geometric_prompt(state=state, box=b, label=True)
    # Cleanest box source on this calibrated rig: a neighboring view's mask
    # reprojected into the failing view, or the animal's depth blob.
    return best_mask  # bool [H, W]; None if nothing found


def get_sam3_mask(image, processor):
    """Return a float [1, H, W] mask (1 = animal, 0 = background).

    `image` may be a file path, a PIL image, or an HxWx3 numpy array. Accepting
    an already-decoded image lets callers skip a redundant read from disk.
    Kept at [1, H, W] so downstream (segment_images_modified) is a drop-in and
    needs no changes: it does swapaxes(0,2).swapaxes(0,1) -> [H, W, 1].
    """
    if isinstance(image, str):
        pil_image = Image.open(image).convert("RGB")
    elif isinstance(image, np.ndarray):
        pil_image = Image.fromarray(image).convert("RGB")
    else:                                   # already a PIL image
        pil_image = image.convert("RGB")
    best_mask = segment_animal(processor, pil_image)

    if best_mask is None:
        # nothing detected across all prompts -> empty mask (background stays white)
        w, h = pil_image.size
        print("[get_sam3_mask] WARNING: no animal detected")
        return np.zeros((1, h, w), dtype=np.float32)

    return best_mask[None, ...].astype(np.float32)   # [1, H, W]


globT = 0
def register_two_views_teaser(A_pcd_raw,B_pcd_raw,VOXEL_SIZE):
    
    VISUALIZE = True
    A_pcd = A_pcd_raw.voxel_down_sample(voxel_size=VOXEL_SIZE)
    B_pcd = B_pcd_raw.voxel_down_sample(voxel_size=VOXEL_SIZE)
    #if VISUALIZE:
     #   o3d.visualization.draw_geometries([A_pcd,B_pcd]) # plot downsampled A and B 

    A_xyz = pcd2xyz(A_pcd) # np array of size 3 by N
    B_xyz = pcd2xyz(B_pcd) # np array of size 3 by M

    print("Extracting FPFH features")
    # extract FPFH features
    A_feats = extract_fpfh(A_pcd,VOXEL_SIZE)
    B_feats = extract_fpfh(B_pcd,VOXEL_SIZE)
    print(A_feats.shape)
    print("Computing FPFH correspondences")
    # establish correspondences by nearest neighbour search in feature space
    corrs_A, corrs_B = find_correspondences(
        A_feats, B_feats, mutual_filter=True)
    A_corr = A_xyz[:,corrs_A] # np array of size 3 by num_corrs
    B_corr = B_xyz[:,corrs_B] # np array of size 3 by num_corrs

    num_corrs = A_corr.shape[1]
    print(f'FPFH generates {num_corrs} putative correspondences.')

    # visualize the point clouds together with feature correspondenc
    # robust global registration using TEASER++
    NOISE_BOUND = VOXEL_SIZE
    teaser_solver = get_teaser_solver(NOISE_BOUND)
    teaser_solver.solve(A_corr,B_corr)
    solution = teaser_solver.getSolution()
    R_teaser = solution.rotation
    t_teaser = solution.translation
    T_teaser = Rt2T(R_teaser,t_teaser)

    # Visualize the registration results
    A_pcd_T_teaser = copy.deepcopy(A_pcd).transform(T_teaser)
    #o3d.visualization.draw_geometries([A_pcd_T_teaser,B_pcd])

    # local refinement using ICP
    icp_sol = o3d.pipelines.registration.registration_icp(
          A_pcd, B_pcd, NOISE_BOUND, T_teaser,
          o3d.pipelines.registration.TransformationEstimationPointToPoint(),
          o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
    T_icp = icp_sol.transformation
    

    # visualize the registration after ICP refinement
    A_pcd_T_icp = copy.deepcopy(A_pcd).transform(T_icp)
    if VISUALIZE:
        Acopy = copy.deepcopy(A_pcd_T_icp).paint_uniform_color([0.0,0.0,1])
        Bcopy = copy.deepcopy(B_pcd).paint_uniform_color([1.0,0.0,0.0])
        o3d.visualization.draw_geometries([Acopy,Bcopy])
    tformed_A = copy.deepcopy(A_pcd_raw).transform(T_icp)
    res = o3d.geometry.PointCloud()
    res = tformed_A + B_pcd_raw
    
    return res,T_icp

def pcd2xyz(pcd):
    return np.asarray(pcd.points).T

def extract_fpfh(pcd, voxel_size):
    radius_normal = voxel_size * 2
    pcd.estimate_normals(
      o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
      pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return np.array(fpfh.data).T

def find_knn_cpu(feat0, feat1, knn=1, return_distance=False):
    feat1tree = cKDTree(feat1)
    dists, nn_inds = feat1tree.query(feat0, k=knn, workers=10)
    if return_distance:
        return nn_inds, dists
    else:
        return nn_inds

def find_correspondences(feats0, feats1, mutual_filter=True):
    nns01 = find_knn_cpu(feats0, feats1, knn=1, return_distance=False)
    corres01_idx0 = np.arange(len(nns01))
    corres01_idx1 = nns01

    if not mutual_filter:
        return corres01_idx0, corres01_idx1

    nns10 = find_knn_cpu(feats1, feats0, knn=1, return_distance=False)
    corres10_idx1 = np.arange(len(nns10))
    corres10_idx0 = nns10

    mutual_filter = (corres10_idx0[corres01_idx1] == corres01_idx0)
    corres_idx0 = corres01_idx0[mutual_filter]
    corres_idx1 = corres01_idx1[mutual_filter]

    return corres_idx0, corres_idx1

def get_teaser_solver(noise_bound):
    solver_params = teaserpp_python.RobustRegistrationSolver.Params()
    solver_params.cbar2 = 1.0
    solver_params.noise_bound = noise_bound
    solver_params.estimate_scaling = False
    solver_params.inlier_selection_mode = \
        teaserpp_python.RobustRegistrationSolver.INLIER_SELECTION_MODE.PMC_EXACT
    solver_params.rotation_tim_graph = \
        teaserpp_python.RobustRegistrationSolver.INLIER_GRAPH_FORMULATION.CHAIN
    solver_params.rotation_estimation_algorithm = \
        teaserpp_python.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
    solver_params.rotation_gnc_factor = 1.4
    solver_params.rotation_max_iterations = 10000
    solver_params.rotation_cost_threshold = 1e-16
    solver = teaserpp_python.RobustRegistrationSolver(solver_params)
    return solver

def Rt2T(R,t):
    T = np.identity(4)
    T[:3,:3] = R
    T[:3,3] = t
    return T 

def save_results(animal_id, data_path, mesh, pcd_downsampled, sa, v):
    # Save the mesh and PCD
    mesh_filename = os.path.join(data_path, f"{animal_id}_mesh.ply")
    pcd_filename = os.path.join(data_path, f"{animal_id}_pcd_downsampled.ply")
    
    #print(mesh_filename)
    #print(pcd_filename)
    o3d.io.write_triangle_mesh(mesh_filename, mesh)
    
    o3d.io.write_point_cloud(pcd_filename, pcd_downsampled)

    CSV_PATH = '/home/vigir3d/Datasets/cattle_scans/cow_measurements.csv'
    # Check if CSV file exists to decide whether to write headers
    write_header = not os.path.exists(CSV_PATH)
  # Save SA & V to the CSV file in append mode
    with open(CSV_PATH, 'a', newline='') as csvfile:
        fieldnames = ['Animal ID', 'SA', 'V']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        if write_header:
            writer.writeheader()

        writer.writerow({'Animal ID': animal_id, 'SA': sa, 'V': v})

        
@jit(nopython=True, parallel=True)
def numba_eliminate_flying_pixels(depth_image, ws, threshold):
    height, width = depth_image.shape
    result = np.zeros_like(depth_image, dtype=np.float64)
    
    for cy in prange(height):
        for cx in prange(width):
            x_start, x_end = max(0, cx - ws), min(width, cx + ws + 1)
            y_start, y_end = max(0, cy - ws), min(height, cy + ws + 1)
            window = depth_image[y_start:y_end, x_start:x_end]
            result[cy, cx] = np.sum(np.abs(window - depth_image[cy, cx]))

    return result


# ---------------------------------------------------------------------------
# Depth / point-cloud cleaning
# ---------------------------------------------------------------------------
# The original flying-pixel filter deletes points using a sum-of-abs-diffs over a
# window with one global threshold. That score scales with surface slant, depth
# magnitude, and window size, and it treats invalid (0) depth as a huge jump -- so
# it both over-removes valid slanted/distant/silhouette points and leaves per-pixel
# surface noise behind. The tools below separate "clean" from "remove":
#   * bilateral_smooth_depth      -> reduce ToF noise, keep edges, remove NO points
#   * numba_flying_pixel_maxjump  -> image-space removal, fixed to be scale-aware
#                                    and to ignore invalid neighbors (middle option)
#   * clean_pointcloud_sor        -> remove real flying-pixel tendrils in 3D (primary)
#
# Toggle via the CLEAN_* module constants (set them from the notebook to A/B, e.g.
# `import helpers; helpers.CLEAN_BILATERAL = False`). Defaults clean only the masked
# animal cloud; the full pcd_tsdf used for registration is left untouched.
CLEAN_BILATERAL = False    # edge-preserving depth smoothing; marginal here (raw depth is
                           # already ~2.6mm noise -> only ~12% reduction), so off by default
CLEAN_MAXJUMP   = False    # image-space max-jump flying-pixel removal (middle option)
CLEAN_SOR       = True     # statistical outlier removal on the masked cloud
CLEAN_RADIUS    = False    # radius outlier removal on the masked cloud

# bilateral: d = neighborhood diameter (px); sigmas in depth units and px.
# NOTE: in this pipeline the depth passed in is float METRES (rgbdc.depth), so the
# range sigma is in metres. ~0.03 m (3 cm) preserves the animal/background edge
# (a >1 m jump) while smoothing surface noise.
BILATERAL_D           = 5
BILATERAL_SIGMA_DEPTH = 0.03    # metres; neighbors within ~this depth are averaged
BILATERAL_SIGMA_SPACE = 5.0     # px

# max-jump filter: reject a pixel if its largest jump to a *valid* neighbor exceeds
# max(MAXJUMP_ABS, MAXJUMP_REL * depth). Threshold is in the SAME units as the depth
# passed in (metres in this pipeline).
MAXJUMP_WS  = 2
MAXJUMP_REL = 0.03      # 3% of the pixel's depth
MAXJUMP_ABS = 0.03      # metres floor

# statistical / radius outlier removal (Open3D)
SOR_NB_NEIGHBORS = 20
SOR_STD_RATIO    = 2.0
ROR_NB_POINTS = 16
ROR_RADIUS    = 0.02    # metres


@jit(nopython=True, parallel=True)
def numba_flying_pixel_maxjump(depth_image, ws, rel_thresh, abs_floor):
    """Flag flying pixels by the largest jump to a *valid* neighbor.

    Returns a bool mask (True = flying pixel). Unlike the SAD filter this is
    invariant to surface slant and window size, ignores invalid (<=0) neighbors so
    it does not carve the silhouette against background holes, and uses a
    depth-relative threshold so one setting works near and far.
    """
    height, width = depth_image.shape
    out = np.zeros((height, width), dtype=np.bool_)
    for cy in prange(height):                      # only the outer loop parallelizes in numba
        for cx in range(width):
            d0 = depth_image[cy, cx]
            if d0 <= 0:
                continue
            thr = abs_floor if abs_floor > rel_thresh * d0 else rel_thresh * d0
            maxjump = 0.0
            y0, y1 = max(0, cy - ws), min(height, cy + ws + 1)
            x0, x1 = max(0, cx - ws), min(width, cx + ws + 1)
            for yy in range(y0, y1):
                for xx in range(x0, x1):
                    dn = depth_image[yy, xx]
                    if dn <= 0:                    # ignore invalid neighbors
                        continue
                    j = dn - d0
                    if j < 0:
                        j = -j
                    if j > maxjump:
                        maxjump = j
            if maxjump > thr:
                out[cy, cx] = True
    return out


def bilateral_smooth_depth(depth_np):
    """Edge-preserving smoothing of a depth image (removes no points).

    Zeros (no return) are preserved so the filter can't invent depth in holes. The
    range kernel (BILATERAL_SIGMA_DEPTH) gives background/hole neighbors ~zero
    weight, so real edges and the silhouette are kept.
    """
    depth32 = depth_np.astype(np.float32)
    valid = depth32 > 0
    smoothed = cv2.bilateralFilter(
        depth32, BILATERAL_D, BILATERAL_SIGMA_DEPTH, BILATERAL_SIGMA_SPACE
    )
    out = np.where(valid, smoothed, 0.0)
    return out.astype(depth_np.dtype)


def clean_pointcloud_sor(pcd):
    """Remove flying-pixel tendrils / isolated specks in 3D while keeping dense surface."""
    if len(pcd.points) == 0:
        return pcd
    if CLEAN_SOR:
        pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=SOR_NB_NEIGHBORS, std_ratio=SOR_STD_RATIO
        )
    if CLEAN_RADIUS:
        pcd, _ = pcd.remove_radius_outlier(nb_points=ROR_NB_POINTS, radius=ROR_RADIUS)
    return pcd


def _run_colored_icp(source, target, initial_tform):
    voxel_radius = [0.04, 0.02, 0.01]
    max_iter = [50, 30, 14]
    current_transformation = initial_tform

    for scale in range(3):
        iters = max_iter[scale]
        radius = voxel_radius[scale]

        source_down = copy.deepcopy(source).voxel_down_sample(radius)
        target_down = copy.deepcopy(target).voxel_down_sample(radius)

        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source_down, target_down, radius, current_transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                              relative_rmse=1e-6,
                                                              max_iteration=iters))
        current_transformation = result_icp.transformation

    return current_transformation


def colored_ICP_teaser(source, target, initial_tform):
    return _run_colored_icp(source, target, initial_tform)


def colored_ICP(source, target):
    return _run_colored_icp(source, target, np.identity(4))

def backproject_o3d(rgbd_frame, intrinsics):
    
    rgbdc = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgbd_frame.color, rgbd_frame.depth, depth_trunc=4.0, convert_rgb_to_intensity=False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbdc, intrinsics)
    n_radius = 0.01*2.0
    pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=n_radius, max_nn=30))
    
    return pcd



def load_maskrcnn_model(model_path):
    model = torchvision.models.detection.maskrcnn_resnet50_fpn(weights="DEFAULT")
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, 2)
    in_features_mask = model.roi_heads.mask_predictor.conv5_mask.in_channels
    hidden_layer = 256
    model.roi_heads.mask_predictor = MaskRCNNPredictor(in_features_mask, hidden_layer, 2)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device)
    model.load_state_dict(torch.load(model_path))
    model.eval()

    return model


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def cluster_point_cloud_new(outlier_cloud):
    cloud_colors = copy.deepcopy(np.asarray(outlier_cloud.colors).T)
    
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error) as cm:
        labels = np.array(outlier_cloud.cluster_dbscan(eps=0.05, min_points=10, print_progress=False))

    # Identify the largest cluster
    values, counts = np.unique(labels, return_counts=True)
    ind = np.argmax(counts)
    largest_cluster_label = values[ind]
    #print(f"Largest cluster label: {largest_cluster_label}")

    # Filter points, normals, and colors for the largest cluster
    cloud_xyz = pcd2xyz(outlier_cloud)
    cloud_normals = pcd2normals(outlier_cloud)

    cloud_filtered = cloud_xyz[:, labels == largest_cluster_label]
    normals_filtered = cloud_normals[:, labels == largest_cluster_label]
    colors_filtered = cloud_colors[:, labels == largest_cluster_label]

    # Create a point cloud for the largest cluster
    pcd_filtered_largest_cluster = o3d.geometry.PointCloud()
    pcd_filtered_largest_cluster.points = o3d.utility.Vector3dVector(cloud_filtered.T)
    pcd_filtered_largest_cluster.normals = o3d.utility.Vector3dVector(normals_filtered.T)
    pcd_filtered_largest_cluster.colors = o3d.utility.Vector3dVector(colors_filtered.T)

    #o3d.visualization.draw_geometries([pcd_filtered_largest_cluster])
    return pcd_filtered_largest_cluster

def load_calibration(tform_path, num_transforms=5):
    transforms = [np.eye(4)]  # Initialize with identity matrix
    
    # Load transformations from file
    for i in range(1, num_transforms+1):
        filename = tform_path + f"H_0_{i}.txt"
        
        if not os.path.exists(filename):
            raise FileNotFoundError(f"The file {filename} does not exist!")
        
        transforms.append(np.loadtxt(filename))
    
    return transforms

def upsample_using_reference_normals_new(sparse_pcd, dense_pcd, search_radius=0.02, angle_threshold=30):
    # Ensure the point clouds have normals
    if not sparse_pcd.has_normals():
        sparse_pcd.estimate_normals()
    if not dense_pcd.has_normals():
        dense_pcd.estimate_normals()

    dense_points = np.asarray(dense_pcd.points)
    dense_tree = cKDTree(dense_points)

    sparse_points = np.asarray(sparse_pcd.points)
    sparse_normals = np.asarray(sparse_pcd.normals)
    sparse_colors = np.asarray(sparse_pcd.colors)

    
    # This retrieves the indices of all neighbors within the search_radius
    neighbor_indices = dense_tree.query_ball_point(sparse_points, search_radius, workers=-1)
    
    valid_indices = []
    for i, idx_row in enumerate(neighbor_indices):
        neighbors = dense_points[idx_row]
        neighbor_normals = np.asarray(dense_pcd.normals)[idx_row]
        neighbor_colors = np.asarray(dense_pcd.colors)[idx_row]

        # Calculate angles between sparse point's normal and all its neighbors' normals
        angles = np.degrees(np.arccos(np.clip(np.dot(sparse_normals[i], neighbor_normals.T), -1.0, 1.0)))

        # Filtering neighbors based on angle threshold
        valid_neighbor_indices = np.array(idx_row)[angles < angle_threshold]
        valid_indices.extend(valid_neighbor_indices)

    unique_valid_indices = np.unique(valid_indices)
    final_valid_neighbors = dense_points[unique_valid_indices]
    final_valid_normals = np.asarray(dense_pcd.normals)[unique_valid_indices]
    final_valid_colors = np.asarray(dense_pcd.colors)[unique_valid_indices]
    
    upsampled_points = np.vstack([sparse_points, final_valid_neighbors])
    upsampled_normals = np.vstack([sparse_normals, final_valid_normals])
    upsampled_colors = np.vstack([sparse_colors, final_valid_colors])
    
    upsampled_pcd = o3d.geometry.PointCloud()
    upsampled_pcd.points = o3d.utility.Vector3dVector(upsampled_points)
    upsampled_pcd.normals = o3d.utility.Vector3dVector(upsampled_normals)
    upsampled_pcd.colors = o3d.utility.Vector3dVector(upsampled_colors)
    
    return upsampled_pcd

def process_file_list_comp_no_seg(file_id, dpath,tform, model_rgb, model_depth, predictor,color_correct=False):
    global globT
    ws = 2
    # Format filenames
    dpath = dpath.rstrip('/')

    # Split by the directory separator and get the last part
    suffix = os.path.basename(dpath)
    
    file_suffix = suffix + "_nano_" + str(file_id)
    # Construct full file paths using os.path.join for better cross-platform compatibility
    depth_file = os.path.join(dpath, file_suffix + "_depth.png")
    color_file = os.path.join(dpath, file_suffix + ".jpg")
    if color_correct :
        color_file = gamma_correct_image(color_file, 2.0)
    intrinsics_file = os.path.join(dpath, file_suffix + "_intrinsics.txt")
    # Read the files
    depth = o3d.io.read_image(depth_file)
    color = o3d.io.read_image(color_file)
    #depth = o3d.io.read_image(dpath + 'Animal_box3_u_nano_11_depth.png')
    #color = o3d.io.read_image(dpath + 'Animal_box3_u_nano_11.jpg')
    K = np.loadtxt(intrinsics_file)
      

    camera_intrinsics = o3d.camera.PinholeCameraIntrinsic()
    camera_intrinsics.set_intrinsics(int(K[0]), int(K[1]), K[2], K[3], K[4], K[5])

    rgbdc = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color, depth, depth_trunc=4.0, convert_rgb_to_intensity=False
    )

    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=4.0 / 512.0, sdf_trunc=0.04, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
    )
    
    volume.integrate(rgbdc, camera_intrinsics, np.eye(4))
    pcd_tsdf = volume.extract_point_cloud()
    pcd_tmp = pcd_tsdf
#     pcd_tsdf.transform(tform)
#     depth_np = np.asarray(rgbdc.depth)
    
#     if ( depth_np.max() > 20 ) : # Depth is in millimeters
#         flying_pixel_filter_threshold = 1e5#100 # 100
#     else : # meters
#         flying_pixel_filter_threshold = 1e5# 0.1 # 0.1
#         #print(flying_pixel_filter_threshold, " -- ", depth_np.max())

#     result_mask = numba_eliminate_flying_pixels(depth_np.copy(), ws, flying_pixel_filter_threshold)
#     depth_np[result_mask > flying_pixel_filter_threshold] = 0
#     # Re-insert filtered depth into the rgbd image
#     rgbdc.depth = o3d.geometry.Image(depth_np)
#     # Apply maskrcnn to filtered depth image
#     t1 = time.time()
#     masked_rgbd = segment_images_modified(rgbdc, model_rgb, model_depth, predictor)
#     t2 = time.time()
#     T = t2-t1
#     globT+=T
#     #print("Duration for maskrcnn 1 image ",file_id, ": ", T , "s")
#     # necessary lol
#     rgbdc_new = o3d.geometry.RGBDImage.create_from_color_and_depth(
#                     masked_rgbd.color, masked_rgbd.depth, depth_trunc=4.0, convert_rgb_to_intensity=False)
#     d = np.asarray(rgbdc_new.depth)

#     v = o3d.pipelines.integration.ScalableTSDFVolume(
#         voxel_length=4.0 / 512.0, sdf_trunc=0.04, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
#     )

#     v.integrate(rgbdc_new, camera_intrinsics, np.eye(4))
#     pcd_tmp = v.extract_point_cloud()
#     pcd_tmp.transform(tform)
#     #pcd_tmp = o3d.geometry.PointCloud.create_from_rgbd_image(rgbdc_new,
#      #                                                   camera_intrinsics)

    return pcd_tmp, pcd_tsdf

def perform_pairwise_alignment(pcds_tsdf,pcds_cropped):
    """Compute and apply transformations."""
    t01 = colored_ICP(pcds_tsdf[0], pcds_tsdf[1])
    t52 = colored_ICP(pcds_tsdf[5], pcds_tsdf[2])
    t43 = colored_ICP(pcds_tsdf[4], pcds_tsdf[3])
    t12 = colored_ICP(pcds_tsdf[1], pcds_tsdf[2])
    t32 = colored_ICP(pcds_tsdf[3], pcds_tsdf[2])

    H0 = t12 @ t01
    H1 = t12
    H3 = t32
    H4 = t32 @ t43
    H5 = t52

    # Transform the point clouds
    p0 = copy.deepcopy(pcds_cropped[0]).transform(H0)
    p1 = copy.deepcopy(pcds_cropped[1]).transform(H1)
    p3 = copy.deepcopy(pcds_cropped[3]).transform(H3)
    p4 = copy.deepcopy(pcds_cropped[4]).transform(H4)
    p5 = copy.deepcopy(pcds_cropped[5]).transform(H5)
    
    d0 = copy.deepcopy(pcds_tsdf[0]).transform(H0)
    d1 = copy.deepcopy(pcds_tsdf[1]).transform(H1)
    d3 = copy.deepcopy(pcds_tsdf[3]).transform(H3)
    d4 = copy.deepcopy(pcds_tsdf[4]).transform(H4)
    d5 = copy.deepcopy(pcds_tsdf[5]).transform(H5)
    
    pcd_combined = o3d.geometry.PointCloud()
    pcd_combined = p0+p1+pcds_cropped[2]+p3+p4+p5
    
    ptsdf_combined = o3d.geometry.PointCloud()
    ptsdf_combined = d0+d1+pcds_tsdf[2]+d3+d4+d5

    return pcd_combined, ptsdf_combined
def pcd2normals(pcd):
    return np.asarray(pcd.normals).T


def call_cpp_program(directory_path):
    # Adjust this to the path of your compiled C++ executable if it's not in the system's PATH.
    cpp_executable = "/home/vigir3d/Software/programs/k4a-read-mkvs-synched/build/k4a_read_mkv"
    command = cpp_executable + " " + directory_path + "*.mkv"
    #print(command)
    # Call the C++ program.
    result = subprocess.run(command, shell=True, check=True)

    # Check if the C++ program executed without errors.
    if result.returncode != 0:
        print(f"Error running {cpp_executable}")
        return False

    return True

def combine_pcds(pcds):
    pcd_combined = o3d.geometry.PointCloud()
    for pcd in pcds :
        pcd_combined+=pcd
    #pcd_combined = pcds[0]+pcds[1]+pcds[2]+pcds[3]+pcds[4]+pcds[5]
    
    return pcd_combined

def transform_clouds(pcds, transforms):
    """
    Applies one 4x4 transform per point cloud and returns the combined result.
    """
    transformed = []

    for i, pcd in enumerate(pcds):
        transformed.append(copy.deepcopy(pcd).transform(transforms[i]))

    return combine_pcds(transformed)

def viz(pcds):
    o3d.visualization.draw_geometries(pcds)

# ---------------------------------------------------------------------------
# TSDF integration (legacy CPU vs. Open3D tensor / GPU) + per-stage timing
# ---------------------------------------------------------------------------
TSDF_VOXEL       = 4.0 / 512.0
TSDF_SDF_TRUNC   = 0.04
TSDF_DEPTH_TRUNC = 4.0

# Flip to True to integrate each frame on the GPU with VoxelBlockGrid instead of
# the CPU ScalableTSDFVolume (~4x faster per frame on this rig, geometry matches
# to ~1cm). Automatically falls back to legacy if CUDA/tensor Open3D is absent.
USE_TENSOR_TSDF = False
VBG_BLOCK_RESOLUTION = 16
VBG_BLOCK_COUNT      = 50000
VBG_WEIGHT_THRESHOLD = 0.5      # single-frame: each voxel is seen once (weight ~1.0)
_VBG_FELL_BACK = False          # so we only warn once


def _tsdf_pcd_from_rgbd(rgbd, camera_intrinsics, K, extrinsic=np.eye(4)):
    """Integrate one legacy RGBDImage into a TSDF and return a legacy point cloud.

    Legacy path == the original ScalableTSDFVolume behavior exactly. Tensor path
    runs VoxelBlockGrid on the GPU: it needs (uint16 mm depth, uint8 color), so the
    float-metre depth from `rgbd` is converted back to uint16 mm with depth_scale=1000.
    """
    global _VBG_FELL_BACK
    if USE_TENSOR_TSDF and _O3D_CUDA_OK:
        dev = o3c.Device("CUDA:0")
        depth_m = np.asarray(rgbd.depth)                       # float32 metres
        depth_mm = np.ascontiguousarray((depth_m * 1000.0).astype(np.uint16))
        color_np = np.ascontiguousarray(np.asarray(rgbd.color))  # uint8 HxWx3
        intr = o3c.Tensor([[K[2], 0, K[4]], [0, K[3], K[5]], [0, 0, 1]], o3c.float64)
        ext  = o3c.Tensor(np.asarray(extrinsic), o3c.float64)
        dt = o3d.t.geometry.Image(o3c.Tensor(depth_mm)).to(dev)
        ct = o3d.t.geometry.Image(o3c.Tensor(color_np)).to(dev)
        vbg = o3d.t.geometry.VoxelBlockGrid(
            ("tsdf", "weight", "color"),
            (o3c.float32, o3c.float32, o3c.float32),
            ((1), (1), (3)),
            TSDF_VOXEL, VBG_BLOCK_RESOLUTION, VBG_BLOCK_COUNT, dev,
        )
        fb = vbg.compute_unique_block_coordinates(dt, intr, ext, 1000.0, TSDF_DEPTH_TRUNC)
        vbg.integrate(fb, dt, ct, intr, intr, ext, 1000.0, TSDF_DEPTH_TRUNC)
        return vbg.extract_point_cloud(weight_threshold=VBG_WEIGHT_THRESHOLD).to_legacy()

    if USE_TENSOR_TSDF and not _O3D_CUDA_OK and not _VBG_FELL_BACK:
        print("[TSDF] USE_TENSOR_TSDF set but CUDA/tensor Open3D unavailable -- using legacy CPU path")
        _VBG_FELL_BACK = True

    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=TSDF_VOXEL, sdf_trunc=TSDF_SDF_TRUNC,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )
    volume.integrate(rgbd, camera_intrinsics, np.asarray(extrinsic))
    return volume.extract_point_cloud()


# Per-stage wall-clock timing. Use _stage("name") around a block; read/reset from
# the notebook via helpers.print_stage_times() / helpers.reset_stage_times().
STAGE_TIMES  = defaultdict(float)
STAGE_COUNTS = defaultdict(int)


def reset_stage_times():
    STAGE_TIMES.clear()
    STAGE_COUNTS.clear()


def print_stage_times():
    if not STAGE_TIMES:
        print("no stage timings recorded")
        return
    total = sum(STAGE_TIMES.values())
    print(f"{'stage':<14}{'total s':>10}{'calls':>8}{'ms/call':>10}{'%':>7}")
    for name, t in sorted(STAGE_TIMES.items(), key=lambda kv: -kv[1]):
        n = STAGE_COUNTS[name]
        print(f"{name:<14}{t:>10.3f}{n:>8}{1000*t/max(n,1):>10.1f}{100*t/max(total,1e-9):>6.0f}%")
    print(f"{'TOTAL':<14}{total:>10.3f}")


@contextlib.contextmanager
def _stage(name):
    t = time.perf_counter()
    try:
        yield
    finally:
        STAGE_TIMES[name]  += time.perf_counter() - t
        STAGE_COUNTS[name] += 1


def process_file_list_comp(file_id, dpath,tform, model_rgb, model_depth, predictor, use_sam=False,color_correct=False):
    global globT
    ws = 2
    # Format filenames
    #print(f"processing {file_id}")
    dpath = dpath.rstrip('/')

    # Split by the directory separator and get the last part
    suffix = os.path.basename(dpath)
    
    file_suffix = suffix + "_nano_" + str(file_id)
    # Construct full file paths using os.path.join for better cross-platform compatibility
    masked_fname = os.path.join(dpath, file_suffix +"_mask.png")
    depth_file = os.path.join(dpath, file_suffix + "_depth.png")
    color_file = os.path.join(dpath, file_suffix + ".jpg")
    
    if color_correct :
        color_file = gamma_correct_image(color_file, 2.0)
    
    intrinsics_file = os.path.join(dpath, file_suffix + "_intrinsics.txt")
    # Read the files
    with _stage("read"):
        depth = o3d.io.read_image(depth_file)
        color = o3d.io.read_image(color_file)
        K = np.loadtxt(intrinsics_file)# W,H,Fx,Fy,Cx,Cy

        camera_intrinsics = o3d.camera.PinholeCameraIntrinsic()
        camera_intrinsics.set_intrinsics(int(K[0]), int(K[1]), K[2], K[3], K[4], K[5])

        rgbdc = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_trunc=TSDF_DEPTH_TRUNC, convert_rgb_to_intensity=False
        )

    # full-body cloud for registration (left untouched by the depth cleaning below)
    with _stage("tsdf_full"):
        pcd_tsdf = _tsdf_pcd_from_rgbd(rgbdc, camera_intrinsics, K)
        pcd_tsdf.transform(tform)

    # Clean the depth that feeds the *masked* animal cloud. Bilateral smoothing
    # reduces ToF noise without dropping points; the max-jump filter (if enabled)
    # removes flying pixels image-side. pcd_tsdf above is intentionally not touched
    # so the teaser registration inputs stay identical -- flip CLEAN_* to A/B.
    with _stage("clean_depth"):
        depth_np = np.asarray(rgbdc.depth)
        depth_changed = False
        if CLEAN_BILATERAL:
            depth_np = bilateral_smooth_depth(depth_np)
            depth_changed = True
        if CLEAN_MAXJUMP:
            fp_mask = numba_flying_pixel_maxjump(depth_np, MAXJUMP_WS, MAXJUMP_REL, MAXJUMP_ABS)
            depth_np = depth_np.copy()
            depth_np[fp_mask] = 0
            depth_changed = True
        if depth_changed:
            rgbdc.depth = o3d.geometry.Image(depth_np)

    # Segment the animal (SAM3 or maskrcnn)
    t1 = time.time()
    with _stage("segment"):
        masked_rgbd = segment_images_modified(rgbdc, model_rgb, model_depth, predictor,masked_fname, color_file, use_sam)
    t2 = time.time()
    T = t2-t1
    globT+=T
    #print("Duration for maskrcnn 1 image ",file_id, ": ", T , "s")
    # necessary lol
    with _stage("tsdf_masked"):
        rgbdc_new = o3d.geometry.RGBDImage.create_from_color_and_depth(
                        masked_rgbd.color, masked_rgbd.depth, depth_trunc=TSDF_DEPTH_TRUNC, convert_rgb_to_intensity=False)
        pcd_tmp = _tsdf_pcd_from_rgbd(rgbdc_new, camera_intrinsics, K)

    # Remove flying-pixel tendrils / isolated specks in 3D (SOR is transform-invariant,
    # so cleaning before or after the transform is equivalent).
    with _stage("sor"):
        pcd_tmp = clean_pointcloud_sor(pcd_tmp)
    pcd_tmp.transform(tform)
    #mesh = v.extract_triangle_mesh()
    #mesh.transform(tform)
    return pcd_tmp, pcd_tsdf#, mesh

def segment_images_modified(last_frame, model_rgb, model_depth, predictor, m_fname, color_file=None, use_sam=False):
    depth_image_array = np.asarray(last_frame.depth)
    #print("Pre:",np.max(depth_image_array), " -- ", np.min(depth_image_array))
    rgb_image_array = np.asarray(last_frame.color)
    rgb_PIL = Image.fromarray(rgb_image_array)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
   # print(device)
    if use_sam :
        # pass the already-decoded RGB frame instead of re-reading color_file from disk
        rgb_mask = get_sam3_mask(rgb_PIL, predictor)
#         rgb_im_sam = cv2.imread(color_file)
#         rgb_im_sam = cv2.cvtColor(rgb_im_sam, cv2.COLOR_BGR2RGB)
#         rgb_mask = get_model_mask(model_rgb, rgb_PIL,  predictor, rgb_im_sam, use_sam)
        mask_combined = rgb_mask
        mask_combined = mask_combined.astype(rgb_image_array.dtype)
        #print(f"Using sam:{type(mask_combined)}")
    else :
        depth_PIL = Image.fromarray(depth_image_array).convert("RGB")
        rgb_mask = get_model_mask(model_rgb, rgb_PIL,  predictor)

        depth_mask = get_model_mask(model_depth, depth_PIL,  predictor)

        if depth_mask.nelement() == 0:
            mask_combined = rgb_mask
        else:
            mask_combined = depth_mask | rgb_mask  # 1 vote arbitration (OR the masks)
        
        #print(f"Not using sam:{type(mask_combined)}")

    # Convert tensor to numpy array and ensure the right datatype
        mask_combined = mask_combined.numpy().astype(rgb_image_array.dtype)
#     print(f" mask shape: {mask_combined.shape}")
    mask_image = mask_combined.swapaxes(0, 2).swapaxes(0, 1)
    mask_image = (mask_image > 0).astype(rgb_image_array.dtype)   

    fg_image_rgb = rgb_image_array * mask_image
    
    # for writing outputs
    if mask_image.ndim == 2:
        mask = mask_image[..., None]
    else:
        mask = mask_image

    mask = mask.astype(np.uint8)

    # ensure uint8
    fg_masked = fg_image_rgb
    if fg_masked.dtype != np.uint8:
        fg_masked = np.clip(fg_masked, 0, 255).astype(np.uint8)

    white = np.ones_like(fg_masked) * 255
    fg_white = fg_masked * mask + white * (1 - mask)
    Image.fromarray(fg_white, "RGB").save(m_fname)

    # For the depth image:
    squeezed_mask = np.squeeze(mask_image)
    fg_image_depth = (depth_image_array * squeezed_mask)*1000 # upscale because re-inserting in "last_frame" rescales
    
    #print("Post:" ,np.max(fg_image_depth), " -- ", np.min(fg_image_depth))
    last_frame.color = o3d.geometry.Image(fg_image_rgb)
    last_frame.depth = o3d.geometry.Image(fg_image_depth)

    return last_frame

def make_results_dir(base_path):
    # Define the full path for the "results" directory
    results_path_sam = os.path.join(base_path, "results_sam")
    
    results_path_maskrcnn = os.path.join(base_path, "results_maskrcnn")
    results_path_teaser = os.path.join(base_path, "teaser_reg_results")

    # Create the directory if it doesn't exist
    os.makedirs(results_path_sam, exist_ok=True)
    os.makedirs(results_path_maskrcnn, exist_ok=True)
    os.makedirs(results_path_teaser, exist_ok=True)

    print(f"Directory 'results' is ready at: {results_path_sam}")
    print(f"Directory 'results' is ready at: {results_path_teaser}")
    print(f"Directory 'results' is ready at: {results_path_maskrcnn}")
    return results_path_sam, results_path_maskrcnn,results_path_teaser
    
def get_folders_in_path(path):
    # List all entries in the given path
    entries = os.listdir(path)
    # Filter entries to include only directories
    folders = [entry for entry in entries if os.path.isdir(os.path.join(path, entry))]
    return folders


def get_model_mask(model_generic, image, predictor, sam_image=None, use_sam=False): # predictor is SAM
    proba_threshold = 0.5
    transform = T.ToTensor()
    ig = transform(image)
    device = "cuda"

    with torch.no_grad():
        prediction = model_generic([ig.to(device)])
        
    if(prediction[0]['masks'].nelement() == 0):
        XX = torch.empty((0,0), dtype=torch.int64)
        return XX
    predicted_mask = prediction[0]
    predicted_mask = predicted_mask['masks'][0] > proba_threshold
    
    predicted_mask = predicted_mask.squeeze(1)
    mask = predicted_mask.cpu().detach()
    #print(f"No sam: Returned mask type is : {type(mask)}, shape: {mask.shape}")
    box_coords = prediction[0]['boxes'].cpu().numpy()
    #print(len(box_coords)) 
    if len(box_coords > 1):
        box_coords = box_coords[0]
    #print(len(box_coords))
    #print(box_coords)
    if use_sam :
        predictor.set_image(sam_image)
        mask, _, _ = predictor.predict(
            point_coords=None,
            point_labels=None,
            box=box_coords,
            multimask_output=False,
        )
    
        #print(f"Sam: Returned mask type is : {type(mask)}, shape: {mask.shape}")
    return mask


def get_processed_sessions(dir_path):
    proc_sesssions = []
    proc_files = os.listdir(dir_path)
    for file in proc_files:
        if "sam" in file :
            proc_sess = file.replace("_sam_teaser.ply", "")
           # print(proc_sess)
            proc_sesssions.append(proc_sess)
    return proc_sesssions
