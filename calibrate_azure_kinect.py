from dt_apriltags import Detector
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
import os
import time
from numba import jit, prange
import argparse
import subprocess



def detect_tag(img, k, tag_size):
    at_detector = Detector(families='tagStandard41h12',
                           nthreads=8,
                           quad_decimate=1.0,
                           quad_sigma=0.8,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)
    tags = at_detector.detect(img, True, k, tag_size)
    print("Detected:", len(tags), "tags")

    # Filter out tags with ID greater than 4
    filtered_tags = [tag for tag in tags if tag.tag_id <= 4]

    if filtered_tags:
        best_tag = min(filtered_tags, key=lambda tag: tag.pose_err)

        # create a 4x4 identity matrix
        H = np.eye(4)

        # set the rotation
        H[:3, :3] = best_tag.pose_R

        # set the translation
        H[:3, 3] = best_tag.pose_t[:, 0]

        # Print the tag IDs (excluding those greater than 4)
        #for tag in filtered_tags:
        #   print("Tag ID:", tag.tag_id)
    else:
        print("No tags detected or all tags have ID > 4.")
        best_tag = None
        H = None
    #Collect corners of the best tag for visualization
    tag_corners = None
    if best_tag:
        tag_corners = best_tag.corners

    return H, best_tag.tag_id if best_tag else None, tag_corners



def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


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

def eliminate_flying_pixels(depth_image, ws, threshold):
   
    # Get image size
    height, width = depth_image.shape
    # Create an empty array for the result
    result = np.zeros_like(depth_image, dtype=float)

    # Iterate over the entire image
    for cy in range(height):
        for cx in range(width):
            # Set the range for the window
            x_start, x_end = max(0, cx - ws), min(width, cx + ws + 1)
            y_start, y_end = max(0, cy - ws), min(height, cy + ws + 1)
            
            # Get the window
            window = depth_image[y_start:y_end, x_start:x_end]

            # Calculate the sum of absolute differences
            result[cy, cx] = np.sum(np.abs(window - depth_image[cy, cx]))
    count = np.sum(result > threshold)

    depth_image[result > threshold] = 0
    return  depth_image



def colored_ICP(source, target):
    
    #draw_registration_result(source,target,np.eye(4))
    voxel_radius = [0.04, 0.02, 0.01]
    max_iter = [50, 30, 14]
    current_transformation = np.identity(4)
    print("3. Colored point cloud registration")
    for scale in range(3):
        iters = max_iter[scale]
        radius = voxel_radius[scale]
        print("iteration: ", iters, radius, scale)

        print("3-1. Downsample with a voxel size %.2f" % radius)
        source_down = copy.deepcopy(source).voxel_down_sample(radius)
        target_down = copy.deepcopy(target).voxel_down_sample(radius)

        print("3-2. Estimate normal.")
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        print("3-3. Applying colored point cloud registration")
        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source_down, target_down, radius, current_transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                              relative_rmse=1e-6,
                                                              max_iteration=iters))
        current_transformation = result_icp.transformation
    
   
    #draw_registration_result(source, target, current_transformation)
    return current_transformation

def backproject_o3d(rgbd_frame, intrinsics):
    
    rgbdc = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgbd_frame.color, rgbd_frame.depth, depth_trunc=4.0, convert_rgb_to_intensity=False)


    # Create a point cloud from the RGBD image
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbdc, intrinsics)
    n_radius = 0.01*2.0
    pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=n_radius, max_nn=30))
    # Visualize the point cloud
    #o3d.visualization.draw_geometries([pcd])
    
    return pcd

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
    
    #for i in prange(height):
       # for j in prange(width):
      #      if result[i, j] > threshold:
     #           depth_image[i, j] = 0
    #count = np.sum(result > threshold)
    #print("Numba detected: #", count)
    return result

def load_filter_pcds(data_path,mode):  # returns a list of  pcds
    l = 0.3048  # replace with the actual value
    
    # define a dictionary where the keys are the tag_ids and the values are the transformation matrices
    transformations = {
        0: np.array([[0, 1, 0, 0],
                    [1, 0, 0, 0],
                    [0, 0, -1, l/2],
                    [0, 0, 0, 1]]),
        1: np.array([[0, 0, -1, l/2],
                    [1, 0, 0, 0],
                    [0, -1, 0, 0],
                    [0, 0, 0, 1]]),
        2: np.array([[-1, 0, 0, 0],
                    [0, 0, -1, l/2],
                    [0, -1, 0, 0],
                    [0, 0, 0, 1]]),
        3: np.array([[0, 0, 1, -l/2],
                    [-1, 0, 0, 0],
                    [0, -1, 0, 0],
                    [0, 0, 0, 1]]),
        4: np.array([[1, 0, 0, 0],
                    [0, 0, 1, -l/2],
                    [0, -1, 0, 0],
                    [0, 0, 0, 1]])
    }
    
    tag_size = 0.145    # assuming best_tag is defined and contains the tag with the smallest pose_err
   
    if mode.lower() == "body": # 10 cameras - swap 18 and 19
        file_ids = [11,12,15,19,18,20]  # exclude head cameras
    elif  mode.lower() == "normal": # 8 cameras
        file_ids = [11,12,14,16,17,18]  # exclude head cameras
    elif mode.lower() == "head":
        file_ids = [13,14,16,17]  # exclude head cameras
    
    list_size = len(file_ids)

    tag_poses = [None] * list_size  # assuming tag_poses and current_transform are dictionaries
    current_transform = [None] * list_size
    pcds = []
    data_path = data_path.rstrip('/')
    for i in range(len(file_ids)):
         # Split by the directory separator and get the last part
        suffix = os.path.basename(data_path)
        file_suffix = suffix + "_nano_" + str(file_ids[i])
        # Construct full file paths using os.path.join for better cross-platform compatibility
        depth_file = os.path.join(data_path, file_suffix + "_depth.png")
        color_file = os.path.join(data_path, file_suffix + ".jpg")
        intrinsics_file = os.path.join(data_path, file_suffix + "_intrinsics.txt")
        # Read the files
        depth = o3d.io.read_image(depth_file)
        color = o3d.io.read_image(color_file)
        K = np.loadtxt(intrinsics_file)
        # width,height,fx,fy,cx,cy
        # 0,    1,     2,  3, 4, 5

        camera_intrinsics = o3d.camera.PinholeCameraIntrinsic()
        fx = K[2]
        fy = K[3]
        cx = K[4]
        cy = K[5]
        camera_intrinsics.set_intrinsics(int(K[0]), int(K[1]), fx,fy,cx,cy)
        K = (fx,fy,cx,cy)
        rgb_im_np = np.asarray(color)
    
        gray_img = cv2.cvtColor(rgb_im_np, cv2.COLOR_RGB2GRAY)
        transform, tag_id, tag_corners = detect_tag(gray_img,K, tag_size)
        # Drawing tag corners on the RGB image if detected
        if tag_corners is not None:
            tag_corners_int = np.int0(tag_corners)  # Convert to integer
            rgb_im_np_debug = cv2.polylines(rgb_im_np, [tag_corners_int], True, (0, 255, 0), 2)

        # Save the image with detections
        debug_image_path = os.path.join(data_path, f"detected_{file_suffix}.jpg")
        cv2.imwrite(debug_image_path, rgb_im_np_debug)

            
        # get the transformation matrix for the given tag_id
        cube_transform = transformations.get(tag_id, np.eye(4))  # defaults to identity matrix if tag_id is not found

        tag_poses[i] = np.dot(transform, np.linalg.inv(cube_transform))
        
        # assuming tag_poses[0] is defined and is the pose of the first camera
        tag_pose = np.dot(tag_poses[0], np.linalg.inv(tag_poses[i]))

        current_transform[i] = tag_pose  # current_transform is now the tag pose
        
        # WRITE THE TRANSFORMATIONS
        fname_tform = "H_0_" + str(i) + ".txt"
        np.savetxt(os.path.join(data_path,fname_tform), current_transform[i])
        rgbdc = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_trunc=4.0, convert_rgb_to_intensity=False
        )

        volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=4.0 / 512.0, sdf_trunc=0.04, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
        )
    
        volume.integrate(rgbdc, camera_intrinsics, np.eye(4))
        pcd = volume.extract_point_cloud()
      
        pcds.append(copy.deepcopy(pcd).transform(current_transform[i]))
        #o3d.visualization.draw_geometries(pcds)

    return pcds, current_transform
   

def compute_and_orient_normals(pcd, voxel_size):
    normal_radius = voxel_size*2.0
    pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(normal_radius=0.1, max_nn=30))


def perform_pairwise_alignment_head(pcds):
    
    t10 = colored_ICP(pcds[1],pcds[0])
    t23 = colored_ICP(pcds[2],pcds[3])
    t30 = colored_ICP(pcds[3],pcds[0])

    h0 = np.eye(4)
    h1 = t10
    h2 = t30 @ t23
    h3 = t30

    p0 = copy.deepcopy(pcds[0]).transform(h0)
    p1 = copy.deepcopy(pcds[1]).transform(h1)
    p2 = copy.deepcopy(pcds[2]).transform(h2)
    p3 = copy.deepcopy(pcds[3]).transform(h3)
    
    o3d.visualization.draw_geometries([p0,p1,p2,p3])

def perform_pairwise_alignment(pcds,mode):
    #o3d.visualization.draw_geometries(pcds)

    if mode == "head":
        perform_pairwise_alignment_head(pcds)
        return pcds

    t01= colored_ICP(pcds[0],pcds[1])
    t52= colored_ICP(pcds[5],pcds[2])
    t43= colored_ICP(pcds[4],pcds[3])
    t12= colored_ICP(pcds[1],pcds[2])
    t32= colored_ICP(pcds[3],pcds[2])


    H0 = t12 @ t01
    H1 = t12
    H3 = t32
    H4 = t32 @ t43
    H5 = t52

    d_path = args.input
    np.savetxt(d_path+"c_icp_0_2.txt", H0)
    np.savetxt(d_path+"c_icp_1_2.txt", H1)
    np.savetxt(d_path+"c_icp_3_2.txt", H3)
    np.savetxt(d_path+"c_icp_4_2.txt", H4)
    np.savetxt(d_path+"c_icp_5_2.txt", H5)

    p0 = copy.deepcopy(pcds[0]).transform(H0)
    p1 = copy.deepcopy(pcds[1]).transform(H1)
    p3 = copy.deepcopy(pcds[3]).transform(H3)
    p4 = copy.deepcopy(pcds[4]).transform(H4)
    p5 = copy.deepcopy(pcds[5]).transform(H5)

    o3d.visualization.draw_geometries([p0,p1,pcds[2],p3,p4,p5])

def call_cpp_program(directory_path):
    # Adjust this to the path of your compiled C++ executable if it's not in the system's PATH.
    cpp_executable = "/home/vigir3d/Software/programs/k4a-read-mkvs/build/k4a_read_mkv"
    command = cpp_executable + " " + directory_path + "*.mkv"
    print(command)
    # Call the C++ program.
    result = subprocess.run(command, shell=True, check=True)

    # Check if the C++ program executed without errors.
    if result.returncode != 0:
        print(f"Error running {cpp_executable}")
        return False

    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Perform Azure Kinect calibration using the apriltag cube.")
    parser.add_argument("-i", "--input", required=True, help="Path to the calibration data.")
    parser.add_argument("-m", "--mode", required=True, help="Calibration mode: normal (8 cam), body(10 cam, body only), head(10 cam, head only)")

    args = parser.parse_args()
    
    #main(args.input, args.transform)
    #data_path = '/home/vigir3d/Datasets/cattle_scans/farm_07_28/Animal_calib_new4/'
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
    start_time = time.time()
    new_dpath = copy.deepcopy(args.input).rstrip('/')
    success = call_cpp_program(args.input)

    # Rest of your existing code goes here
    # Replace 'dpath' with 'input_path' and 'tform_path' with the passed argument 'tform_path'
    # Use 'colored_icp_path' where necessary, after checking if it's provided
    if not success:
        print("Failed to process MKV files with the C++ program.")
    else:
        print("success!!")
    pcds, tforms = load_filter_pcds(args.input,args.mode) # lower threshold will remove more points()
    o3d.visualization.draw_geometries(pcds)
    end_time = time.time()
    execution_time = end_time - start_time
    print("It took :", execution_time, "s to process all frames")

    print("Created " ,len(pcds), " point clouds")
    perform_pairwise_alignment(pcds,args.mode)
    #o3d.visualization.draw_geometries(pcds)