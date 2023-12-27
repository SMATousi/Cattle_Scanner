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

def detect_tag(img,k,tag_size):
    at_detector = Detector(families='tagStandard41h12',
                       nthreads=8,
                       quad_decimate=1.0,
                       quad_sigma=0.8,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
    tags  = at_detector.detect(img,True,k,tag_size)
    #print("detected : ", len(tags), " tags")
    best_tag = min(tags, key=lambda tag: tag.pose_err)
    # create a 4x4 identity matrix
    H = np.eye(4)

    # set the rotation
    H[:3, :3] = best_tag.pose_R

    # set the translation
    H[:3, 3] = best_tag.pose_t[:, 0]
    #print("Done detecting tags")

    return H, best_tag.tag_id

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
    T = np.identity(4)d
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
    
   
    draw_registration_result(source, target, current_transformation)
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

def filter_file_names(file_list, mode="normal"):
    # Adjusted the function to handle different modes: "normal", "body", and "head"
    new_file_list = []
    
    for f in file_list:
        tmp = f.split('.mkv')[0]
        ID = tmp.split('_')[-1]

        if mode.lower() == "normal":
            # Filter out 13 and 15
            if ID in ["13", "15"]:
                continue
        elif mode.lower() == "body":
            # Filter out 13, 14, 16, 17
            if ID in ["13", "14", "16", "17"]:
                continue
        elif mode.lower() == "head":
            # Keep only 13, 14, 16, 17
            if ID not in ["13", "14", "16", "17"]:
                continue

        new_file_list.append(f)

    return new_file_list



# v2 returns two point clouds, 1 segmented, the other is unmodified
def process_file_list_comp_v2(file_id, dpath,tform):
    ws = 2
    # Format filenames
    dpath = dpath.rstrip('/')

    # Split by the directory separator and get the last part
    suffix = os.path.basename(dpath)
    file_suffix = suffix + "_nano_" + str(file_id)
      
    # Construct full file paths using os.path.join for better cross-platform compatibility
    depth_file = os.path.join(dpath, file_suffix + "_depth.png")
    color_file = os.path.join(dpath, file_suffix + ".jpg")
    intrinsics_file = os.path.join(dpath, file_suffix + "_intrinsics.txt")
    # Read the files
    depth = o3d.io.read_image(depth_file)
    color = o3d.io.read_image(color_file)
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
    pcd_tsdf.transform(tform)
    depth_np = np.asarray(rgbdc.depth)
    
    if ( depth_np.max() > 20 ) : # Depth is in millimeters
        flying_pixel_filter_threshold = 100 # 100
    else : # meters
        flying_pixel_filter_threshold = 0.1 # 0.1
        #print(flying_pixel_filter_threshold, " -- ", depth_np.max())

    APPLY_FLYING_PIXEL_FILTER = False
    if APPLY_FLYING_PIXEL_FILTER : 
        result_mask = numba_eliminate_flying_pixels(depth_np.copy(), ws, flying_pixel_filter_threshold)
        depth_np[result_mask > flying_pixel_filter_threshold] = 0
    # Re-insert filtered depth into the rgbd image
    rgbdc.depth = o3d.geometry.Image(depth_np)
    # Apply maskrcnn to filtered depth image
    masked_rgbd = segment_images_modified(rgbdc,dpath+"/", file_suffix)
    # necessary lol
    rgbdc_new = o3d.geometry.RGBDImage.create_from_color_and_depth(
                    masked_rgbd.color, masked_rgbd.depth, depth_trunc=4.0, convert_rgb_to_intensity=False)
    d = np.asarray(rgbdc_new.depth)

    v = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=4.0 / 512.0, sdf_trunc=0.04, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
    )

    v.integrate(rgbdc_new, camera_intrinsics, np.eye(4))
    pcd_tmp = v.extract_point_cloud()
    pcd_tmp.transform(tform)
    #pcd_tmp = o3d.geometry.PointCloud.create_from_rgbd_image(rgbdc_new,
     #                                                   camera_intrinsics)

    return pcd_tmp, pcd_tsdf

def load_filter_pcds(data_path,mode):  # returns a list of  pcds


    l = 0.3048  # replace with the actual value
    ws = 2
   
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

    reader = o3d.io.AzureKinectMKVReader()
    abspath = data_path
   
    files_raw = glob.glob(data_path+'/*.mkv')
    files = filter_file_names(files_raw,mode)
    files.sort()   

    #print("There are : ", len(files), "present here!")

    list_size = len(files)
    rgbd_frames = [None] * list_size

    tag_poses = [None] * list_size  # assuming tag_poses and current_transform are dictionaries
    current_transform = [None] * list_size
    pcds = []
    for i in range(len(files)): # for each view
        inFile = files[i]
        fname = inFile.split('/')[-1]
        file_name = fname.split('.mkv')[0]
        #print("Current File: ", file_name)

        reader.open(inFile)
        if not reader.is_opened():
            raise RuntimeError("Unable to open file {}".format(inFile))
        metadata = reader.get_metadata()
  
        # write the metadata to a JSON file since that seems to be the only
        # way to retrieve that data
        o3d.io.write_azure_kinect_mkv_metadata(
                    '{}/{}_intrinsic.json'.format(abspath,file_name), metadata)

        # Open the file and load the JSON
        with open(abspath+"/" + file_name + "_intrinsic.json") as f:
            data = json.load(f)
        
        height = data['height']
        width = data['width']
        intrinsics = data["intrinsic_matrix"]
        camera_intrinsics = o3d.camera.PinholeCameraIntrinsic()
        cx = intrinsics[6]
        cy = intrinsics[7]
        fx = intrinsics[0]
        fy = intrinsics[4]
        camera_intrinsics.set_intrinsics(width,height,fx,fy,cx,cy)
        K = (fx,fy,cx,cy)

        last_frame = None
        while not reader.is_eof(): # go until hitting eof because of exposure issues in early color frames
            rgbda = reader.next_frame()
            if rgbda is None:
                #print("Got nothing! ")
                continue
            last_frame = rgbda

        if last_frame is not None:
            #print("Got the last frame")
            rgbd_frames[i] = last_frame
        else:
            
            print("************No valid frames found in the .mkv file.**********")

        rgb_im_np = np.asarray(last_frame.color)
    
        gray_img = cv2.cvtColor(rgb_im_np, cv2.COLOR_RGB2GRAY)
        im_name = "img_" + str(i) + ".jpg" 
        cv2.imwrite(im_name, gray_img)
        transform, tag_id = detect_tag(gray_img,K, tag_size)
        #print("T: ", transform)
        #print("ID: ", tag_id)
        
        # get the transformation matrix for the given tag_id
        cube_transform = transformations.get(tag_id, np.eye(4))  # defaults to identity matrix if tag_id is not found

        tag_poses[i] = np.dot(transform, np.linalg.inv(cube_transform))
        
        # assuming tag_poses[0] is defined and is the pose of the first camera
        tag_pose = np.dot(tag_poses[0], np.linalg.inv(tag_poses[i]))

        current_transform[i] = tag_pose  # current_transform is now the tag pose
        reader.close()
        # WRITE THE TRANSFORMATIONS
        fname_tform = "H_0_" + str(i) + ".txt"
        np.savetxt(data_path+fname_tform, current_transform[i])
        depth_image_array = np.asarray(last_frame.depth) # reference the same depth so it will change
         
        #start_time_numba = time.time()
        
        #result_mask = numba_eliminate_flying_pixels(depth_image_array.copy(), ws, flying_pixel_filter_threshold)
        #end_time_numba = time.time()

        # apply thresholding
        #count = np.sum(result_mask > flying_pixel_filter_threshold)
        #print("Numba Detected :#", count)

        #depth_image_array[result_mask > flying_pixel_filter_threshold] = 0
        filtered_depth = depth_image_array
        #execution_time_numba = end_time_numba - start_time_numba
        #filtered_depth = bilateral_filter(depth_image_array)
        last_frame.depth = o3d.geometry.Image(filtered_depth)
        pcd = backproject_o3d(last_frame, camera_intrinsics)
        #o3d.visualization.draw_geometries([pcd])
        #break
        pcds.append(copy.deepcopy(pcd).transform(current_transform[i]))

    o3d.visualization.draw_geometries(pcds)
    if mode.lower=="body":
        # Swap the point clouds at index 3 and 4
        pcds[3], pcds[4] = pcds[4], pcds[3]
        current_transform[3], current_transform[4] = current_transform[4], current_transform[3]

    return pcds,current_transform


def compute_and_orient_normals(pcd, voxel_size):
    normal_radius = voxel_size*2.0
    pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(normal_radius=0.1, max_nn=30))


def perform_pairwise_alignment(pcds,mode):

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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Perform Azure Kinect calibration using the apriltag cube.")
    parser.add_argument("-i", "--input", required=True, help="Path to the calibration data.")
    parser.add_argument("-m", "--mode", required=True, help="Calibration mode: normal (8 cam), body(10 cam, body only), head(10 cam, head only)")

    args = parser.parse_args()
    
    #main(args.input, args.transform)
    #data_path = '/home/vigir3d/Datasets/cattle_scans/farm_07_28/Animal_calib_new4/'
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
    start_time = time.time()
    pcds, tforms = load_filter_pcds(args.input,args.mode) # lower threshold will remove more points()
    end_time = time.time()
    execution_time = end_time - start_time
    print("It took :", execution_time, "s to process all frames")

    print("Created " ,len(pcds), " point clouds")
    perform_pairwise_alignment(pcds,args.mode)
    #o3d.visualization.draw_geometries(pcds)

