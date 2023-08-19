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


def load_filter_pcds(data_path):  # returns a list of  pcds


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

    reader = o3d.io.AzureKinectMKVReader()
    abspath = data_path
    
        
    files = glob.glob(data_path+'/*.mkv')
    files.sort()
    print("There are : ", len(files), "present here!")

    list_size = len(files)
    rgbd_frames = [None] * list_size

    tag_poses = [None] * list_size  # assuming tag_poses and current_transform are dictionaries
    current_transform = [None] * list_size
    pcds = []
    for i in range(len(files)): # for each view
        inFile = files[i]
        fname = inFile.split('/')[-1]
        file_name = fname.split('.mkv')[0]
        print("Current File: ", file_name)

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
                print("Got nothing! ")
                continue
            last_frame = rgbda

        if last_frame is not None:
            print("Got the last frame")
            rgbd_frames[i] = last_frame
        else:
            print("************No valid frames found in the .mkv file.**********")

        rgb_im_np = np.asarray(last_frame.color)
    
        gray_img = cv2.cvtColor(rgb_im_np, cv2.COLOR_RGB2GRAY)
        im_name = "img_" + str(i) + ".jpg" 
        cv2.imwrite(im_name, gray_img)
        transform, tag_id = detect_tag(gray_img,K, tag_size)
        print("T: ", transform)
        print("ID: ", tag_id)
        
        # get the transformation matrix for the given tag_id
        cube_transform = transformations.get(tag_id, np.eye(4))  # defaults to identity matrix if tag_id is not found

        tag_poses[i] = np.dot(transform, np.linalg.inv(cube_transform))
        
        # assuming tag_poses[0] is defined and is the pose of the first camera
        tag_pose = np.dot(tag_poses[0], np.linalg.inv(tag_poses[i]))

        current_transform[i] = tag_pose  # current_transform is now the tag pose
        reader.close()
        fname_tform = "H_0_" + str(i) + ".txt"
        np.savetxt(fname_tform, current_transform[i])
        pcd = backproject_o3d(last_frame, camera_intrinsics)
        pcds.append(copy.deepcopy(pcd).transform(current_transform[i]))

    o3d.visualization.draw_geometries(pcds)
    return pcds,current_transform
    

def detect_tag(img,k,tag_size):
    at_detector = Detector(families='tagStandard41h12',
                       nthreads=8,
                       quad_decimate=1.0,
                       quad_sigma=0.8,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
    tags  = at_detector.detect(img,True,k,tag_size)
    print("detected : ", len(tags), " tags")
    best_tag = min(tags, key=lambda tag: tag.pose_err)
    # create a 4x4 identity matrix
    H = np.eye(4)

    # set the rotation
    H[:3, :3] = best_tag.pose_R

    # set the translation
    H[:3, 3] = best_tag.pose_t[:, 0]
    print("Done detecting tags")

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
    T = np.identity(4)
    T[:3,:3] = R
    T[:3,3] = t
    return T 


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
    
   
        #draw_registration_result(source, target, current_transformation)
    return current_transformation

def backproject_o3d(rgbd_frame, intrinsics):
    
    rgbdc = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgbd_frame.color, rgbd_frame.depth, depth_trunc=4.0, convert_rgb_to_intensity=False)


    # Create a point cloud from the RGBD image
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbdc, intrinsics)
    #n_radius = 0.01*2.0
    #pcd.estimate_normals(
    #search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=n_radius, max_nn=30))
    # Visualize the point cloud
    #o3d.visualization.draw_geometries([pcd])
    
    return pcd
data_path = '/home/vigir3d/Datasets/cattle_scans/farm_07_24/Animal_calib'
pcds, tforms = load_filter_pcds(data_path)


print("Created " ,len(pcds), " point clouds")

t01= colored_ICP(pcds[0],pcds[1])
t52= colored_ICP(pcds[5],pcds[2])
t43= colored_ICP(pcds[4],pcds[3])
t12= colored_ICP(pcds[1],pcds[2])
t32= colored_ICP(pcds[3],pcds[2])



r01, h01 = register_two_views_teaser(pcds[0],pcds[1], 0.05)
r52, h52 = register_two_views_teaser(pcds[5],pcds[2], 0.05)
r43, h43 = register_two_views_teaser(pcds[4],pcds[3], 0.05)
r12, h12 = register_two_views_teaser(pcds[1],pcds[2], 0.05)
r32, h32 = register_two_views_teaser(pcds[3],pcds[2], 0.05)

H0 = t12 @ t01
H1 = t12
H3 = t32
H4 = t32 @ t43
H5 = t52

p0 = copy.deepcopy(pcds[0]).transform(H0)
p1 = copy.deepcopy(pcds[1]).transform(H1)
p3 = copy.deepcopy(pcds[3]).transform(H3)
p4 = copy.deepcopy(pcds[4]).transform(H4)
p5 = copy.deepcopy(pcds[5]).transform(H5)

o3d.visualization.draw_geometries([p0,p1,pcds[2],p3,p4,p5])

current_transform = [H0,H1,np.eye(4),H3,H4,H5]

pose_graph = o3d.pipelines.registration.PoseGraph()
for i in range(6):  # for each camera
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(current_transform[i]))



pairs = [(0,1), (1,2), (4,3), (5,2), (3,2), (1,5)]
for i, j in pairs:
    transformation_matrix = np.dot(current_transform[i], np.linalg.inv(current_transform[j]))
    information_matrix = np.identity(6)  # Assuming all measurements are equally reliable
    pose_graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(i, j, transformation_matrix, information_matrix, uncertain = False))



option = o3d.pipelines.registration.GlobalOptimizationOption(
    max_correspondence_distance = 0.0001,
    edge_prune_threshold = 0.5,
    reference_node = 2)
o3d.pipelines.registration.global_optimization(
    pose_graph,
    o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
    o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
    option)

# Assuming pcds is a list of your point clouds
for i in range(len(pcds)):
    # Get the optimized transformation for the point cloud
    optimized_transformation = np.asarray(pose_graph.nodes[i].pose)
    # Apply the transformation to the point cloud
    pcds[i].transform(optimized_transformation)

# Draw all point clouds in one window
o3d.visualization.draw_geometries(pcds)