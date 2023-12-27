import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
import json
import glob
from PIL import Image
import torchvision
import torch
from torchvision import transforms as T 
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor
import time
from numba import jit, prange
import copy

import argparse

# Load MaskRCNN 

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

def colored_ICP(source, target):
    
    voxel_radius = [0.04, 0.02, 0.01]
    max_iter = [50, 30, 14]
    current_transformation = np.identity(4)
    #print("3. Colored point cloud registration")
    for scale in range(3):
        iters = max_iter[scale]
        radius = voxel_radius[scale]
        #print("iteration: ", iters, radius, scale)

        #print("3-1. Downsample with a voxel size %.2f" % radius)
        source_down = source.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)

        #print("3-2. Estimate normal.")
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        #print("3-3. Applying colored point cloud registration")
        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source_down, target_down, radius, current_transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                              relative_rmse=1e-6,
                                                              max_iteration=iters))
        current_transformation = result_icp.transformation
        return current_transformation
    

def get_model_mask(model_generic, image):
    proba_threshold = 0.5
    ig = transform(image)
    with torch.no_grad():
        prediction = model_generic([ig.to(device)])
        
    if(prediction[0]['masks'].nelement() == 0):
        XX = torch.empty((0,0), dtype=torch.int64)
        return XX
    predicted_mask = prediction[0]
    predicted_mask = predicted_mask['masks'][0] > proba_threshold
    
    predicted_mask = predicted_mask.squeeze(1)
    mask = predicted_mask.cpu().detach()
    return mask

def segment_images(last_frame):
    depth_PIL = Image.fromarray(np.asarray(last_frame.depth)).convert("RGB")
    rgb_image_array = np.asarray(last_frame.color)
    depth_image_array = np.asarray(last_frame.depth)
    rgb_PIL = Image.fromarray(rgb_image_array)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    rgb_mask = get_model_mask(model_rgb, rgb_PIL)
    depth_mask = get_model_mask(model_depth, depth_PIL)

    if depth_mask.nelement() == 0:
        mask_combined = rgb_mask
    else:
        mask_combined = depth_mask | rgb_mask  # 1 vote arbitration (OR the masks)

    # Convert tensor to numpy array and ensure the right datatype
    mask_combined = mask_combined.numpy().astype(rgb_image_array.dtype)
    mask_image = mask_combined.swapaxes(0, 2).swapaxes(0, 1)
    mask_image = (mask_image > 0).astype(rgb_image_array.dtype)   

    fg_image_rgb = rgb_image_array * mask_image

    # For the depth image:
    squeezed_mask = np.squeeze(mask_image)
    fg_image_depth = depth_image_array * squeezed_mask
    
    last_frame.color = o3d.geometry.Image(fg_image_rgb)
    last_frame.depth = o3d.geometry.Image(fg_image_depth)

    return last_frame



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

def filter_file_names(file_list): # to avoid the two head cameras
    new_file_list = []
    for f in file_list:
        tmp = f.split('.mkv')[0]
        ID  = tmp.split('_')[-1]
        if (ID == "13" or ID == "15"):
            continue
        else :
            new_file_list.append(f)
    return new_file_list



def load_filter_pcds(data_path,Tforms):  # returns a list of  pcds
    ws = 2
    flying_pixel_filter_threshold = 100

    reader = o3d.io.AzureKinectMKVReader()
    abspath = data_path
    pcds = []    
        
    files_raw = glob.glob(data_path+'/*.mkv')
    files = filter_file_names(files_raw)
    files.sort()
    
    list_size = len(files)
    rgbd_frames = [None] * list_size
  
    for i in range(len(files)): # for each view
   
        inFile = files[i]
        fname = inFile.split('/')[-1]
        file_name = fname.split('.mkv')[0]
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
        #time_stamp = data["stream_length_usec"]
        #print(f"Intrinsic Matrix {intrinsics}")

        camera_intrinsics = o3d.camera.PinholeCameraIntrinsic()
        cx = intrinsics[6]
        cy = intrinsics[7]
        fx = intrinsics[0]
        fy = intrinsics[4]
        camera_intrinsics.set_intrinsics(width,height,fx,fy,cx,cy)
            
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
    
        # filter the depth image for flying pixels
        
        depth_image_array = np.asarray(last_frame.depth)
        result_mask = numba_eliminate_flying_pixels(depth_image_array.copy(), ws, flying_pixel_filter_threshold)
        depth_image_array[result_mask > flying_pixel_filter_threshold] = 0
        # Re-insert filtered depth into the rgbd image
        last_frame.depth = o3d.geometry.Image(depth_image_array)
      
        # Apply maskrcnn to filtered depth image
        masked_rgbd = segment_images(last_frame)
        rgbdc = o3d.geometry.RGBDImage.create_from_color_and_depth(
        masked_rgbd.color, masked_rgbd.depth, depth_trunc=4.0, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbdc,
                                                            camera_intrinsics) 
        
        pcds.append(copy.deepcopy(pcd).transform(Tforms[i]))
        reader.close()
        
    return pcds

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



# Main
rgb_model_path = '/home/vigir3d/Datasets/cattle_scans/maskrcnn_data/maskrcnn_v2.pth'
depth_model_path = '/home/vigir3d/Datasets/cattle_scans/maskrcnn_data/maskrcnn_depth_best.pth'
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Loads the maskrcnn trained for depth and rgb
model_rgb = load_maskrcnn_model(rgb_model_path)
model_depth = load_maskrcnn_model(depth_model_path)

transform = T.ToTensor()
def load_transformations(tform_path):
    """Load transformations from the given path."""
    htm_files = ["htm_0_3.txt", "htm_1_3.txt", "htm_5_3.txt", "htm_6_3.txt", "htm_7_3.txt"]
    transformations = [np.loadtxt(tform_path + file) for file in htm_files]
    transformations.insert(2, np.eye(4))
    return transformations

def apply_transformations(pcds_all):
    """Compute and apply transformations."""
    t01 = colored_ICP(pcds_all[0], pcds_all[1])
    t52 = colored_ICP(pcds_all[5], pcds_all[2])
    t43 = colored_ICP(pcds_all[4], pcds_all[3])
    t12 = colored_ICP(pcds_all[1], pcds_all[2])
    t32 = colored_ICP(pcds_all[3], pcds_all[2])

    H0 = t12 @ t01
    H1 = t12
    H3 = t32
    H4 = t32 @ t43
    H5 = t52

    # Transform the point clouds
    p0 = copy.deepcopy(pcds_all[0]).transform(H0)
    p1 = copy.deepcopy(pcds_all[1]).transform(H1)
    p3 = copy.deepcopy(pcds_all[3]).transform(H3)
    p4 = copy.deepcopy(pcds_all[4]).transform(H4)
    p5 = copy.deepcopy(pcds_all[5]).transform(H5)

    return p0, p1, p3, p4, p5

def main(data_path, tform_path):
    Tforms = load_transformations(tform_path)

    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
    start_time = time.time()
    pcds_all = load_filter_pcds(data_path, Tforms)
    
    p0, p1, p3, p4, p5 = apply_transformations(pcds_all)

    # Combine point clouds
    pcds = o3d.geometry.PointCloud()
    pcds = p0 + p1 + pcds_all[2] + p3 + p4 + p5
    o3d.io.write_point_cloud(data_path+"")

    # Mesh reconstruction
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcds, depth=9)
    sa = mesh.get_surface_area()
    
    end_time = time.time()
    execution_time = end_time - start_time
    print(f"It took {execution_time:.2f}s to reconstruct mesh.")
    print("Surface area is:", sa)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process and reconstruct mesh from data.")
    parser.add_argument("-i", "--input", required=True, help="Path to the data.")
    parser.add_argument("-t", "--transform", required=True, help="Path to the transformations.")
    args = parser.parse_args()
    
    main(args.input, args.transform)

# data_path = '/home/vigir3d/Datasets/cattle_scans/farm_scan1/Animal_482_2/'

# tform_path = '/home/vigir3d/Datasets/cattle_scans/farm_scan1/Animal_482_1/'
# h0 = np.loadtxt(tform_path+"htm_0_3.txt")
# h1 = np.loadtxt(tform_path+"htm_1_3.txt")
# h5 = np.loadtxt(tform_path+"htm_5_3.txt")
# h6 = np.loadtxt(tform_path+"htm_6_3.txt")
# h7 = np.loadtxt(tform_path+"htm_7_3.txt")

# Tforms = [h0,h1,h5,h6,h7]


# o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
# start_time = time.time()
# pcds_all = load_filter_pcds(data_path,Tforms);
# t01= colored_ICP(pcds_all[0],pcds_all[1])
# t52= colored_ICP(pcds_all[5],pcds_all[2])
# t43= colored_ICP(pcds_all[4],pcds_all[3])
# t12= colored_ICP(pcds_all[1],pcds_all[2])
# t32= colored_ICP(pcds_all[3],pcds_all[2])


# H0 = t12 @ t01
# H1 = t12
# H3 = t32
# H4 = t32 @ t43
# H5 = t52


# # np.savetxt("c_icp_0_2.txt", H0)
# # np.savetxt("c_icp_1_2.txt", H1)
# # np.savetxt("c_icp_3_2.txt", H3)
# # np.savetxt("c_icp_4_2.txt", H4)
# # np.savetxt("c_icp_5_2.txt", H5)

# p0 = copy.deepcopy(pcds_all[0]).transform(H0)
# p1 = copy.deepcopy(pcds_all[1]).transform(H1)
# p3 = copy.deepcopy(pcds_all[3]).transform(H3)
# p4 = copy.deepcopy(pcds_all[4]).transform(H4)
# p5 = copy.deepcopy(pcds_all[5]).transform(H5)
# #end_time = time.time()
# #execution_time = end_time - start_time
# #print("It took :", execution_time, "s to process all frames")

# # combine pcds

# #print('run Poisson surface reconstruction')

# pcds = o3d.geometry.PointCloud()
# pcds = p0+p1+pcds_all[2]+p3+p4+p5
# #pcds = pcds_all[0]+pcds_all[1]+pcds_all[2]+pcds_all[3]+pcds_all[4]+pcds_all[5]
# with o3d.utility.VerbosityContextManager(
#         o3d.utility.VerbosityLevel.Error) as cm:
#     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
#         pcds, depth=9)
# sa = mesh.get_surface_area()
# end_time = time.time()
# execution_time = end_time - start_time
# print("It took :", execution_time, "s to reconstruct mesh")
# print(sa)
