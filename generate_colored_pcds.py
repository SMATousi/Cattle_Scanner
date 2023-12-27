# OFFICIAL PIPELINE CODE FOR THE CATTLE SCANNER
# AUTHOR : GBENGA OMOTARA
# LAST MODIFIED : Nov 30 2023


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
from PIL import Image, ImageEnhance
import torchvision
import torch
from torchvision import transforms as T 
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor
from numba import jit, prange
import os
import csv 
import argparse

# Load MaskRCNN 

import subprocess


def save_results(animal_id, data_path, pcd_full):
    # Save the mesh and PCD
    pcd_full_filename = os.path.join(data_path, f"{animal_id}_head.ply")
    o3d.io.write_point_cloud(pcd_full_filename, pcd_full)

        
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


def pcd2xyz(pcd):
    return np.asarray(pcd.points).T

def pcd2normals(pcd):
    return np.asarray(pcd.normals).T

def segment_images_modified(last_frame,spath,fname):
    depth_image_array = np.asarray(last_frame.depth)
    #print("Pre:",np.max(depth_image_array), " -- ", np.min(depth_image_array))
    depth_PIL = Image.fromarray(np.asarray(last_frame.depth)).convert("RGB")
    rgb_image_array = np.asarray(last_frame.color)
    rgb_PIL = Image.fromarray(rgb_image_array)

    #device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    rgb_mask = get_model_mask(model_rgb, rgb_PIL)
    depth_mask = get_model_mask(model_depth, depth_PIL)

    #rgb_mask_pil = Image.fromarray(rgb_mask.numpy() * 255)  # Scale mask to 0-255 for visualization
    #depth_mask_pil = Image.fromarray(depth_mask.numpy() * 255)  # Scale mask to 0-255 for visualization
    #rgb_mask_pil.save(spath+ fname + "_rgb.png")
    #depth_mask_pil.save(spath+ fname +"_depth.png")

    if depth_mask.nelement() == 0:
        mask_combined = rgb_mask
    else:
        mask_combined = depth_mask | rgb_mask  # 1 vote arbitration (OR the masks)

    # Convert tensor to numpy array and ensure the right datatype
    mask_combined = mask_combined.numpy().astype(rgb_image_array.dtype)
    mask_image = mask_combined.swapaxes(0, 2).swapaxes(0, 1)
    mask_image = (mask_image > 0).astype(rgb_image_array.dtype)   
    #mask_image_pil = Image.fromarray(mask_image * 255)  # Scale mask to 0-255 for visualization
    #mask_image_pil.save(spath+ fname + "_mask.png")

    fg_image_rgb = rgb_image_array * mask_image
    fg_image_rgb_pil = Image.fromarray(fg_image_rgb)
    #Optionally, increase contrast
    enhancer = ImageEnhance.Contrast(fg_image_rgb_pil)
    fg_image_rgb_pil = enhancer.enhance(5.0)  # Increase contrast, adjust 2.0 as needed

    fg_image_rgb_pil.save(spath+ fname + "_mask.png")
    # For the depth image:
    squeezed_mask = np.squeeze(mask_image)
    fg_image_depth = (depth_image_array * squeezed_mask)*1000 # upscale because re-inserting in "last_frame" rescales
    
    #print("Post:" ,np.max(fg_image_depth), " -- ", np.min(fg_image_depth))
    last_frame.color = o3d.geometry.Image(fg_image_rgb)
    last_frame.depth = o3d.geometry.Image(fg_image_depth)

    return last_frame

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


# v1 returns only the segmented point cloud
def process_file_list_comp_v1(file_id, dpath):
    ws = 2
    flying_pixel_filter_threshold = 1000
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
   
    # using H_0_camID.txt
    # subsequently, apply coloredICP results since they are computed wrt to that initial trnasform.
    pcd_fname = os.path.join(dpath,file_suffix+"_head.ply")
    print(pcd_fname)
    o3d.io.write_point_cloud(pcd_fname,pcd_tsdf) # write each point cloud to disk

def segment_floor_plane(pcd):
    
    #o3d.visualization.draw_geometries([pcd])
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.05,
                                             ransac_n=3,
                                             num_iterations=1000)


    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    #o3d.visualization.draw_geometries([inlier_cloud], window_name="Detected plane")
    o3d.visualization.draw_geometries([outlier_cloud],window_name="pcd without plane")
    return outlier_cloud



def combine_pcds(pcds):
    # takes a list and returns a combined point cloud
    pcd_comb = o3d.geometry.PointCloud()
    for pcd in pcds:
        pcd_comb+=pcd
    return pcd_comb



def main(input_path):

    success = call_cpp_program(input_path)

    # Rest of your existing code goes here
    # Replace 'dpath' with 'input_path' and 'tform_path' with the passed argument 'tform_path'
    # Use 'colored_icp_path' where necessary, after checking if it's provided
    if not success:
        print("Failed to process MKV files with the C++ program.")
    else:
        print("success!!")
        #tforms = load_tforms_teaser()
        mkv_files = [file for file in os.listdir(input_path) if file.endswith('.mkv')]
        file_ids = [file.split('_')[-2] for file in mkv_files]  # Extracting ID numbers
        print(file) for file in file_ids_
        # get the pcds for each view
        pcds = [process_file_list_comp_v1(file_id, input_path) for file_id in file_ids]
        
        
        
      

if __name__ == "__main__":
    start = time.time()
    parser = argparse.ArgumentParser(description="Process the input, transformation matrices, and colored ICP registration transformation path.")

    parser.add_argument("-i", "--input_path", required=True, help="Path to the input files.")
    
    args = parser.parse_args()

    main(args.input_path)
    end = time.time()
    print("Duration was:", end - start, "s")
