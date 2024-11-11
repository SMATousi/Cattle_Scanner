import cv2
import numpy as np
import open3d as o3d
import json
import glob
import matplotlib.pyplot as plt
import copy
import time
from PIL import Image, ImageEnhance
import os
import csv 
import argparse
import subprocess
import pandas as pd

# Example CLI
# python count_points.py -i ~/Datasets/wireless_tests/Animal_basecase1/ ~/Datasets/cattle_scans/wired_sync_capture/Animal_case3/ ~/Datasets/cattle_scans/test_wireless/Animal_case10/ ~/Datasets/cattle_scans/wired_synchronization_0/Animal_case5/



def call_cpp_program(directory_path):
    cpp_executable = "/home/vigir3d/Software/programs/k4a-read-mkvs/build/k4a_read_mkv"
    command = cpp_executable + " " + directory_path + "*.mkv"
    print(command)
    result = subprocess.run(command, shell=True, check=True)
    if result.returncode != 0:
        print(f"Error running {cpp_executable}")
        return False
    return True


import os
import open3d as o3d
import numpy as np

def process_file_list_comp_v1(file_id, dpath):
    dpath = dpath.rstrip('/')
    suffix = os.path.basename(dpath)
    
    # Generate both file name patterns
    file_suffixes = [f"{suffix}_nano_{file_id}", f"{suffix}_nano{file_id}"]
    
    files_found = False
    
    for file_suffix in file_suffixes:
        depth_file = os.path.join(dpath, file_suffix + "_depth.png")
        color_file = os.path.join(dpath, file_suffix + ".jpg")
        intrinsics_file = os.path.join(dpath, file_suffix + "_intrinsics.txt")
        
        #print(f"Checking files for suffix {file_suffix}:")
        #print(f"Depth file: {depth_file}")
        #print(f"Color file: {color_file}")
        #print(f"Intrinsics file: {intrinsics_file}")
        
        if os.path.exists(depth_file) and os.path.exists(color_file) and os.path.exists(intrinsics_file):
            files_found = True
            break
        else:
            print("Files not found, continuing...")
    
    if not files_found:
        raise FileNotFoundError(f"Files not found for {file_id} in {dpath} with either naming pattern.")
    
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
    return pcd_tsdf

def write_individual_pcds(file_id, dpath, pcd):
    dpath = dpath.rstrip('/')
    suffix = os.path.basename(dpath)
    file_suffix = suffix + "_nano_" + str(file_id)
    o3d.io.write_point_cloud(file_suffix + ".ply", pcd)
    print("Written: ", file_suffix + ".ply")
    return 1

def process_folder(folder_path, file_ids):
    point_counts = []
    success = call_cpp_program(folder_path)
    if not success:
        print(f"Failed to process MKV files in {folder_path}")
    else:
        for file_id in file_ids:
            pcd = process_file_list_comp_v1(file_id, folder_path)
            point_count = len(np.asarray(pcd.points))
            point_counts.append(point_count)
    return point_counts

def main(folder_paths, file_name):
    file_ids = range(11, 21)
    column_titles = ["base case", "w/ wire sync", "w/ esp sync", "w/o sync"]
    all_counts = {}
    
    for folder, column_title in zip(folder_paths, column_titles):
        all_counts[column_title] = process_folder(folder, file_ids)
    
    point_count_table = pd.DataFrame(all_counts, index=[f'MKV {i}' for i in file_ids])
    print(point_count_table)
    point_count_table.to_csv('point_counts.csv')

    # Calculate percentage table
    base_case = point_count_table["base case"]
    percentage_table = point_count_table.div(base_case, axis=0)
    print(percentage_table)
    percentage_table.to_csv(file_name+'_point_counts.csv')

if __name__ == "__main__":
    start = time.time()
    
    parser = argparse.ArgumentParser(description="Process MKV files and count points in the resulting point clouds.")
    parser.add_argument("-n", "--file_name", required=True, help="Output File Name")
    parser.add_argument("-i", "--input_paths", required=True, nargs='+', help="Paths to the input folders.")
    args = parser.parse_args()
    
    main(args.input_paths, args.file_name)
    end = time.time()
    print("Duration was:", end - start, "s")