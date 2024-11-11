import os
import shutil

def organize_and_copy_images(src_root_dir, dst_dir, valid_prefixes, exclude_string):
    # Make sure the destination directory exists
    if not os.path.exists(dst_dir):
        os.makedirs(dst_dir)

    # Walk through each parent directory
    for parent_dir in os.listdir(src_root_dir):
        parent_dir_path = os.path.join(src_root_dir, parent_dir)

        # Check if the parent directory name starts with any of the valid prefixes
        if os.path.isdir(parent_dir_path) and any(parent_dir.startswith(prefix) for prefix in valid_prefixes):
            # Extract session name (assuming the session name is the parent directory name)
            session_name = parent_dir

            # Walk through subdirectories inside the parent directory
            for sub_dir in os.listdir(parent_dir_path):
                sub_dir_path = os.path.join(parent_dir_path, sub_dir)

                # Skip subdirectories containing the exclude string (e.g., "cal")
                if exclude_string in sub_dir:
                    print(f"Skipping subdirectory: {sub_dir_path}")
                    continue

                if os.path.isdir(sub_dir_path):
                    # Process all the .png files in the subdirectory
                    for file_name in os.listdir(sub_dir_path):
                        # Skip .png files that contain "mask" in the filename
                        if file_name.endswith(".png") and "mask" not in file_name:
                            # Construct source file path
                            src_file_path = os.path.join(sub_dir_path, file_name)
                            
                            # Modify the file name to include the session name
                            new_file_name = f"{os.path.splitext(file_name)[0]}_{session_name}{os.path.splitext(file_name)[1]}"

                            # Construct destination file path
                            dst_file_path = os.path.join(dst_dir, new_file_name)

                            # Copy the file to the destination directory with the new name
                            shutil.copy2(src_file_path, dst_file_path)
                            print(f"Copied and renamed: {src_file_path} -> {dst_file_path}")

# Set parameters
src_root_dir = "/media/vigir3d/Cow_Backup_1/cattle_scans/"  # Root directory containing session directories
dst_dir = "/home/vigir3d/Datasets/grounded_sam_data_depth"  # Destination directory for copied images
valid_prefixes = ["metab", "thompson", "beef_farm", "trowbridge"]  # List of expected prefixes
exclude_string = "cal"  # Subdirectories containing this string will be skipped

# Run the function
organize_and_copy_images(src_root_dir, dst_dir, valid_prefixes, exclude_string)
