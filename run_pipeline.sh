#!/bin/bash

# Check if the path argument is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <path_to_directories>"
    exit 1
fi

# The path to the directories
path_to_directories=$1
calib="/home/vigir3d/Datasets/cattle_scans/FORAGE_FARMS/tmp_folder/Animal_calib5/"
write_path="/home/vigir3d/Datasets/cattle_scans/forage_farms_processed/"
# Check if the provided path is a directory
if [ ! -d "$path_to_directories" ]; then
    echo "The provided path is not a directory: $path_to_directories"
    exit 1
fi

# Iterate over each directory in the provided path
for dir in "$path_to_directories"/*/ ; do
    # Get the name of the directory
    dir_name=$(basename "$dir")

    # Skip the directory if it is "tmp_folder"
    if [ "$dir_name" == "tmp_folder" ]; then
        echo "Skipping directory: $dir"
        continue
    fi

    # Assuming 'your_script.py' is the name of your Python script
    # Add any necessary Python script arguments after the script name
    echo "Running script on directory: $dir and calibration: $calib"
    python /home/vigir3d/Software/programs/Cattle_Scanner/pipeline_code.py -i "$dir" -t "$calib" -c "$calib" -w "$write_path"

    # Check the exit status of the Python script
    if [ $? -ne 0 ]; then
        echo "Python script failed on directory: $dir"
        # Uncomment the next line if you want the script to stop on failure
        # exit 1
    fi
done

echo "Script execution completed for all directories."
