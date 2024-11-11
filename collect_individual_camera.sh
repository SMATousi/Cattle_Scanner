#!/bin/bash

# List of device IP addresses
devices=(
  "10.14.10.11"
  "10.14.10.12"
  "10.14.10.13"
  "10.14.10.14"
  "10.14.10.15"
  "10.14.10.16"
  "10.14.10.17"
  "10.14.10.18"
  "10.14.10.19"
  "10.14.10.20"
)

# Local destination directory
local_dir="/home/vigir3d/Datasets/wired_sync_base_case_7_30_24"

# List of session names
sessions=("Animal_cap1" "Animal_cap2" "Animal_cap3" "Animal_cap4" "Animal_cap5" "Animal_cap6" "Animal_cap7" "Animal_cap8" "Animal_cap9" "Animal_cap10")

# Loop through each session
for session in "${sessions[@]}"; do
  # Create the session directory if it doesn't exist
  mkdir -p "$local_dir/$session"
  
  # Loop through each device
  for device in "${devices[@]}"; do
    # Construct the MKV file name
    server_no=$(echo $device | awk -F '.' '{print $4}')
    mkv_file="${session}_nano${server_no}.mkv"
    case_path="$local_dir/$session"
    
    echo "Processing device $device with file $mkv_file for session $session..."

    # Capture command
    sync_command="export DISPLAY=:0 && k4arecorder -e 2500 -r 30 -l 1 -d NFOV_UNBINNED -c 1080p $mkv_file"

    # Run the capture command on the device
    ssh vigir@$device "$sync_command"

    # Wait for 5 seconds to ensure the capture is complete
    sleep 5

    # Rsync the captured data to the local folder
    rsync -av vigir@$device:~/$mkv_file "$local_dir/$session"
    
    sleep 5

    echo "Completed processing device $device with file $mkv_file for session $session"
  done
done

echo "All sessions have been processed."
