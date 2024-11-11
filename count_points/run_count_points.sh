#!/bin/bash

# Define the base paths
BASE_PATH="~/Datasets/cattle_scans"
WIRED_SYNC_BASE_CASE="$BASE_PATH/wired_sync_base_case_7_30_24"
WIRED_SYNC="$BASE_PATH/wired_sync_7_30_24"
WIRELESS_ESP="$BASE_PATH/wireless_esp_7_30_24"
WIRED_ZERO_SYNC="$BASE_PATH/wired_zero_sync_7_30_24"

# Loop through cap1 to cap10
for i in {1..10}
do
    CAP="cap$i"
    echo "Processing $CAP..."
    
    python count_points.py -n $CAP -i \
    "$WIRED_SYNC_BASE_CASE/Animal_$CAP/" \
    "$WIRED_SYNC/Animal_$CAP/" \
    "$WIRELESS_ESP/Animal_$CAP/" \
    "$WIRED_ZERO_SYNC/Animal_$CAP/"
    
    echo "$CAP done."
done
