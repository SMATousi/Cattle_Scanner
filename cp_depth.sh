#!/bin/bash

echo "Data collection path: $1";


for d in $1/* ; do
    cd ${d}
    cp *_depth.png /home/vigir3d/Datasets/cattle_scans/maskrcnn_data/depth_images/ 
    cd -
    echo "Copying depth files for $d"
done
