#!/bin/bash

echo "Data collection path: $1";


for d in $1/* ; do
    /home/vigir3d/Software/programs/k4a-kinfu-gen-plys/build/kinfu_create_plys ${d}/*.mkv
    echo "Creating point cloud data for $d"
done