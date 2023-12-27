#!/bin/bash

echo "Data collection path: $1";


for d in $1/* ; do
    cd ${d}
    cp *.jpg /home/vigir3d/Datasets/cattle_scans/maskrcnn_data/images/ 
    cd -
    echo "Copying jpeg files for $d"
done
