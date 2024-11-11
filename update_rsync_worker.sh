#!/bin/bash
for i in {11..20}
do
   rsync /home/vigir3d/Software/programs/Cattle_Scanner/rsync_worker.sh vigir@10.14.10.$i:/home/vigir/Desktop/rsync_worker.sh
done

