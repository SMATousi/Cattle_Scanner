#!/bin/bash
for i in {11..20}
do
   rsync -av /home/vigir3d/Software/programs/Cattle_Scanner/launch_server.sh vigir@10.14.10.$i:/home/vigir/Desktop/launch_server.sh
done

