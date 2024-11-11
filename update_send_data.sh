#!/bin/bash
for i in {11..20}
do
   rsync /home/vigir3d/Software/programs/Cattle_Scanner/send_data.sh vigir@10.14.10.$i:/home/vigir/Desktop/send_data.sh
done

