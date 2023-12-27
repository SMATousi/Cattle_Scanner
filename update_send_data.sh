#!/bin/bash
for i in {11..20}
do
   rsync /home/vigir3d/send_data.sh vigir@192.168.0.$i:/home/vigir/Desktop/send_data.sh
done

