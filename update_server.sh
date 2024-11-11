#!/bin/bash

for ip in $(seq 11 20)
do
   rsync /home/vigir3d/Software/programs/Cattle_Scanner/zmq_server.py vigir@10.14.10.$ip:/home/vigir/Desktop/zmq_server.py
done


