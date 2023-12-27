#!/bin/bash

for ip in $(seq 11 20)
do
   rsync /home/vigir3d/Software/programs/Cattle_Scanner/zmq_server.py vigir@192.168.0.$ip:/home/vigir/Desktop/zmq_server.py
done


