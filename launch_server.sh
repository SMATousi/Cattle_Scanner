#!/bin/bash


sh ~/Desktop/send_data.sh &
sh ~/Desktop/rsync_worker.sh &

python3 ~/Desktop/zmq_server.py &
