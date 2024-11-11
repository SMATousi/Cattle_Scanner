#!/bin/bash
pkill -f zmq_server.py

sleep 1

python3 ~/Desktop/zmq_server.py &
