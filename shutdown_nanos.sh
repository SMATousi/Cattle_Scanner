#!/bin/bash

for ip in $(seq 11 20)
do
    ssh root@192.168.0.$ip 'shutdown -h now'
done

