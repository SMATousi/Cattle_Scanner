#!/bin/bash

for ip in $(seq 11 20)
do
    ssh root@10.14.10.$ip 'shutdown -r now'
done

