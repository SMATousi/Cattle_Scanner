#!/bin/bash

for i in {11..20}
do
   ping -c 1 10.14.10.$i > /dev/null
   if [ $? -eq 0 ]; then
      echo "Device 10.14.10.$i is up"
   else
      echo "Device 10.14.10.$i is down"
   fi
done
