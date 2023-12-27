#!/bin/bash

#add to ~/.config/autostart/MyScript.desktop -- in EXEC

while true;
do
	# watch the ~/Images directory on the nano
	# the exact rsync command will be posted in a .txt file
	# we read the file after each capture and then we begin to rsync	
	watch -g ls -laR --full-time /home/vigir/Images
	value=`cat /home/vigir/Desktop/rsync_command.txt`
	eval "$value"
	echo "$value"
done

