#!/bin/bash

# Directory to monitor for changes
watch_dir="/home/vigir/Images"

# File containing the rsync command
rsync_command_file="/home/vigir/Desktop/rsync_command.txt"

# Queue file to store rsync commands
rsync_queue="/home/vigir/Desktop/rsync_queue.txt"

# Log file
log_file="/home/vigir/Desktop/capture_log.txt"

touch "$rsync_queue"
# Function to add unique rsync commands to the queue
add_to_queue() {
    #if [ -s "$rsync_command_file" ]; then
	command1=`cat /home/vigir/Desktop/rsync_command.txt`
	  
    	#command= `cat /home/vigir/Desktop/rsync_command.txt`
	echo "$(date): Read command: $command1" >> "$log_file"
	if ! grep -Fxq "$command1" "$rsync_queue"; then
		echo "$command1" >> "$rsync_queue"
		echo "$(date): Added command to queue: $command1" >> "$log_file"
	else
		echo "$(date): Command already in queue: $command1" >> "$log_file"
	fi
    #else
    #   echo "$(date): Rsync command file is empty or does not exist" >> "$log_file"
    #fi
}

while true; do
    # Monitor the directory for changes
    watch -g ls -laR --full-time /home/vigir/Images/

    # Add the rsync command to the queue
    add_to_queue

    # Small delay before the next iteration
    sleep 2
done
