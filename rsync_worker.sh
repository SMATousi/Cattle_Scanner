#!/bin/bash

# Queue file that stores all rsync commands
rsync_queue="/home/vigir/Desktop/rsync_queue.txt"

# Temp file for processing
rsync_temp="/home/vigir/Desktop/rsync_temp.txt"

# Log file
log_file="/home/vigir/Desktop/rsync_worker_log.txt"

# Create the queue file if it doesn't exist
touch "$rsync_queue"
touch "$rsync_temp"

# Function to process rsync queue
process_rsync_queue() {
    while true; do
        #if [ -s "$rsync_queue" ]; then
            echo "$(date): Processing queue" >> "$log_file"

            # Copy queue contents to a temporary file
            cp "$rsync_queue" "$rsync_temp"
            # Truncate the original queue file
            > "$rsync_queue"

            # Process each rsync command in the temp file
            while IFS= read -r command; do
                echo "$(date): Executing command: $command" >> "$log_file"
                eval "$command"

                # Check if rsync command was successful
                if [ $? -eq 0 ]; then
                    echo "$(date): Data transfer complete: $command" >> "$log_file"
                else
                    echo "$(date): Data transfer failed: $command. Retrying..." >> "$log_file"
                    # Requeue the failed command
                    echo "$command" >> "$rsync_queue"
                fi
            done < "$rsync_temp"

            # Remove the temporary file
            rm -f "$rsync_temp"
        #else
            #echo "$(date): Rsync queue is empty" >> "$log_file"
        #fi

        # Small delay before checking the queue again
        sleep 1
    done
}

# Start processing the rsync queue
process_rsync_queue
