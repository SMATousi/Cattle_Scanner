#!/bin/bash

# The text file that the client program writes to
TEXT_FILE="/path/to/text_file.txt"

# The Python script you want to run
PYTHON_SCRIPT="/home/vigir3d/Software/programs/Cattle_Scanner/scanner_pipeline_code.py"

# Monitor the text file for changes
while inotifywait -e modify $TEXT_FILE; do
    # Read the directory path from the text file
    DIR_PATH=$(cat $TEXT_FILE)

    # Check if the directory exists
    if [ -d "$DIR_PATH" ]; then
        echo "Monitoring directory: $DIR_PATH"

        # Monitor the directory for the close_write event
        while inotifywait -e close_write $DIR_PATH; do
            echo "Detected a write operation in $DIR_PATH"

            # You might want to add some sleep here to ensure all writes are done
            sleep 10
        done

        # Now, invoke your Python script with the directory as an argument
        python $PYTHON_SCRIPT -i "$DIR_PATH" -t "$CALIB_PATH"
    else
        echo "$DIR_PATH does not exist or is not a directory."
    fi
done
