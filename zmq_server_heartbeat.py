import zmq
import json
import os
import socket
import subprocess
import sys
import time

# Socket setup to get IP and Nano ID
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
ip = s.getsockname()[0]
nano_ID = ip.split('.')[-1]

context = zmq.Context()

# Sockets for ZeroMQ
socket = context.socket(zmq.REQ)
socket_s = context.socket(zmq.SUB)

IP_addr = "10.14.10.21"
socket.connect("tcp://"+IP_addr+":55" + nano_ID)
socket_s.connect("tcp://"+IP_addr+":6666")
socket_s.setsockopt(zmq.SUBSCRIBE, b'')

# Define paths
base_path = "/home/vigir/Images/"

# Heartbeat mechanism parameters
heartbeat_interval = 5  # Seconds between heartbeats
last_heartbeat = time.time()  # Record the last heartbeat

def size_check(file_path, threshold=1e6):
    """Check if file size exceeds a threshold."""
    try:
        size = os.path.getsize(file_path)
        return size > threshold
    except:
        return False

def capture_3D(site, case_path, case_number, server_no, master_ID, sync_par, sync_delay):
    """3D capture function with external sync."""
    case_file = open(case_path + "/" + "Animal_" + str(case_number) + "_nano_" + server_no + '.txt', 'w')
    # Command setup for k4a recording, adjusted based on nano_ID
    if master_ID == nano_ID:
        command = "k4arecorder --external-sync master -e 33330 -r 30 -l 1 -d NFOV_UNBINNED -c 1080p " + \
                  case_path + "/" + "Animal_" + str(case_number) + "_nano_" + server_no + ".mkv"
    else:
        command = "k4arecorder --external-sync sub --sync-delay " + str(int(nano_ID[1]) * 160) + \
                  " -e 33330 -r 30 -l 1 -d NFOV_UNBINNED -c 1080p " + \
                  case_path + "/" + "Animal_" + str(case_number) + "_nano_" + server_no + ".mkv"

    os.system("export DISPLAY=:0")
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
    process_pass = process.wait()
    size_pass = size_check(case_path + "/" + "Animal_" + str(case_number) + "_nano_" + server_no + ".mkv")

    camera_pass = process_pass == 0 and size_pass
    case_file.write("The capture process has been done!")
    case_file.close()

    return camera_pass

while True:
    try:
        # Send "Ready" status to client
        socket.send_string(f"server {nano_ID} Ready")
        
        # Check for heartbeats
        if time.time() - last_heartbeat > heartbeat_interval:
            print(f"No heartbeat from client. Exiting server {nano_ID}.")
            break  # This will allow systemd to restart the server

        # Initialize message1 with an empty dictionary for robustness
        message1 = {}

        # Receive the client message
        try:
            message1 = socket.recv(zmq.NOBLOCK).decode("utf-8")
            message1 = json.loads(message1)
            last_heartbeat = time.time()  # Update heartbeat on receiving a valid message
        except zmq.Again:
            pass  # No message received, continue

        # Handle client requests if a message was received
        if message1:
            if message1[0] == "c":
                site = message1[1]
                case_number = message1[2]
                master_ID = message1[3]
                sync_par = message1[4] == "True"
                site_path = os.path.join(base_path, site)
                os.makedirs(site_path, exist_ok=True)
                camera_pass = capture_3D(site, site_path, case_number, nano_ID, master_ID, sync_par, sync_delay)
                socket.send_string("1" if camera_pass else "0")

            elif message1[0] == "r":
                continue
            elif message1[0] == "s":
                # Handle rsync process
                site = message1[1]
                case_number = message1[2]
                site_path = os.path.join(base_path, site)
                os.system(f"rsync -av {site_path}/Animal_{case_number}_nano_{nano_ID}.txt vigir3d@10.14.10.21:/home/vigir3d/Datasets/cattle_scans/{site}/Animal_{case_number}/")
                socket.send_string("Done")
            elif message1[0] == "w":
                os.system("/home/vigir/Documents/Azure-Kinect-Sensor-SDK/build/bin/viewer_opengl &")
                socket.send_string("Done")
            elif message1[0] == "wk":
                os.system("killall viewer_opengl")
                socket.send_string("Done")
            elif message1[0] == "dd":
                site = message1[1]
                site_path = os.path.join(base_path, site)
                files = os.listdir(site_path)
                for obj in files:
                    os.system(f"rsync -av --remove-source-files {site_path}/{obj} vigir3d@10.14.10.21:/home/vigir3d/Datasets/cattle_scans/{site}/")
                socket.send_string("Done")
            elif message1[0] == "e":
                os.execl(sys.executable, sys.executable, *sys.argv)

    except Exception as e:
        print(f"Exception: {e}")
        break  # Exit to allow systemd to restart the service

# Close the socket at the end
socket.close()
socket_s.close()
