import zmq
import json
import os
import socket
import subprocess
import sys
import time

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8",80))
ip = s.getsockname()[0]
nano_ID = ip.split('.')[-1]

# Add this to the server
# s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# s.connect(("8.8.8.8",80))
# ip = s.getsockname()[0]
# dev_id = ip.split('.')[-1]


context = zmq.Context()

socket = context.socket(zmq.REQ)
socket_s = context.socket(zmq.SUB) 

IP_addr = "192.168.0.21"

socket.connect("tcp://"+IP_addr+":55" + nano_ID)

socket_s.connect("tcp://"+IP_addr+":6666")
socket_s.setsockopt(zmq.SUBSCRIBE, b'')

go_par = True

sync_par = False

sync_delay = False

base_path = "/home/vigir/Images/"

def size_check(file_path, threshold = 1e6):

    try:
        size = os.path.getsize(file_path)
        if size > threshold:
            size_pass = True
        else:
            size_pass = False
    except:
        size_pass = False
    
    

    return size_pass





def capture_3D(site, case_path, case_number, server_no, master_ID, sync_par, sync_delay):
    ####
    case_file = open(case_path + "/" + "Animal_" + str(case_number) + "_nano_" + server_no + '.txt', 'w')
    if sync_par:

        if sync_delay:

            if master_ID == nano_ID:


                command = "k4arecorder --external-sync master -e 20000 -r 5 -l 1 -d WFOV_UNBINNED -c 1080p " + case_path + "/" + "Animal_" + str(case_number) + "_nano_" + server_no + ".mkv"
                os.system("export DISPLAY=:0")
                time.sleep(2)
            else:
                command = "k4arecorder --external-sync sub --sync-delay " + str((int(nano_ID[1])) * 160) + " -e 20000 -r 5 -l 1 -d WFOV_UNBINNED -c 1080p " + case_path + "/" + "Animal_" + str(case_number) + "_nano_" + server_no + ".mkv"
                os.system("export DISPLAY=:0")
        else:
            if master_ID == nano_ID:


                command = "k4arecorder --external-sync master -e 20000 -r 5 -l 1 -d WFOV_UNBINNED -c 1080p " + case_path + "/" + "Animal_" + str(case_number) + "_nano_" + server_no + ".mkv"
                os.system("export DISPLAY=:0")
                time.sleep(2)
            else:
                command = "k4arecorder --external-sync sub -e 20000 -r 5 -l 1 -d WFOV_UNBINNED -c 1080p " + case_path + "/" + "Animal_" + str(case_number) + "_nano_" + server_no + ".mkv"
                os.system("export DISPLAY=:0")
    else:
        command = "k4arecorder -r 5 -l 1 -d WFOV_UNBINNED -c 1080p " + case_path + "/" + "Animal_" + str(case_number) + "_nano_" + server_no + ".mkv"
        os.system("export DISPLAY=:0")
        time.sleep(2)


    
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
    time.sleep(10)
    os.system("rsync -av {source} vigir3d@192.168.0.21:/home/vigir3d/Datasets/cattle_scans/".format(source = "~/Images/"+ site + "/" + "Animal_" + str(case_number) + "_nano_" + server_no + ".mkv"))

    process_pass = process.wait()
    
    size_pass = size_check(case_path + "/" + "Animal_" + str(case_number) 
                + "_nano_" + server_no + ".mkv")

    if (process_pass == 0) and (size_pass == True):
        camera_pass = True
    else:
        camera_pass = False

    case_file.write("The capture process has been done!")
    case_file.close()
    return camera_pass




while True:

    socket.send_string("server " + nano_ID + " Ready")



    message1 = socket.recv()
    message1 = message1.decode("utf-8")
    message1 = json.loads(message1)

    mode_msg = socket_s.recv()
    mode_msg = mode_msg.decode("utf-8")
    mode_msg = json.loads(mode_msg)
    message = mode_msg
    print(message)

    

    
    print(message)
    if message[0] == "c":
        site = message[1]
        case_number = message[2]
        master_ID = message[3]
        site_path = os.path.join(base_path, site)
        #os.makedirs(site_path, exist_ok = True)
        os.makedirs(site_path, exist_ok = True)

        camera_pass = capture_3D(site, site_path, case_number, nano_ID, master_ID, sync_par, sync_delay)
        



        if camera_pass:
            socket.send_string("1")
        else:
            socket.send_string("0")
        message = socket.recv()
        message = message.decode("utf-8")
    
    elif message1[0] == "r":
        continue
    

    elif message1[0] == "s":
        site = message[1]
        case_number = message[2]
        site_path = os.path.join(base_path, site)
        os.system("rsync -av {source} vigir3d@192.168.0.21:/home/vigir3d/Datasets/cattle_scans/".format(source = site_path + "/" + "Animal_" + str(case_number) + "_nano_" + nano_ID + '.txt'))
        socket.send_string("Done")
        message = socket.recv()
        message = message.decode("utf-8")

    elif message1[0] == "w":
        os.system("/home/vigir/Documents/Azure-Kinect-Sensor-SDK/build/bin/viewer_opengl &")
        socket.send_string("Done")
        message = socket.recv()
        message = message.decode("utf-8")

    elif message1[0] == "wk":
        os.system("killall viewer_opengl")
        socket.send_string("Done")
        message = socket.recv()
        message = message.decode("utf-8")

    elif message1[0] == "dd":
        site = message[1]
        site_path = os.path.join(base_path, site)
        files = os.listdir(site_path)
        for obj in files:
            os.system("rsync -av --remove-source-files {source} vigir3d@192.168.0.21:/home/vigir3d/Datasets/cattle_scans/".format(source = site_path + "/" + str(obj)))
        socket.send_string("Done")
        message = socket.recv()
        message = message.decode("utf-8")

    elif message1[0] == "e":
        os.execl(sys.executable, sys.executable, *sys.argv)



