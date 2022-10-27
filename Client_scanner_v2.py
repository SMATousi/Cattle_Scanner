import os
import sys
import time
import zmq
import json
# from RPLCD.i2c import CharLCD


# def lcd_setup():
#     lcd = CharLCD('PCF8574', 0x27)
#     lcd.clear()
#     lcd.cursor_mode = 'hide'
#     lcd.cursor_pos = (0,5)
#     lcd.write_string('0123456789')
#     lcd.cursor_pos = (1,5)
#     return lcd


# def lcd_update(lcd, camera_idx,camera_state, animal_id) :
#         lcd.cursor_pos = (1,5+int(camera_idx))
#         lcd.write_string(camera_state)
#         lcd.cursor_pos = (0,0) # COW ID
#         lcd.write_string(str(animal_id)) 

# def m2xo(message):
#     if message == "1":
#         return "O"
#     else:
#         return "X"

ip = "192.168.0.21"

context = zmq.Context()

socket1 = context.socket(zmq.REP)
socket1.bind("tcp://192.168.0.21:5511")

socket2 = context.socket(zmq.REP)
socket2.bind("tcp://192.168.0.21:5512")

socket3 = context.socket(zmq.REP)
socket3.bind("tcp://192.168.0.21:5513")

socket4 = context.socket(zmq.REP)
socket4.bind("tcp://192.168.0.21:5514")

socket5 = context.socket(zmq.REP)
socket5.bind("tcp://192.168.0.21:5515")

socket6 = context.socket(zmq.REP)
socket6.bind("tcp://192.168.0.21:5516")

socket7 = context.socket(zmq.REP)
socket7.bind("tcp://192.168.0.21:5517")

socket8 = context.socket(zmq.REP)
socket8.bind("tcp://192.168.0.21:5518")

socket9 = context.socket(zmq.REP)
socket9.bind("tcp://192.168.0.21:5519")

socket10 = context.socket(zmq.REP)
socket10.bind("tcp://192.168.0.21:5520")

socket_p = context.socket(zmq.PUB)
socket_p.bind("tcp://192.168.0.21:6666")

class color:
   PURPLE = '\033[95m'
   CYAN = '\033[96m'
   DARKCYAN = '\033[36m'
   BLUE = '\033[94m'
   GREEN = '\033[92m'
   YELLOW = '\033[93m'
   RED = '\033[91m'
   BOLD = '\033[1m'
   UNDERLINE = '\033[4m'
   END = '\033[0m'



def camera_state(name, status):
    if status == "1":
        print("camera " + str(name) + " OK!")

    else:
        print(color.RED + color.BOLD + "The camera no." + str(name) + " is not working!" + color.END)



def log_writer(site, site_path, case_number, camera_stats):
    logs_file = open(site_path + '/site_.' + str(site) + '_logs.txt', 'a')
    logs_file.write("Site. " + str(site) + "/Animal No. <" + str(case_number) + "> / Cameras: ")
    for cam in camera_stats:
        if cam == "1":
            logs_file.write("O-")
        else:
            logs_file.write("X-")
    logs_file.write("\n")
    logs_file.close()

def main():
#    lcd = lcd_setup()
    while(True):

        

        print(color.GREEN + "Please select the mode: w for warm up the cameras, c for Capture, s for Survey, dd for Data dumping and e for ending the program:" + color.END)

        mode_char = input() #Reading the mode

            


        if mode_char == 'e':
            server1_message = socket1.recv() 
            server1_message = server1_message.decode("utf-8")
            server2_message = socket2.recv()
            server2_message = server2_message.decode("utf-8")
            server3_message = socket3.recv()
            server3_message = server3_message.decode("utf-8")
            server4_message = socket4.recv()
            server4_message = server4_message.decode("utf-8")
            server5_message = socket5.recv()
            server5_message = server5_message.decode("utf-8")
            server6_message = socket6.recv()
            server6_message = server6_message.decode("utf-8")
            server7_message = socket7.recv()
            server7_message = server7_message.decode("utf-8")
            server8_message = socket8.recv()
            server8_message = server8_message.decode("utf-8")
            server9_message = socket9.recv()
            server9_message = server9_message.decode("utf-8")
            server10_message = socket10.recv()
            server10_message = server10_message.decode("utf-8")

            msg = ["e"]

            
            if server1_message == "server 11 Ready":
                socket1.send_string(json.dumps(msg))
            if server2_message == "server 12 Ready":
                socket2.send_string(json.dumps(msg))
            if server3_message == "server 13 Ready":
                socket3.send_string(json.dumps(msg))
            if server4_message == "server 14 Ready":
                socket4.send_string(json.dumps(msg))
            if server5_message == "server 15 Ready":
                socket5.send_string(json.dumps(msg))
            if server6_message == "server 16 Ready":
                socket6.send_string(json.dumps(msg))
            if server7_message == "server 17 Ready":
                socket7.send_string(json.dumps(msg))
            if server8_message == "server 18 Ready":
                socket8.send_string(json.dumps(msg))
            if server9_message == "server 19 Ready":
                socket9.send_string(json.dumps(msg))
            if server10_message == "server 20 Ready":
                socket10.send_string(json.dumps(msg))
                
            socket_p.send_string(json.dumps(msg))


            break

        if mode_char == 'c':

            in_use_cameras = [1,2,3,4,5,6]
            default_master = "11"
            change_camera = input(color.GREEN + "In use cameras: " + str(in_use_cameras) + " The master camera:" + default_master + "/ if you want to change the default cameras: type CHANGE, otherwise press ENTER!" + color.END)

            if change_camera == "CHANGE":
                camera_order_ok = False
                while(not camera_order_ok):
                    print(color.BLUE + "Current order:" + str(in_use_cameras) + color.END)
                    cam_ch = input(color.GREEN + "If you are satisfied with this camera order: type stop. otherwise please specify the number of camera that you want to change:" + color.END)
                    if cam_ch == "stop":
                        break
                    else:
                        cam_in = input(color.GREEN + "Please insert the camera number that you want to use:" + color.END)
                        for i in range(len(in_use_cameras)):
                            if str(in_use_cameras[i]) == str(cam_ch):
                                # print("##############")
                                in_use_cameras[i] = int(cam_in)
                default_master = input(color.GREEN + "Please insert the master camera ID:" + color.END)
                            
            camera_choose = []
            for i in range(10):
                if (i+1) in in_use_cameras:
                    camera_choose.append(1)
                else:
                    camera_choose.append(0)
            print(str(camera_choose))


            #Reading the site name
            site = input(color.GREEN + "Please insert the site name:" + color.END)
            print(site)

            #Making a directory for the specified site
            site_path = os.path.join("./", site)
            os.makedirs(site_path, exist_ok = True)

            cap_mode = input(color.GREEN + "Please specify the capturing mode: A for Automatic increment, M for Manual:" + color.END)

            if cap_mode == "a":
                initial_case_number = input(color.GREEN + "Please insert the initial animal number:" + color.END)

                case_number = int(initial_case_number)
                while(True):

                    server1_message = socket1.recv() 
                    server1_message = server1_message.decode("utf-8")
                    server2_message = socket2.recv()
                    server2_message = server2_message.decode("utf-8")
                    server3_message = socket3.recv()
                    server3_message = server3_message.decode("utf-8")
                    server4_message = socket4.recv()
                    server4_message = server4_message.decode("utf-8")
                    server5_message = socket5.recv()
                    server5_message = server5_message.decode("utf-8")
                    server6_message = socket6.recv()
                    server6_message = server6_message.decode("utf-8")
                    server7_message = socket7.recv()
                    server7_message = server7_message.decode("utf-8")
                    server8_message = socket8.recv()
                    server8_message = server8_message.decode("utf-8")
                    server9_message = socket9.recv()
                    server9_message = server9_message.decode("utf-8")
                    server10_message = socket10.recv()
                    server10_message = server10_message.decode("utf-8")

                    ret = input(color.GREEN + "Processing case number: <" + str(case_number) + "> If you want to stop and return: r, otherwise Press ENTER to capture!:" + color.END)
                    if ret == "r":
                        msg = ["r"]
                        socket_p.send_string(json.dumps(msg))
                        socket1.send_string(json.dumps(msg))
                        socket2.send_string(json.dumps(msg))
                        socket3.send_string(json.dumps(msg))
                        socket4.send_string(json.dumps(msg))
                        socket5.send_string(json.dumps(msg))
                        socket6.send_string(json.dumps(msg))
                        socket7.send_string(json.dumps(msg))
                        socket8.send_string(json.dumps(msg))
                        socket9.send_string(json.dumps(msg))
                        socket10.send_string(json.dumps(msg))
                        break
                    else:
                        #msg = "c" + " " + site + " " + str(case_number)
                        msg = ["c", site, str(case_number), default_master]
                        print(msg)
                        if server1_message == "server 11 Ready":
                            socket1.send_string(json.dumps(msg))
                        if server2_message == "server 12 Ready":
                            socket2.send_string(json.dumps(msg))
                        if server3_message == "server 13 Ready":
                            socket3.send_string(json.dumps(msg))
                        if server4_message == "server 14 Ready":
                            socket4.send_string(json.dumps(msg))
                        if server5_message == "server 15 Ready":
                            socket5.send_string(json.dumps(msg))
                        if server6_message == "server 16 Ready":
                            socket6.send_string(json.dumps(msg))
                        if server7_message == "server 17 Ready":
                            socket7.send_string(json.dumps(msg))
                        if server8_message == "server 18 Ready":
                            socket8.send_string(json.dumps(msg))
                        if server9_message == "server 19 Ready":
                            socket9.send_string(json.dumps(msg))
                        if server10_message == "server 20 Ready":
                            socket10.send_string(json.dumps(msg))
                        

                        os.system("mkdir -p /home/vigir3d/Datasets/cattle_scans/" + site + "/Animal_" + str(case_number) + "/")
                        socket_p.send_string(json.dumps(msg))
                        


                        if server1_message == "server 11 Ready":
                            
                            server1_message = socket1.recv()
                            server1_message = server1_message.decode("utf-8")
                            camera_state(1, server1_message)
                            #lcd_update(lcd,1,m2xo(server1_message), case_number)                        
                            socket1.send_string("")
                            #print(client_message)
                            

                            
                        if server2_message == "server 12 Ready":
                            
                            server2_message = socket2.recv()
                            server2_message = server2_message.decode("utf-8")
                            camera_state(2, server2_message)
                            #lcd_update(lcd,2,m2xo(server2_message), case_number)
                            socket2.send_string("")
                            #print(client_message)

                        if server3_message == "server 13 Ready":
                            
                            server3_message = socket3.recv()
                            server3_message = server3_message.decode("utf-8")
                            camera_state(3, server3_message)
                           # lcd_update(lcd,3,m2xo(server3_message), case_number)
                            socket3.send_string("")
                            #print(client_message)

                        if server4_message == "server 14 Ready":
                            
                            server4_message = socket4.recv()
                            server4_message = server4_message.decode("utf-8")
                            camera_state(4, server4_message)
                            #lcd_update(lcd,4,m2xo(server4_message), case_number)
                            socket4.send_string("")
                            #print(client_message)

                        if server5_message == "server 15 Ready":
                            
                            server5_message = socket5.recv()
                            server5_message = server5_message.decode("utf-8")
                            camera_state(5, server5_message)
                            #lcd_update(lcd,5,m2xo(server5_message), case_number)
                            socket5.send_string("")
                            #print(client_message)

                        if server6_message == "server 16 Ready":
                            
                            server6_message = socket6.recv()
                            server6_message = server6_message.decode("utf-8")
                            camera_state(6, server6_message)
                            #lcd_update(lcd,6,m2xo(server6_message), case_number)
                            socket6.send_string("")
                            #print(client_message)

                        if server7_message == "server 17 Ready":
                            
                            server7_message = socket7.recv()
                            server7_message = server7_message.decode("utf-8")
                            camera_state(7, server7_message)
                            #lcd_update(lcd,7,m2xo(server7_message), case_number)
                            socket7.send_string("")
                            #print(client_message)

                        if server8_message == "server 18 Ready":
                            
                            server8_message = socket8.recv()
                            server8_message = server8_message.decode("utf-8")
                            camera_state(8, server8_message)
                            #lcd_update(lcd,8,m2xo(server8_message), case_number)
                            socket8.send_string("")
                            #print(client_message)

                        if server9_message == "server 19 Ready":
                            
                            server9_message = socket9.recv()
                            server9_message = server9_message.decode("utf-8")
                            camera_state(9, server9_message)
                            #lcd_update(lcd,9,m2xo(server9_message), case_number)
                            socket9.send_string("")
                            #print(client_message)

                        if server10_message == "server 20 Ready":
                            
                            server10_message = socket10.recv()
                            server10_message = server10_message.decode("utf-8")
                            camera_state(10, server10_message)
                            #lcd_update(lcd,10,m2xo(server10_message), case_number)
                            socket10.send_string("")
                            #print(client_message)
                            
                        else:
                            break
                        
                        camera_stats = [server1_message, server2_message, server3_message, server4_message, server5_message, server6_message, server7_message, server8_message, server9_message, server10_message]
                        log_writer(site, site_path, case_number, camera_stats)
                    
                        if server1_message == "0" and camera_choose[0]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(1) + color.END)
                        elif server2_message == "0" and camera_choose[1]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(2) + color.END)
                        elif server3_message == "0" and camera_choose[2]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(3) + color.END)
                        elif server4_message == "0" and camera_choose[3]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(4) + color.END)
                        elif server5_message == "0" and camera_choose[4]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(5) + color.END)
                        elif server6_message == "0" and camera_choose[5]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(6) + color.END)
                        elif server7_message == "0" and camera_choose[6]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(7) + color.END)
                        elif server8_message == "0" and camera_choose[7]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(8) + color.END)
                        elif server9_message == "0" and camera_choose[8]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(9) + color.END)
                        elif server10_message == "0" and camera_choose[9]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(10) + color.END)
                        else:
                            case_number = case_number + 1
                            
            elif cap_mode == "m":
                while(True):
                    server1_message = socket1.recv() 
                    server1_message = server1_message.decode("utf-8")
                    server2_message = socket2.recv()
                    server2_message = server2_message.decode("utf-8")
                    server3_message = socket3.recv()
                    server3_message = server3_message.decode("utf-8")
                    server4_message = socket4.recv()
                    server4_message = server4_message.decode("utf-8")
                    server5_message = socket5.recv()
                    server5_message = server5_message.decode("utf-8")
                    server6_message = socket6.recv()
                    server6_message = server6_message.decode("utf-8")
                    server7_message = socket7.recv()
                    server7_message = server7_message.decode("utf-8")
                    server8_message = socket8.recv()
                    server8_message = server8_message.decode("utf-8")
                    server9_message = socket9.recv()
                    server9_message = server9_message.decode("utf-8")
                    server10_message = socket10.recv()
                    server10_message = server10_message.decode("utf-8")

                    case_number = input(color.GREEN + "If you want to stop and return: r, Please insert the case number and press ENTER to capture:" + color.END)
                    if case_number == "r":
                        msg = ["r"]
                        socket_p.send_string(json.dumps(msg))
                        socket1.send_string(json.dumps(msg))
                        socket2.send_string(json.dumps(msg))
                        socket3.send_string(json.dumps(msg))
                        socket4.send_string(json.dumps(msg))
                        socket5.send_string(json.dumps(msg))
                        socket6.send_string(json.dumps(msg))
                        socket7.send_string(json.dumps(msg))
                        socket8.send_string(json.dumps(msg))
                        socket9.send_string(json.dumps(msg))
                        socket10.send_string(json.dumps(msg))
                        break
                    else:



                        msg = ["c", site, str(case_number), default_master]
                        msg = json.dumps(msg)

                        if server1_message == "server 11 Ready":
                                socket1.send_string(json.dumps(msg))
                        if server2_message == "server 12 Ready":
                            socket2.send_string(json.dumps(msg))
                        if server3_message == "server 13 Ready":
                            socket3.send_string(json.dumps(msg))
                        if server4_message == "server 14 Ready":
                            socket4.send_string(json.dumps(msg))
                        if server5_message == "server 15 Ready":
                            socket5.send_string(json.dumps(msg))
                        if server6_message == "server 16 Ready":
                            socket6.send_string(json.dumps(msg))
                        if server7_message == "server 17 Ready":
                            socket7.send_string(json.dumps(msg))
                        if server8_message == "server 18 Ready":
                            socket8.send_string(json.dumps(msg))
                        if server9_message == "server 19 Ready":
                            socket9.send_string(json.dumps(msg))
                        if server10_message == "server 20 Ready":
                            socket10.send_string(json.dumps(msg))

                        os.system("mkdir -p /home/vigir3d/Datasets/cattle_scans/" + site + "/Animal_" + str(case_number) + "/")
                        socket_p.send_string(msg)

                        if server1_message == "server 11 Ready":
                            
                            server1_message = socket1.recv()
                            server1_message = server1_message.decode("utf-8")
                            camera_state(1, server1_message)                        
                            socket1.send_string("")
                            #print(client_message)
                            

                            
                        if server2_message == "server 12 Ready":
                            
                            server2_message = socket2.recv()
                            server2_message = server2_message.decode("utf-8")
                            camera_state(2, server2_message)
                            socket2.send_string("")
                            #print(client_message)

                        if server3_message == "server 13 Ready":
                            
                            server3_message = socket3.recv()
                            server3_message = server3_message.decode("utf-8")
                            camera_state(3, server3_message)
                            socket3.send_string("")
                            #print(client_message)

                        if server4_message == "server 14 Ready":
                            
                            server4_message = socket4.recv()
                            server4_message = server4_message.decode("utf-8")
                            camera_state(2, server4_message)
                            socket4.send_string("")
                            #print(client_message)

                        if server5_message == "server 15 Ready":
                            
                            server5_message = socket5.recv()
                            server5_message = server5_message.decode("utf-8")
                            camera_state(5, server5_message)
                            socket5.send_string("")
                            #print(client_message)

                        if server6_message == "server 16 Ready":
                            
                            server6_message = socket6.recv()
                            server6_message = server6_message.decode("utf-8")
                            camera_state(6, server6_message)
                            socket6.send_string("")
                            #print(client_message)

                        if server7_message == "server 17 Ready":
                            
                            server7_message = socket7.recv()
                            server7_message = server7_message.decode("utf-8")
                            camera_state(7, server7_message)
                            socket7.send_string("")
                            #print(client_message)

                        if server8_message == "server 18 Ready":
                            
                            server8_message = socket8.recv()
                            server8_message = server8_message.decode("utf-8")
                            camera_state(8, server8_message)
                            socket8.send_string("")
                            #print(client_message)

                        if server9_message == "server 19 Ready":
                            
                            server9_message = socket9.recv()
                            server9_message = server9_message.decode("utf-8")
                            camera_state(9, server9_message)
                            socket9.send_string("")
                            #print(client_message)

                        if server10_message == "server 20 Ready":
                            
                            server10_message = socket10.recv()
                            server10_message = server10_message.decode("utf-8")
                            camera_state(10, server10_message)
                            socket10.send_string("")
                            #print(client_message)
                            
                        
                        camera_stats = [server1_message, server2_message, server3_message, server4_message, server5_message, server6_message, server7_message, server8_message, server9_message, server10_message]
                        log_writer(site, site_path, case_number, camera_stats)
                    
                        if server1_message == "0" and camera_choose[0]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(1) + color.END)
                        elif server2_message == "0" and camera_choose[1]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(2) + color.END)
                        elif server3_message == "0" and camera_choose[2]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(3) + color.END)
                        elif server4_message == "0" and camera_choose[3]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(4) + color.END)
                        elif server5_message == "0" and camera_choose[4]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(5) + color.END)
                        elif server6_message == "0" and camera_choose[5]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(6) + color.END)
                        elif server7_message == "0" and camera_choose[6]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(7) + color.END)
                        elif server8_message == "0" and camera_choose[7]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(8) + color.END)
                        elif server9_message == "0" and camera_choose[8]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(9) + color.END)
                        elif server10_message == "0" and camera_choose[9]:
                            print(color.RED + color.BOLD + "Please fix the camera No." + str(10) + color.END)
                        



                            





        elif mode_char == 's':

            while True:

                server1_message = socket1.recv() 
                server1_message = server1_message.decode("utf-8")
                server2_message = socket2.recv()
                server2_message = server2_message.decode("utf-8")
                server3_message = socket3.recv()
                server3_message = server3_message.decode("utf-8")
                server4_message = socket4.recv()
                server4_message = server4_message.decode("utf-8")
                server5_message = socket5.recv()
                server5_message = server5_message.decode("utf-8")
                server6_message = socket6.recv()
                server6_message = server6_message.decode("utf-8")
                server7_message = socket7.recv()
                server7_message = server7_message.decode("utf-8")
                server8_message = socket8.recv()
                server8_message = server8_message.decode("utf-8")
                server9_message = socket9.recv()
                server9_message = server9_message.decode("utf-8")
                server10_message = socket10.recv()
                server10_message = server10_message.decode("utf-8")

                site = input(color.GREEN + "For return insert: r, Otherwise please insert the site name:" + color.END)
                if site == "r":
                    msg = ["r"]
                    socket_p.send_string(json.dumps(msg))
                    socket1.send_string(json.dumps(msg))
                    socket2.send_string(json.dumps(msg))
                    socket3.send_string(json.dumps(msg))
                    socket4.send_string(json.dumps(msg))
                    socket5.send_string(json.dumps(msg))
                    socket6.send_string(json.dumps(msg))
                    socket7.send_string(json.dumps(msg))
                    socket8.send_string(json.dumps(msg))
                    socket9.send_string(json.dumps(msg))
                    socket10.send_string(json.dumps(msg))
                    break
                case_number = input(color.GREEN + "Please insert the case number:" + color.END)
                msg = ["s", site, str(case_number)]
                socket_p.send_string(json.dumps(msg))
                socket1.send_string(json.dumps(msg))
                socket2.send_string(json.dumps(msg))
                socket3.send_string(json.dumps(msg))
                socket4.send_string(json.dumps(msg))
                socket5.send_string(json.dumps(msg))
                socket6.send_string(json.dumps(msg))
                socket7.send_string(json.dumps(msg))
                socket8.send_string(json.dumps(msg))
                socket9.send_string(json.dumps(msg))
                socket10.send_string(json.dumps(msg))

                server1_message = socket1.recv() 
                server1_message = server1_message.decode("utf-8")
                print(server1_message)
                socket1.send_string("")

                server2_message = socket2.recv()
                server2_message = server2_message.decode("utf-8")
                print(server2_message)
                socket2.send_string("")

                server3_message = socket3.recv() 
                server3_message = server3_message.decode("utf-8")
                print(server3_message)
                socket3.send_string("")

                server4_message = socket4.recv() 
                server4_message = server4_message.decode("utf-8")
                print(server4_message)
                socket4.send_string("")

                server5_message = socket5.recv() 
                server5_message = server5_message.decode("utf-8")
                print(server5_message)
                socket5.send_string("")

                server6_message = socket6.recv() 
                server6_message = server6_message.decode("utf-8")
                print(server6_message)
                socket6.send_string("")

                server7_message = socket7.recv() 
                server7_message = server7_message.decode("utf-8")
                print(server7_message)
                socket7.send_string("")

                server8_message = socket8.recv() 
                server8_message = server8_message.decode("utf-8")
                print(server8_message)
                socket8.send_string("")

                server9_message = socket9.recv() 
                server9_message = server9_message.decode("utf-8")
                print(server9_message)
                socket9.send_string("")

                server10_message = socket10.recv() 
                server10_message = server10_message.decode("utf-8")
                print(server10_message)
                socket10.send_string("")




<<<<<<< HEAD
        elif mode_char == 'w':
            while True:

                server1_message = socket1.recv() 
                server1_message = server1_message.decode("utf-8")
                server2_message = socket2.recv()
                server2_message = server2_message.decode("utf-8")
                server3_message = socket3.recv()
                server3_message = server3_message.decode("utf-8")
                server4_message = socket4.recv()
                server4_message = server4_message.decode("utf-8")
                server5_message = socket5.recv()
                server5_message = server5_message.decode("utf-8")
                server6_message = socket6.recv()
                server6_message = server6_message.decode("utf-8")
                server7_message = socket7.recv()
                server7_message = server7_message.decode("utf-8")
                server8_message = socket8.recv()
                server8_message = server8_message.decode("utf-8")
                server9_message = socket9.recv()
                server9_message = server9_message.decode("utf-8")
                server10_message = socket10.recv()
                server10_message = server10_message.decode("utf-8")

                site = input(color.GREEN + "For return insert: r, Otherwise press ENTER to start warming up:" + color.END)
                if site == "r":
                    msg = ["r"]
                    socket_p.send_string(json.dumps(msg))
                    socket1.send_string(json.dumps(msg))
                    socket2.send_string(json.dumps(msg))
                    socket3.send_string(json.dumps(msg))
                    socket4.send_string(json.dumps(msg))
                    socket5.send_string(json.dumps(msg))
                    socket6.send_string(json.dumps(msg))
                    socket7.send_string(json.dumps(msg))
                    socket8.send_string(json.dumps(msg))
                    socket9.send_string(json.dumps(msg))
                    socket10.send_string(json.dumps(msg))
                    break
                
                print("Warm up mode")

                msg = ["w"]
                socket_p.send_string(json.dumps(msg))
                socket1.send_string(json.dumps(msg))
                socket2.send_string(json.dumps(msg))
                socket3.send_string(json.dumps(msg))
                socket4.send_string(json.dumps(msg))
                socket5.send_string(json.dumps(msg))
                socket6.send_string(json.dumps(msg))
                socket7.send_string(json.dumps(msg))
                socket8.send_string(json.dumps(msg))
                socket9.send_string(json.dumps(msg))
                socket10.send_string(json.dumps(msg))

                server1_message = socket1.recv() 
                server1_message = server1_message.decode("utf-8")
                print(server1_message)
                socket1.send_string("")

                server2_message = socket2.recv()
                server2_message = server2_message.decode("utf-8")
                print(server2_message)
                socket2.send_string("")

                server3_message = socket3.recv() 
                server3_message = server3_message.decode("utf-8")
                print(server3_message)
                socket3.send_string("")

                server4_message = socket4.recv() 
                server4_message = server4_message.decode("utf-8")
                print(server4_message)
                socket4.send_string("")

                server5_message = socket5.recv() 
                server5_message = server5_message.decode("utf-8")
                print(server5_message)
                socket5.send_string("")

                server6_message = socket6.recv() 
                server6_message = server6_message.decode("utf-8")
                print(server6_message)
                socket6.send_string("")

                server7_message = socket7.recv() 
                server7_message = server7_message.decode("utf-8")
                print(server7_message)
                socket7.send_string("")

                server8_message = socket8.recv() 
                server8_message = server8_message.decode("utf-8")
                print(server8_message)
                socket8.send_string("")

                server9_message = socket9.recv() 
                server9_message = server9_message.decode("utf-8")
                print(server9_message)
                socket9.send_string("")

                server10_message = socket10.recv() 
                server10_message = server10_message.decode("utf-8")
                print(server10_message)
                socket10.send_string("")

                site = input(color.RED + "NOW WARMING UP...................To end the warming up process: press ENTER" + color.END)

                server1_message = socket1.recv() 
                server1_message = server1_message.decode("utf-8")
                server2_message = socket2.recv()
                server2_message = server2_message.decode("utf-8")
                server3_message = socket3.recv()
                server3_message = server3_message.decode("utf-8")
                server4_message = socket4.recv()
                server4_message = server4_message.decode("utf-8")
                server5_message = socket5.recv()
                server5_message = server5_message.decode("utf-8")
                server6_message = socket6.recv()
                server6_message = server6_message.decode("utf-8")
                server7_message = socket7.recv()
                server7_message = server7_message.decode("utf-8")
                server8_message = socket8.recv()
                server8_message = server8_message.decode("utf-8")
                server9_message = socket9.recv()
                server9_message = server9_message.decode("utf-8")
                server10_message = socket10.recv()
                server10_message = server10_message.decode("utf-8")

                msg = ["wk"]
                socket_p.send_string(json.dumps(msg))
                socket1.send_string(json.dumps(msg))
                socket2.send_string(json.dumps(msg))
                socket3.send_string(json.dumps(msg))
                socket4.send_string(json.dumps(msg))
                socket5.send_string(json.dumps(msg))
                socket6.send_string(json.dumps(msg))
                socket7.send_string(json.dumps(msg))
                socket8.send_string(json.dumps(msg))
                socket9.send_string(json.dumps(msg))
                socket10.send_string(json.dumps(msg))

                server1_message = socket1.recv() 
                server1_message = server1_message.decode("utf-8")
                print(server1_message)
                socket1.send_string("")

                server2_message = socket2.recv()
                server2_message = server2_message.decode("utf-8")
                print(server2_message)
                socket2.send_string("")

                server3_message = socket3.recv() 
                server3_message = server3_message.decode("utf-8")
                print(server3_message)
                socket3.send_string("")

                server4_message = socket4.recv() 
                server4_message = server4_message.decode("utf-8")
                print(server4_message)
                socket4.send_string("")

                server5_message = socket5.recv() 
                server5_message = server5_message.decode("utf-8")
                print(server5_message)
                socket5.send_string("")
||||||| merged common ancestors
=======
        elif mode_char == 'w':
            while True:

                server1_message = socket1.recv() 
                server1_message = server1_message.decode("utf-8")
                server2_message = socket2.recv()
                server2_message = server2_message.decode("utf-8")
                server3_message = socket3.recv()
                server3_message = server3_message.decode("utf-8")
                server4_message = socket4.recv()
                server4_message = server4_message.decode("utf-8")
                server5_message = socket5.recv()
                server5_message = server5_message.decode("utf-8")
                server6_message = socket6.recv()
                server6_message = server6_message.decode("utf-8")
                server7_message = socket7.recv()
                server7_message = server7_message.decode("utf-8")
                server8_message = socket8.recv()
                server8_message = server8_message.decode("utf-8")
                server9_message = socket9.recv()
                server9_message = server9_message.decode("utf-8")
                server10_message = socket10.recv()
                server10_message = server10_message.decode("utf-8")

                site = input(color.GREEN + "For return insert: r, Otherwise press ENTER to start warming up:" + color.END)
                if site == "r":
                    msg = ["r"]
                    socket_p.send_string(json.dumps(msg))
                    socket1.send_string(json.dumps(msg))
                    socket2.send_string(json.dumps(msg))
                    socket3.send_string(json.dumps(msg))
                    socket4.send_string(json.dumps(msg))
                    socket5.send_string(json.dumps(msg))
                    socket6.send_string(json.dumps(msg))
                    socket7.send_string(json.dumps(msg))
                    socket8.send_string(json.dumps(msg))
                    socket9.send_string(json.dumps(msg))
                    socket10.send_string(json.dumps(msg))
                    break
                
                print("Warm up mode")

                msg = ["w"]
                socket_p.send_string(json.dumps(msg))
                socket1.send_string(json.dumps(msg))
                socket2.send_string(json.dumps(msg))
                socket3.send_string(json.dumps(msg))
                socket4.send_string(json.dumps(msg))
                socket5.send_string(json.dumps(msg))
                socket6.send_string(json.dumps(msg))
                socket7.send_string(json.dumps(msg))
                socket8.send_string(json.dumps(msg))
                socket9.send_string(json.dumps(msg))
                socket10.send_string(json.dumps(msg))

                server1_message = socket1.recv() 
                server1_message = server1_message.decode("utf-8")
                print(server1_message)
                socket1.send_string("")

                server2_message = socket2.recv()
                server2_message = server2_message.decode("utf-8")
                print(server2_message)
                socket2.send_string("")

                server3_message = socket3.recv() 
                server3_message = server3_message.decode("utf-8")
                print(server3_message)
                socket3.send_string("")

                server4_message = socket4.recv() 
                server4_message = server4_message.decode("utf-8")
                print(server4_message)
                socket4.send_string("")

                server5_message = socket5.recv() 
                server5_message = server5_message.decode("utf-8")
                print(server5_message)
                socket5.send_string("")

                server6_message = socket6.recv() 
                server6_message = server6_message.decode("utf-8")
                print(server6_message)
                socket6.send_string("")
>>>>>>> 08486542ef56c5441bbaf5131cc41812132b5c29

<<<<<<< HEAD
                server6_message = socket6.recv() 
                server6_message = server6_message.decode("utf-8")
                print(server6_message)
                socket6.send_string("")

                server7_message = socket7.recv() 
                server7_message = server7_message.decode("utf-8")
                print(server7_message)
                socket7.send_string("")

                server8_message = socket8.recv() 
                server8_message = server8_message.decode("utf-8")
                print(server8_message)
                socket8.send_string("")

                server9_message = socket9.recv() 
                server9_message = server9_message.decode("utf-8")
                print(server9_message)
                socket9.send_string("")

                server10_message = socket10.recv() 
                server10_message = server10_message.decode("utf-8")
                print(server10_message)
                socket10.send_string("")
||||||| merged common ancestors
=======
                server7_message = socket7.recv() 
                server7_message = server7_message.decode("utf-8")
                print(server7_message)
                socket7.send_string("")

                server8_message = socket8.recv() 
                server8_message = server8_message.decode("utf-8")
                print(server8_message)
                socket8.send_string("")

                server9_message = socket9.recv() 
                server9_message = server9_message.decode("utf-8")
                print(server9_message)
                socket9.send_string("")

                server10_message = socket10.recv() 
                server10_message = server10_message.decode("utf-8")
                print(server10_message)
                socket10.send_string("")

                site = input(color.RED + "NOW WARMING UP...................To end the warming up process: press ENTER" + color.END)

                msg = ["wk"]
                socket_p.send_string(json.dumps(msg))
                socket1.send_string(json.dumps(msg))
                socket2.send_string(json.dumps(msg))
                socket3.send_string(json.dumps(msg))
                socket4.send_string(json.dumps(msg))
                socket5.send_string(json.dumps(msg))
                socket6.send_string(json.dumps(msg))
                socket7.send_string(json.dumps(msg))
                socket8.send_string(json.dumps(msg))
                socket9.send_string(json.dumps(msg))
                socket10.send_string(json.dumps(msg))

                server1_message = socket1.recv() 
                server1_message = server1_message.decode("utf-8")
                print(server1_message)
                socket1.send_string("")

                server2_message = socket2.recv()
                server2_message = server2_message.decode("utf-8")
                print(server2_message)
                socket2.send_string("")

                server3_message = socket3.recv() 
                server3_message = server3_message.decode("utf-8")
                print(server3_message)
                socket3.send_string("")

                server4_message = socket4.recv() 
                server4_message = server4_message.decode("utf-8")
                print(server4_message)
                socket4.send_string("")

                server5_message = socket5.recv() 
                server5_message = server5_message.decode("utf-8")
                print(server5_message)
                socket5.send_string("")

                server6_message = socket6.recv() 
                server6_message = server6_message.decode("utf-8")
                print(server6_message)
                socket6.send_string("")

                server7_message = socket7.recv() 
                server7_message = server7_message.decode("utf-8")
                print(server7_message)
                socket7.send_string("")

                server8_message = socket8.recv() 
                server8_message = server8_message.decode("utf-8")
                print(server8_message)
                socket8.send_string("")

                server9_message = socket9.recv() 
                server9_message = server9_message.decode("utf-8")
                print(server9_message)
                socket9.send_string("")

                server10_message = socket10.recv() 
                server10_message = server10_message.decode("utf-8")
                print(server10_message)
                socket10.send_string("")
>>>>>>> 08486542ef56c5441bbaf5131cc41812132b5c29

        elif mode_char == 'dd':
            while True:

                server1_message = socket1.recv() 
                server1_message = server1_message.decode("utf-8")
                server2_message = socket2.recv()
                server2_message = server2_message.decode("utf-8")
                server3_message = socket3.recv()
                server3_message = server3_message.decode("utf-8")
                server4_message = socket4.recv()
                server4_message = server4_message.decode("utf-8")
                server5_message = socket5.recv()
                server5_message = server5_message.decode("utf-8")
                server6_message = socket6.recv()
                server6_message = server6_message.decode("utf-8")
                server7_message = socket7.recv()
                server7_message = server7_message.decode("utf-8")
                server8_message = socket8.recv()
                server8_message = server8_message.decode("utf-8")
                server9_message = socket9.recv()
                server9_message = server9_message.decode("utf-8")
                server10_message = socket10.recv()
                server10_message = server10_message.decode("utf-8")

                site = input(color.GREEN + "For return insert: r, Otherwise please insert the site name:" + color.END)
                if site == "r":
                    msg = ["r"]
                    socket_p.send_string(json.dumps(msg))
                    socket1.send_string(json.dumps(msg))
                    socket2.send_string(json.dumps(msg))
                    socket3.send_string(json.dumps(msg))
                    socket4.send_string(json.dumps(msg))
                    socket5.send_string(json.dumps(msg))
                    socket6.send_string(json.dumps(msg))
                    socket7.send_string(json.dumps(msg))
                    socket8.send_string(json.dumps(msg))
                    socket9.send_string(json.dumps(msg))
                    socket10.send_string(json.dumps(msg))
                    break
                
                print("Data dumping mode")

                msg = ["dd", site]
                socket_p.send_string(json.dumps(msg))
                socket1.send_string(json.dumps(msg))
                socket2.send_string(json.dumps(msg))
                socket3.send_string(json.dumps(msg))
                socket4.send_string(json.dumps(msg))
                socket5.send_string(json.dumps(msg))
                socket6.send_string(json.dumps(msg))
                socket7.send_string(json.dumps(msg))
                socket8.send_string(json.dumps(msg))
                socket9.send_string(json.dumps(msg))
                socket10.send_string(json.dumps(msg))

                server1_message = socket1.recv() 
                server1_message = server1_message.decode("utf-8")
                print(server1_message)
                socket1.send_string("")

                server2_message = socket2.recv()
                server2_message = server2_message.decode("utf-8")
                print(server2_message)
                socket2.send_string("")

                server3_message = socket3.recv() 
                server3_message = server3_message.decode("utf-8")
                print(server3_message)
                socket3.send_string("")

                server4_message = socket4.recv() 
                server4_message = server4_message.decode("utf-8")
                print(server4_message)
                socket4.send_string("")

                server5_message = socket5.recv() 
                server5_message = server5_message.decode("utf-8")
                print(server5_message)
                socket5.send_string("")

                server6_message = socket6.recv() 
                server6_message = server6_message.decode("utf-8")
                print(server6_message)
                socket6.send_string("")

                server7_message = socket7.recv() 
                server7_message = server7_message.decode("utf-8")
                print(server7_message)
                socket7.send_string("")

                server8_message = socket8.recv() 
                server8_message = server8_message.decode("utf-8")
                print(server8_message)
                socket8.send_string("")

                server9_message = socket9.recv() 
                server9_message = server9_message.decode("utf-8")
                print(server9_message)
                socket9.send_string("")

                server10_message = socket10.recv() 
                server10_message = server10_message.decode("utf-8")
                print(server10_message)
                socket10.send_string("")



        else:
            print("The input is not valid!")
if __name__=='__main__':
    main()
