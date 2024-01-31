from dronekit import connect, VehicleMode, mavutil, Command
from math import radians, cos, sin, asin, sqrt
import csv
import time
#import RPi.GPIO as GPIO
from gpiozero import GPIO
import neopixel
import board
# from cmd_msg_dronekit import *
# from pymavlink import mavutil
import os

num_pixels = 2        #declare the number of NeoPixels
pixel_pin = board.D18   #declare the NeoPixel out put pin
pixels = neopixel.NeoPixel(
                                    pixel_pin, num_pixels, brightness=1.5
                                )

def landComp_safetySwitchOFF(master):
    land_comp_throttle_disarmed = land_Complete_Or_Not(master)
    # print('LAnd complete message')
    # throttle_disarmed = throttle_Disarmed_Or_Not(master)
    # print('throttle disarmed triggered')
    # vehicle = mavutil.mavlink_connection(connection_string)

    if land_comp_throttle_disarmed:
        # print('safety switch on')
        time.sleep(2)
        set_safty_sw_state(master, sw_state=0)
        time.sleep(1)
        return True
    
    return False

def distance(lat1, lat2, lon1, lon2):
    # The math module contains a function named
    # radians which converts from degrees to radians.
    lon1 = radians(lon1)
    lon2 = radians(lon2)
    lat1 = radians(lat1)
    lat2 = radians(lat2)
      
    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
 
    c = 2 * asin(sqrt(a))
    
    # Radius of earth in kilometers. Use 3956 for miles
    r = 6371
      
    # calculate the result
    return(c * r)


def compareDist_retFName(gps_lat, gps_lon):
    min_distance = float('inf')
    min_filename = None

    with open('node_data_id/Node_Data_base.csv', 'r') as file:
        csv_reader = csv.reader(file)
        next(csv_reader) 

        for row in csv_reader:
            node_lat = float(row[2])
            node_lon = float(row[3])
            node_filename = row[4]

            node_distance = distance(gps_lat, node_lat, gps_lon, node_lon)

            if node_distance < min_distance:
                min_distance = node_distance
                min_filename = node_filename
        
        if min_distance <= 0.02:
            return min_filename

    return None


def gps_lat_lon(vehicle):
        # Get the current GPS coordinates
        latitude = vehicle.location.global_frame.lat
        longitude = vehicle.location.global_frame.lon
        return latitude, longitude

def clear_mission(vehicle):
        cmds = vehicle.commands
        cmds.download()
        vehicle.commands.wait_ready()
        cmds.clear()
        cmds.upload()
        time.sleep(1)
        return


def readmission(vehicle, aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

    This function is used by upload_mission().
    """
    aFileName = 'missions/' + aFileName + '.waypoints'
    print("\nReading mission from file: ", aFileName)
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=int(float(linearray[8]) * 1e7)
                ln_param6=int(float(linearray[9]) * 1e7)
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist



def upload_mission(vehicle, aFileName):
    """
    Upload a mission from a file. 
    """
    if aFileName is None:
        clear_mission(vehicle)
        pixels.fill((255, 0, 0))
        return False
    #Read mission from file
    missionlist = readmission(vehicle, aFileName)
    
    print("\nUpload mission from a file: ", aFileName)
    #Add new mission to vehicle
    cmds = vehicle.commands
    cmds.clear()
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()
    restart_mission(vehicle)
    time.sleep(1)
    vehicle.close()
    with open('mission_upload_state.txt', 'w') as f:
        f.write('complete')
    pixels.fill((0, 255, 0))
    time.sleep(10)
    pixels.fill((0,0,0))
    return True


def land_Complete_Or_Not(vehicle):
    land_comp = False
    throttle_disarmed = False
    @vehicle.on_message('STATUSTEXT')
    def handle_status_text(self, name, msg):
        print(msg.text)
        if msg.text == "Land complete":
             nonlocal land_comp 
             land_comp = True
        if msg.text == "Throttle disarmed":
             nonlocal throttle_disarmed
             throttle_disarmed = True

    while not (land_comp and throttle_disarmed):  #come out of the loop only when both land_comp and throttle_disarmed
        time.sleep(1)

    return land_comp and throttle_disarmed

def set_safty_sw_state(vehicle, sw_state):
    msg = vehicle.message_factory.set_mode_encode(0,mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY, not sw_state)
    vehicle.send_mavlink(msg)


def throttle_Disarmed_Or_Not(vehicle):
    throttle_disarmed = False

    @vehicle.on_message('STATUSTEXT')
    def handle_status_text1(self, name, msg):
        # print('Waiting for throttle disarm')
        if msg.text == "Throttle disarmed":
            print(msg.text)
            nonlocal throttle_disarmed
            throttle_disarmed = True

    while not throttle_disarmed:
        pass

    return throttle_disarmed

def toggle_safety_switch(vehicle):
    if vehicle.armed:
        vehicle.armed = False
        # print("Safety switch turned off")
    else:
        vehicle.armed = True
        # print("Safety switch turned on")


def restart_mission(vehicle):
    # Clear the current mission
    cmds = vehicle.commands
    vehicle.commands.next=1
    # print("Mission restarted.")
    time.sleep(1) 