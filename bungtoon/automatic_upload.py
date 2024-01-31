"""ALL THE IMPORTS"""
from dronekit import connect, VehicleMode, Command
from math import radians, cos, sin, asin, sqrt
from Buutton import set_connection , btn_main
# import csv
# import math
from cmd_msg_dronekit import *
from Buutton import *
# from pymavlink import mavutil
# import time

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

#Start SITL if no connection string specified
if not connection_string:
    connection_string = set_connection
  


# Connect to the Vehicle. 
#   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
# if os.path.exists('mission_upload_state.txt'):
with open('mission_upload_state.txt', 'r') as f:
    print('Entering')
    state = f.read()
    if state == 'not complete':
        print("\nConnecting to vehicle on:", connection_string)
        vehicle = connect(connection_string)
        pixels.fill((255, 20, 0))
        time.sleep(5)
        pixels.fill((0,0,0))
        # #vehicle.wait_ready('autopilot_version')
        # driver code
        land_comp = landComp_safetySwitchOFF(vehicle)
        # Get commands object from Vehicle.
        if land_comp:
                gps_lat, gps_lon = gps_lat_lon(vehicle)
                # print('GPS Lat : ', gps_lat)
                # print('GPS Lon : ', gps_lon)
                mission_file_name = compareDist_retFName(gps_lat, gps_lon)
                # print('The mission file name is ', mission_file_name)
                # print('Uploading Mission')
                upload_mission(vehicle, mission_file_name)
                btn_main()
    else:
        btn_main()