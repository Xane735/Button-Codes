from dronekit import connect, VehicleMode, mavutil, Command
from pymavlink import mavutil
from math import radians, cos, sin, asin, sqrt
import time
#import RPi.GPIO as GPIO#uncomment this in rraspi, yellow line is annoying
from Buutton import the_connection, num_pixels, pixel_pin,board,neopixel,pixels

def landComp_safetySwitchOFF(master):
    monitor_sim_hit_ground = land_Complete_Or_Not(master)
    print('landed')
    pixels.fill((73,216,230))#light blue

    if monitor_sim_hit_ground:
        print('go near drone bitch')
        pixels.fill((0,255,0))#green color
        time.sleep(2)
        set_safty_sw_state(master, sw_state=0)
        time.sleep(1)
        return True
    
    return False


def land_Complete_Or_Not(vehicle, msg):
    global monitor_sim_hit_ground
    monitor_sim_hit_ground = False
    print(f"Received STATUSTEXT: {msg.text}")
    @vehicle.on_message('STATUSTEXT')
    def handle_status_text(self, name, msg):
        print(msg.text)
        if "Land" in msg.text:
         print("land message recived, so drone is comingto last waypoint and will land shortly")
         monitor_sim_hit_ground = True
        if monitor_sim_hit_ground and "SIM Hit ground" in msg.text:
           print("sim hit ground detected, so the drone is on ground")
           pixels.fill(())
           time.sleep(1.5)
    
    while not (monitor_sim_hit_ground):  #come out of the loop only when sim hit ground
        time.sleep(1)

    return monitor_sim_hit_ground 

def set_safty_sw_state(vehicle, sw_state):
    msg = vehicle.message_factory.set_mode_encode(0,mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY, not sw_state)
    vehicle.send_mavlink(msg)

def toggle_safety_switch(vehicle):
    if vehicle.armed:
        vehicle.armed = False
        print("Safety switch off, come back")
        pixels.fill((255, 165, 0))#orange coloor 
    else:
        vehicle.armed = True
        print("Safety switch onn, go near drone nigga")
        pixels.fill((255, 255, 0))#yellow color

def restart_mission(vehicle): 
    cmds = vehicle.commands
    vehicle.commands.next=1
    print("Mission restarted,jai balya")
    time.sleep(1) 
    
