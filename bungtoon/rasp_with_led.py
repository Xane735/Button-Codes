from dronekit import connect, VehicleMode,Command, mavutil
from pymavlink import mavutil
#import RPi.GPIO as GPIO #uncomment this when befor running but not on windows 
from gpiozero import GPIO #comment this on linux and other retardes windows
import neopixel
import board
import time

the_connection = mavutil.mavlink_connection('tcp:localhost:5762')
# written by retards named pranav and ani
# code will work with blast but end up in error but still will work 
# Wait for the first heartbeat
# This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("jai balaya ( connect  %u and is online %u)" %
      (the_connection.target_system, the_connection.target_component))

connection_string = 'tcp:localhost:5763'  # Change to your connection string
vehical = connect(connection_string)
print("jai balaya is chad, is using 2 devices")

#danger ahead, dont change anything from here 

BUTTON_PIN = 27         # GPIO 27 pin for BUTTON 
num_pixels = 2          # number of NeoPixels
pixel_pin = board.D18   # NeoPixel out pin
BUTTON_PRESS_TIME = 5   #Button press time
BLINKING_TIME_SEQUENCES = 25

GPIO.setmode(GPIO.BCM)  # for GPIO numbering, choose BCM
GPIO.setwarnings(False) # to disable warnings.
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  

ORDER = neopixel.RGB
# The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.

pixels = neopixel.NeoPixel(
                            pixel_pin, num_pixels, brightness=1.5, pixel_order=ORDER
                            )
    
pixels.fill((255, 250, 250))
time.sleep(5)
pixels.fill((0,0,0))
    
def set_safty_sw_state(vehicle, sw_state):#this is the function to enable and disable the safty switch
    msg = vehicle.message_factory.set_mode_encode(0,mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY, not sw_state)
    vehicle.send_mavlink(msg)
    
    
def obese(the_connection,vehical,pixels):

    time.sleep(2)
    set_safty_sw_state(vehical, sw_state=0)
    pixels.fill((255, 0, 0))#red to indicate u can go near drone
    
    print("Safety switch enabled, go near drone bitch")
    
    while True:
        GPIO.wait_for_edge(BUTTON_PIN, GPIO.FALLING)
        pixels.fill((175, 0, 255))
        print ("Button pressed")
        start = time.time()
        time.sleep(0.2)
    
        while GPIO.input(BUTTON_PIN) == GPIO.LOW:
            time.sleep(0.01)
            
        length = time.time() - start
        print (length)
        pixels.fill((0, 0, 0))
        
        if length >= BUTTON_PRESS_TIME:
        #    for i in range(BLINKING_TIME_SEQUENCES):    
           #  this FOR loop is for, blinking LED interval of 500 miliseconds 
         #       pixels.fill((175, 255, 0))
          #      time.sleep(0.5) 

           #     pixels.fill((0, 175, 255))
            #    time.sleep(0.5)
        #pixels.fill((0, 0, 255))
            print("wait for 5 seconds, lil patience bitch")
            time.sleep(5)
            pixels.fill((0, 0, 0))
            set_safty_sw_state(vehical, sw_state=1)
            print("Safety switch disabled") 
            print("waiting for 5 seconds to change mode to loiter")
        
            time.sleep(5)
        
            print("changing to loiter after 2 sec and then tecking off will happen")
        
            time.sleep(2)
            # Change mode to loiter
            vehical.mode = VehicleMode("LOITER")
   
            print("Mode changed to LOITER")

            # Wait for 3 seconds
            time.sleep(3)

            # Arm the drone 
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     400, 0, 1, 0, 0, 0, 0, 0, 0)

            msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        
            print(msg)
        
            time.sleep(3)
        
            print("change mode to auto")
        
            vehical.mode = VehicleMode("AUTO")
        
            print("Mode changed to AUTO")
        # Request the current mission items from the Pixhawk
            the_connection.waypoint_request_list_send()

        # Wait for the mission count to be received


            msg = the_connection.recv_match(type='MISSION_COUNT', blocking=True)
            if msg:
                mission_count = msg.count
                print(f"Mission count received: {mission_count}")
    
            time.sleep(1)
            print("start mission")
            print("UR mom is gayyy")
            # Start the mission by sending MAV_CMD_MISSION_START
            start_msg = the_connection.mav.command_long_send(
            0, 0,
            300,
            0,  # Confirmation
            0, 0, 0, 0, 0, 0, 0  # Parameters for the command (not used in this case)   
            )

            # Send the start mission command
            the_connection.mav.send(start_msg)

            test_msg = the_connection.recv_match(type='MISSION_CURRENT', blocking=True)
            print(test_msg)


            print(start_msg)



            print("Auto mission started!")
    

# Variable to indicate whether to monitor SIM_HIT_GROUND or not
monitor_sim_hit_ground = False

# Define a function to handle STATUSTEXT messages
def handle_status_text(vehicle, name, msg):
    global monitor_sim_hit_ground
    
    # Print all status texts for debugging
    print(f"Received STATUSTEXT: {msg.text}")
    
    if "Land" in msg.text:
        print("LAND message received. Starting to monitor SIM_HIT_GROUND.")
        monitor_sim_hit_ground = True

    if monitor_sim_hit_ground and "SIM Hit ground" in msg.text:
        print("JAI JAI BAlAYA")
        print("obese prapti dedicated to u")
        print(" wait 3 seconds")
        time.sleep(3)
        obese(the_connection,vehical,pixels)
# Register the event handler for STATUSTEXT messages
vehical.add_message_listener('STATUSTEXT', handle_status_text)

# Keep the program running
try:
    while True:
        pass
except KeyboardInterrupt:
    print("Program terminated.")

    

