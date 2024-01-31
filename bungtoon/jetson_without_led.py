from dronekit import connect, VehicleMode, mavutil
from pymavlink import mavutil
import RPi.GPIO as GPIO #uncomment this when before running but not on windows 
#from gpiozero import GPIO #comment this on linux and other retarded windows
import time

the_connection = mavutil.mavlink_connection('tcp:localhost:5762')
# written by retards named pranav and ani
# code will work with blast but end up in error but still will work 
# Wait for the first heartbeat
# This sets the system and component ID of the remote system for the link
the_connection.wait_heartbeat()
print("jai balaya ( connect  %u and is online %u)" %
      (the_connection.target_system, the_connection.target_component))

connection_string = 'tcp:localhost:5763'  # Change to your connection string
vehicle = connect(connection_string)
print("jai balaya is chad, is using 2 devices")

# danger ahead, don't change anything from here 

gpio_pin_jetson = 15 # GPIO gpio_pin_jetson pin for BUTTON 
#num_pixels = 2          # number of NeoPixels
#signal_pin = 18    # NeoPixel out pin
BUTTON_PRESS_TIME = 5   #Button press time
#BLINKING_TIME_SEQUENCES = 25
 
GPIO.setmode(GPIO.BOARD)
GPIO.setup(gpio_pin_jetson, GPIO.IN) #use to setup gpio pin to connect button 





def set_safety_sw_state(vehicle, sw_state):  # this is the function to enable and disable the safety switch
    msg = vehicle.message_factory.set_mode_encode(0, mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY,
                                                  not sw_state)
    vehicle.send_mavlink(msg)


def obese(the_connection, vehicle):
    
    time.sleep(2)
    set_safety_sw_state(vehicle, sw_state=0)
    #pixels.fill((255, 0, 0))  # red to indicate you can go near the drone

    print("Safety switch enabled, go near drone bitch")
    print("press the button for minimum 5 seconds only after finishing all the checks")
    while True:
        GPIO.wait_for_edge(gpio_pin_jetson, GPIO.FALLING)
        #pixels.fill((175, 0, 255))
        print("Button pressed")
        start = time.time()
        time.sleep(0.2)

        while GPIO.input(gpio_pin_jetson) == GPIO.LOW:
            time.sleep(0.01)

        length = time.time() - start
        print(length)
        #pixels.fill((0, 0, 0))

        if length >= BUTTON_PRESS_TIME:
            print("wait for 5 seconds, lil patience bitch")
            time.sleep(5)
            #pixels.fill((0, 0, 0))
            set_safety_sw_state(vehicle, sw_state=1)
            print("Safety switch disabled")
            print("waiting for 5 seconds to change mode to loiter")

            time.sleep(5)

            print("changing to loiter after 2 sec and then taking off will happen")

            time.sleep(2)
            # Change mode to loiter
            vehicle.mode = VehicleMode("LOITER")

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

            vehicle.mode = VehicleMode("AUTO")

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
    


# Try executing the 'obese' function, even if it results in an error
try:
    obese(the_connection, vehicle)
except Exception as e:
    print("Error in 'obese' function:", e)

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
        obese(the_connection, vehicle)

# Register the event handler for STATUSTEXT messages
vehicle.add_message_listener('STATUSTEXT', handle_status_text)

# Keep the program running
try:
    while True:
        pass
except KeyboardInterrupt:
    print("Program terminated.")
