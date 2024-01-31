from dronekit import connect, VehicleMode, mavutil
from pymavlink import mavutil
from rpi_ws281x import PixelStrip, Color
#import Jetson.GPIO as GPIO #uncomment this when before running on jetson
#import RPi.GPIO as GPIO #uncomment this when before running but not on windows 
from gpiozero import GPIO #comment this on linux and other retarded windows
#from rpi_ws281x import Adafruit_NeoPixel, Color #uncomment this on jetson
#from neopixel import Adafruit_NeoPixel, Color#uncomment this on jetson
#import neopixel
#import board
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

gpio_pin_jetson = 27        # GPIO 27 pin for BUTTON 
num_pixels = 2          # number of NeoPixels
signal_pin = 18    # NeoPixel out pin
BUTTON_PRESS_TIME = 5   #Button press time
BLINKING_TIME_SEQUENCES = 25

GPIO.setmode(GPIO.BCM)  # for GPIO numbering, choose BCM
GPIO.setwarnings(False) # to disable warnings.
GPIO.setup(gpio_pin_jetson, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
# LED strip configuration:
LED_COUNT = 8        # Number of LED pixels.
LED_PIN = 18          # GPIO pin connected to the pixels (18 uses PWM!).
LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800kHz).
LED_DMA = 10          # DMA channel to use for generating signal (try 10).
LED_BRIGHTNESS = 255  # Set to 0 for darkest and 255 for brightest.
LED_INVERT = False    # True to invert the signal (when using NPN transistor level shift).
LED_CHANNEL = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53


#ORDER = neopixel.RGB
# The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.

#pixels = neopixel.NeoPixel(
#    signal_pin, num_pixels, brightness=1.5, pixel_order=ORDER
#)
strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
strip.begin()

pixels.fill((255, 250, 250))
time.sleep(5)
pixels.fill((0, 0, 0))


def set_safety_sw_state(vehicle, sw_state):  # this is the function to enable and disable the safety switch
    msg = vehicle.message_factory.set_mode_encode(0, mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY,
                                                  not sw_state)
    vehicle.send_mavlink(msg)


def obese(the_connection, vehicle, pixels):
    
    time.sleep(2)
    set_safety_sw_state(vehicle, sw_state=0)
    pixels.fill((255, 0, 0))  # red to indicate you can go near the drone

    print("Safety switch enabled, go near drone bitch")

    while True:
        GPIO.wait_for_edge(gpio_pin_jetson, GPIO.FALLING)
        pixels.fill((175, 0, 255))
        print("Button pressed")
        start = time.time()
        time.sleep(0.2)

        while GPIO.input(gpio_pin_jetson) == GPIO.LOW:
            time.sleep(0.01)

        length = time.time() - start
        print(length)
        pixels.fill((0, 0, 0))

        if length >= BUTTON_PRESS_TIME:
            print("wait for 5 seconds, lil patience bitch")
            time.sleep(5)
            pixels.fill((0, 0, 0))
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
    obese(the_connection, vehicle, pixels)
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
        obese(the_connection, vehicle, pixels)

# Register the event handler for STATUSTEXT messages
vehicle.add_message_listener('STATUSTEXT', handle_status_text)

# Keep the program running
try:
    while True:
        pass
except KeyboardInterrupt:
    print("Program terminated.")
