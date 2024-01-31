import time
from xml.etree.ElementTree import PI
#import RPi.GPIO as GPIO
import neopixel
import board
from dronekit import connect, VehicleMode, mavutil, Command
import os
from cmd_msg_dronekit import *


#Set up option parsing to get connection string
import argparse 


def wheel(pos, ORDER):
    # Input a value 0 to 255 to get a color value.
    # The colours are a transition r - g - b - back to r.
    if pos < 0 or pos > 255:
        r = g = b = 0
    elif pos < 85:
        r = int(pos * 3)
        g = int(255 - pos * 3)
        b = 0
    elif pos < 170:
        pos -= 85
        r = int(255 - pos * 3)
        g = 0
        b = int(pos * 3)
    else:
        pos -= 170
        r = 0
        g = int(pos * 3)
        b = int(255 - pos * 3)
    return (b, g, r) if ORDER in (neopixel.RGB, neopixel.GRB) else (b, g, r, 0)

def rainbow_cycle(wait, num_pixels, pixels, ORDER):
    for j in range(255):
        for i in range(num_pixels):
            pixel_index = (i * 255 // num_pixels) + j
            pixels[i] = wheel(pixel_index & 255, ORDER)
        pixels.show()
        time.sleep(wait)

def btn_main():
            parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
            parser.add_argument('--connect', 
                            help="vehicle connection target string. If not specified, SITL automatically started and used.")
            args = parser.parse_args()

            connection_string = args.connect
            sitl = None
            #Start SITL if no connection string specified
            if not connection_string:
                connection_string = 'tcp:127.0.0.1:5760'
                # import dronekit_sitl
                # sitl = dronekit_sitl.start_default()
                # connection_string = sitl.connection_string()


                # Connect to the Vehicle. 
                #   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
            # if os.path.exists('mission_upload_state.txt'):
                with open('/home/pi/button-mission-auto/mission_upload_state.txt', 'r') as f:
                    state = f.read()
                    if state == 'complete':
                        print("\nConnecting to vehicle on:", connection_string)
                        vehicle = connect(connection_string)
                        #User Params

                        BUTTON_PIN = 27         #declare the GPIO 23 pin for the BUTTON input
                        num_pixels = 2          #declare the number of NeoPixels
                        pixel_pin = board.D18   #declare the NeoPixel out put pin
                        BUTTON_PRESS_TIME = 5   #Button press state time
                        BLINKING_TIME_SEQUENCES = 25

                        #Warning: Do not change anything from here

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
                        while True:
                            GPIO.wait_for_edge(BUTTON_PIN, GPIO.FALLING)
                            pixels.fill((175, 0, 255))
                            # print ("Button press detected")
                            start = time.time()
                            time.sleep(0.2)

                            while GPIO.input(BUTTON_PIN) == GPIO.LOW:
                                time.sleep(0.01)
                            length = time.time() - start
                            print (length)
                            pixels.fill((0, 0, 0))
                            
                            if vehicle.armed:
                                    print("Vehicle is already armed. Ignoring the button press.")
                                    pixels.fill((0, 255, 0))
                                    time.sleep(5)
                                    pixels.fill((0, 0, 0))
                                    continue

                                # Check if the vehicle is on the ground
                            if vehicle.location.global_relative_frame.alt > 1.0:
                                    print("Vehicle is not on the ground. Ignoring the button press.")
                                    pixels.fill((0, 255, 0))
                                    time.sleep(5)
                                    pixels.fill((0, 0, 0))
                                    continue
                                    
                            if length >= BUTTON_PRESS_TIME:    
                            #if button pressed time is greater than BUTTON_PRESS_TIME:value then next sequence will begin
                                # print ("Arming sequences started")
                                #Change Mode to AUTO
                                # Check if the vehicle is already armed

                                for i in range(BLINKING_TIME_SEQUENCES):    
                                    #  this FOR loop is for, blinking LED interval of 500 miliseconds 
                                        pixels.fill((175, 255, 0))
                                        time.sleep(0.5) 

                                        pixels.fill((0, 175, 255))
                                        time.sleep(0.5)

                                pixels.fill((0, 0, 255))
                                time.sleep(5)
                                pixels.fill((0, 0, 0))
                                set_safty_sw_state(vehicle, sw_state=1)
                                time.sleep(1)
                                # print ("Basic pre-arm checks")
                                # Don't try to arm until autopilot is ready
                                # while not vehicle.is_armable:
                                #     print (" Waiting for vehicle to initialise...")
                                #     time.sleep(1) 

                                # Set the vehicle into QLOITER mode
                                vehicle.mode = VehicleMode("QLOITER")
                                time.sleep(2)
                                
                                time.sleep(0.5)

                                vehicle.armed   = True
                                # print ("Arming motors")
                                time.sleep(3)

                                while not vehicle.armed:
                                   pixels.fill((0, 255, 0)) #LED color RED
                                     
                                # Set the vehicle into QLOITER mode
                                vehicle.mode = VehicleMode("AUTO")
                                time.sleep(1)
                                
                               
                                with open('/home/pi/button-mission-auto/mission_upload_state.txt', 'w') as f:
                                            f.write('not complete')
                                landComp_safetySwitchOFF(vehicle)
                                time.sleep(1)

                                clear_mission(vehicle) #clear mission after Land
                                time.sleep(1)
                                vehicle.commands.next=1 #set mission wp to one

                                rainbow_cycle(0.001, num_pixels, pixels, ORDER)
                                # print("rainbow") 
                                vehicle.close()

                                break 
                                #the break is for, this code will not run again untill reboot
                                GPIO.cleanup()

                            else:
                                pixels.fill((0, 0, 0))
    # btn_main()