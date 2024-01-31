import time
from xml.etree.ElementTree import PI
#import RPi.GPIO as GPIO  #retard linux liberary
from gpiozero import GPIO #for windows, so u cannot see yellow line and annoie u while correcting the code
import neopixel
import board
from dronekit import connect, VehicleMode, mavutil, Command
from buutton_cmd import *

set_connection ='tcp:localhost:5762'# Change to your connection string
connection_string = set_connection  
the_connection = connect(connection_string)
print("connected, button on fire")
import argparse 

#main function which will handel the status texts of the drone
def button(vehicle, name, msg):
    parser = argparse.ArgumentParser(description='status of the vehical')
    parser.add_argument('--connect', 
                            help="if connectin is set to none, then it will connect to sitl")
    args = parser.parse_args()

    connection_string = args.connect
    sitl = None
            
    vehicle = connect(connection_string)
                       

    BUTTON_PIN = 27         #button pin GPIO 27
    num_pixels = 2          #number of led on the strip
    pixel_pin = board.D18   #the signal pin of the led
    BUTTON_PRESS_TIME = 5   #butto press time
    BLINKING_TIME_SEQUENCES = 25

     #dange ahead, dont change anything nigga        

    GPIO.setmode(GPIO.BCM)  
    GPIO.setwarnings(False) 
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  

    ORDER = neopixel.RGB
                        

    pixels = neopixel.NeoPixel(
                            pixel_pin, num_pixels, brightness=1.5, pixel_order=ORDER
                        )
    pixels.fill((255, 250, 250))#order may be RGB or GRB depending on the company of led, so see properly
    time.sleep(5)
    pixels.fill((0,0,0))
 
    print(f"Received STATUSTEXT: {msg.text}")
    GPIO.wait_for_edge(BUTTON_PIN, GPIO.FALLING)
    pixels.fill((175, 0, 255))
    print ("Button press detected")
    start = time.time()
    time.sleep(0.2)

    while GPIO.input(BUTTON_PIN) == GPIO.LOW:
                time.sleep(0.01)
                length = time.time() - start
                print (length)
                pixels.fill((0, 0, 0))
                            
                if vehicle.armed:
                    print("vehicle is already armed so ignore button ")
                    pixels.fill((0, 255, 0))
                    time.sleep(5)
                    pixels.fill((0, 0, 0))
                    continue
                               
                if vehicle.location.global_relative_frame.alt > 1.0:
                    print("vehicle is not on the ground so ignore button.")
                    pixels.fill((0, 255, 0))
                    time.sleep(5)
                    pixels.fill((0, 0, 0))
                    continue
                                    
                if length >= BUTTON_PRESS_TIME:    
                            
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

                                # sets the drone to loiter 
                                        vehicle.mode = VehicleMode("LOITER")
                                        time.sleep(2)
                                
                                        time.sleep(0.5)

                                        vehicle.armed   = True
                                        print ("Arming motors")
                                        time.sleep(3)

                                        while not vehicle.armed:
                                            pixels.fill((255, 0, 0)) #LED color red
                                     
                                # sets the drone to auto
                                        vehicle.mode = VehicleMode("AUTO")
                                        time.sleep(1)
                                        pixels.fill((255,255,0))
                                        msg = vehicle.message_factory.command_long_encode(
                                                 0, 0,
                                         mavutil.mavlink.MAV_CMD_MISSION_START,
                                                 0,  # Confirmation
                                                 0,  # Params 1-6 not used
                                                 0, 0, 0, 0, 0, 0
                                                                                          )
                                        vehicle.send_mavlink(msg)
                               
                                        landComp_safetySwitchOFF(vehicle)
                                        time.sleep(1)
                                        

                                        vehicle.close()

                                        break 
                                # until reboted again the program will not run again, better give in servive resart 
                                #always to restart the program
                                        GPIO.cleanup()

                            else:
                                pixels.fill((0, 0, 0))

# Register the event handler for STATUSTEXT messages
the_connection.add_message_listener('STATUSTEXT', button)    
print("UR MOM is GAYYYYY")
try:
    while True:
        pass
except KeyboardInterrupt:
    print("Program terminated.")