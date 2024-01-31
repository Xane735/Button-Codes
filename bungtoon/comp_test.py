from cmd_msg_dronekit import *
import time

def test_neopixel_buzzer():
    # Test NeoPixel
    pixels.fill((255, 0, 0))  # Red color
    time.sleep(3)
    pixels.fill((0, 255, 0))  # Green color
    time.sleep(3)
    pixels.fill((0, 0, 255))  # Blue color
    time.sleep(3)
    pixels.fill((0, 0, 0))    # Turn off NeoPixel
    return

test_neopixel_buzzer()