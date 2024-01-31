import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
input_pin = 15
GPIO.setup(input_pin, GPIO.IN)

while True:
    x=GPIO.input(input_pin)
    print(x)