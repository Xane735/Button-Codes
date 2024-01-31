import Jetson.GPIO as GPIO

# Set the GPIO pin you're using
gpio_pin = 12

# Setup GPIO mode and wait for a falling edge
GPIO.setmode(GPIO.BOARD)
GPIO.setup(gpio_pin, GPIO.IN)

try:
    print("Waiting for a falling edge on GPIO {}...".format(gpio_pin))
    GPIO.wait_for_edge(gpio_pin, GPIO.FALLING)
    print("Falling edge detected on GPIO {}!".format(gpio_pin))

except KeyboardInterrupt:
    print("Exiting...")

finally:
    # Cleanup GPIO settings on exit
    GPIO.cleanup()
