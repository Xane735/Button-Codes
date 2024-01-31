import RPi.GPIO as GPIO
import time

# Set the GPIO pin you're using
gpio_pin = 12

# Setup GPIO mode and wait for a falling edge
GPIO.setmode(GPIO.BOARD)
GPIO.setup(gpio_pin, GPIO.IN)

try:
    print("Waiting for a falling edge on GPIO {}...".format(gpio_pin))
    GPIO.wait_for_edge(gpio_pin, GPIO.FALLING)
    print("Falling edge detected on GPIO {}!".format(gpio_pin))

    # Wait for the pin to stay low for 5 seconds
    start_time = time.time()
    while GPIO.input(gpio_pin) == GPIO.LOW:
        time.sleep(0.1)
        if time.time() - start_time >= 5:
            print("Hello! GPIO {} stayed low for 5 seconds.".format(gpio_pin))
            break

except KeyboardInterrupt:
    print("Exiting...")

finally:
    # Cleanup GPIO settings on exit
    GPIO.cleanup()
