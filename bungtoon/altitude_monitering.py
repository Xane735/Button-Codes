from dronekit import connect, VehicleMode
from pymavlink import mavutil
import threading

connection_string = 'tcp:localhost:5762'  # Change to your connection string
vehicle = connect(connection_string)
print("connected")

# Global variable to indicate whether altitude monitoring should start
start_altitude_monitoring = False

# Define a function to handle STATUSTEXT messages
def handle_status_text(vehicle, name, msg):
    global start_altitude_monitoring

    print(f"Received STATUSTEXT: {msg.text}")

    # Check if the status indicates "LAND"
    if "Land" in msg.text:
        print("Received LAND status. Starting altitude monitoring.")
        start_altitude_monitoring = True

# Register the event handler for STATUSTEXT messages
vehicle.add_message_listener('STATUSTEXT', handle_status_text)

# Define a function to monitor altitude
def altitude_monitor():
    while True:
        # Check if altitude monitoring should start
        if start_altitude_monitoring:
            gps = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            altitude_feet = gps.relative_alt / 1000 * 3.28084  # Altit

            print(f"Altitude: {altitude_feet} feet")

            # Check if altitude is below 5 feet
            if altitude_feet < 5:
                print("Hello")

        # Break the loop if needed or continue monitoring

# Create and start the altitude monitor thread
altitude_monitor_thread = threading.Thread(target=altitude_monitor)
altitude_monitor_thread.start()

# Keep the program running
try:
    while True:
        pass
except KeyboardInterrupt:
    # Close the connection to the vehicle
    vehicle.close()
    print("Program terminated.")
