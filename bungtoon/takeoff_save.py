from dronekit import connect, VehicleMode
from pymavlink import mavutil

connection_string = 'tcp:localhost:5762'  # Change to your connection string
the_connection = connect(connection_string)
print("connected")

# Variable to indicate whether to monitor SIM_HIT_GROUND or not
monitor_takeoff = True  # Set to True to start monitoring

# Define a function to handle STATUSTEXT messages
def handle_status_text(vehicle, name, msg):
    global monitor_takeoff
    
    # Print all status texts for debugging
    print(f"Received STATUSTEXT: {msg.text}")
    
    # Check if monitor_takeoff is True and "Takeoff" is in the received text (case-insensitive)
    if monitor_takeoff and "takeoff" in msg.text.lower():
        print("JAI JAI BAlAYA")

# Register the event handler for STATUSTEXT messages
the_connection.add_message_listener('STATUSTEXT', handle_status_text)

# Keep the program running
try:
    while True:
        # You can toggle monitor_takeoff based on your logic here
        pass
except KeyboardInterrupt:
    print("Program terminated.")
