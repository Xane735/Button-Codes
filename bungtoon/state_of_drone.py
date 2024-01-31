from dronekit import connect, VehicleMode
from pymavlink import mavutil

connection_string = 'tcp:localhost:5762'  # Change to your connection string
the_connection = connect(connection_string)
print("connected")

# Define a function to handle STATUSTEXT messages
def handle_status_text(vehicle, name, msg):
    print(f"Received STATUSTEXT: {msg.text}")

# Register the event handler for STATUSTEXT messages
the_connection.add_message_listener('STATUSTEXT', handle_status_text)

# Keep the program running
try:
    while True:
        pass
except KeyboardInterrupt:
    print("Program terminated.")
