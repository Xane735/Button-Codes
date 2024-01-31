from dronekit import connect, VehicleMode
from pymavlink import mavutil

connection_string = 'tcp:localhost:5762'  # Change to your connection string
the_connection = connect(connection_string)
print("connected")

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

# Register the event handler for STATUSTEXT messages
the_connection.add_message_listener('STATUSTEXT', handle_status_text)

# Keep the program running
try:
    while True:
        pass
except KeyboardInterrupt:
    print("Program terminated.")
