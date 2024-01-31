from dronekit import connect, VehicleMode,Command, mavutil
from pymavlink import mavutil
import time

the_connection = mavutil.mavlink_connection('tcp:localhost:5762')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("jai balaya ( connect  %u and is online %u)" %
      (the_connection.target_system, the_connection.target_component))

connection_string = 'tcp:localhost:5763'  # Change to your connection string
vehical = connect(connection_string)
print("jai balaya is chad, is using 2 devices")

def set_safty_sw_state(vehicle, sw_state):
    msg = vehicle.message_factory.set_mode_encode(0,mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY, not sw_state)
    vehicle.send_mavlink(msg)
    
    
def obese(the_connection,vehical):
    time.sleep(2)
    set_safty_sw_state(vehical, sw_state=0)
    print("Safety switch enabled")
    print("Waiting 5 seconds to desable the safty switch")
    time.sleep(5)
    set_safty_sw_state(vehical, sw_state=1)
    print("Safety switch disabled") 
    time.sleep(5)
    print("changing to loiter after 2 sec and then tecking off will happen")
    time.sleep(2)
    # Change mode to loiter
    vehical.mode = VehicleMode("LOITER")
    #while the_connection.mode.name != "LOITER":
    #    print("Changing mode to LOITER...")
    #    pass

    print("Mode changed to LOITER")

    # Wait for 3 seconds
    time.sleep(3)

    # Arm the drone 
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     400, 0, 1, 0, 0, 0, 0, 0, 0)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    time.sleep(2)
# Connect to the Pixhawk autopilot
# Replace 'udp:127.0.0.1:14550' with the appropriate connection string
# For example: 'udp:192.168.1.2:14550' for a UDP connection
# Or 'com14' for a serial connection on COM port 14 (Windows)
    time.sleep(2)
    print("change mode to auto")
    vehical.mode = VehicleMode("AUTO")
    print("Mode changed to AUTO")
# Request the current mission items from the Pixhawk
    the_connection.waypoint_request_list_send()

# Wait for the mission count to be received


    msg = the_connection.recv_match(type='MISSION_COUNT', blocking=True)
    if msg:
        mission_count = msg.count
        print(f"Mission count received: {mission_count}")
    
    time.sleep(1)
    print("start mission")

# Start the mission by sending MAV_CMD_MISSION_START
    start_msg = the_connection.mav.command_long_send(
    0, 0,
    300,
    0,  # Confirmation
    0, 0, 0, 0, 0, 0, 0  # Parameters for the command (not used in this case)   
    )

# Send the start mission command
    the_connection.mav.send(start_msg)

    test_msg = the_connection.recv_match(type='MISSION_CURRENT', blocking=True)
    print(test_msg)


    print(start_msg)
#subbu.takeoff(the_connection)
#time.sleep(10)
#subbu.changetoauto(the_connection)


    print("Auto mission started!")
    #the_connection.armed = True
    #time.sleep(1)
    #Wait for the arming process to complete
    #while not the_connection.armed:
    #   print("Waiting for arming...")
     #  time.sleep(1)

    #print("Drone armed!")
    #time.sleep(2)
    #time.sleep(2)
    #print("change mode to auto")
    # Change mode to auto
    #vehical.mode = VehicleMode("AUTO")
    #while the_connection.mode.name != "AUTO":
    #    print("Changing mode to AUTO...")
    #    pass

    

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
        print("obese prapti dedicated to u")
        print(" wait 3 seconds")
        time.sleep(3)
        obese(the_connection,vehical)
# Register the event handler for STATUSTEXT messages
vehical.add_message_listener('STATUSTEXT', handle_status_text)

# Keep the program running
try:
    while True:
        pass
except KeyboardInterrupt:
    print("Program terminated.")

    

