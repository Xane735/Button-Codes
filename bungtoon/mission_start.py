from pymavlink import mavutil
import time
#import subbu

# Connect to the Pixhawk autopilot
# Replace 'udp:127.0.0.1:14550' with the appropriate connection string
# For example: 'udp:192.168.1.2:14550' for a UDP connection
# Or 'com14' for a serial connection on COM port 14 (Windows)
master = mavutil.mavlink_connection('tcp:localhost:5762', baud=57600)

# Wait for the heartbeat from the Pixhawk
master.wait_heartbeat()

# Request the current mission items from the Pixhawk
master.waypoint_request_list_send()

# Wait for the mission count to be received
while True:

   # subbu.changetoguided(master) # Add cmd for changing to guided mode
    time.sleep(5)
    #subbu.arm(master)   # Add arming cmd
    time.sleep(5)
    #subbu.changetoauto(master) #add Chaging to auto mode
    time.sleep(2)

    msg = master.recv_match(type='MISSION_COUNT', blocking=True)
    if msg:
        mission_count = msg.count
        print(f"Mission count received: {mission_count}")
        break
    time.sleep(0.1)

# Start the mission by sending MAV_CMD_MISSION_START
start_msg = master.mav.command_long_send(
    0, 0,
    300,
    0,  # Confirmation
    0, 0, 0, 0, 0, 0, 0  # Parameters for the command (not used in this case)
)

# Send the start mission command
master.mav.send(start_msg)

test_msg = master.recv_match(type='MISSION_CURRENT', blocking=True)
print(test_msg)


print(start_msg)
#subbu.takeoff(master)
time.sleep(10)
#subbu.changetoauto(master)


print("Auto mission started!")