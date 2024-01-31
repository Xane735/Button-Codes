from dronekit import connect, VehicleMode,Command, mavutil
from pymavlink import mavutil
import time
# Start a connection listening to a UDP port
#the_connection = mavutil.mavlink_connection('tcp:localhost:5762')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the linkm
#the_connection.wait_heartbeat()
#print("Heartbeat from system (system %u component %u)" %
#      (the_connection.target_system, the_connection.target_component))

connection_string = 'tcp:localhost:5763'  # Change to your connection string
vehicle = connect(connection_string)

'''def enable_safety_switch(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     400, 0, 1, 0, 0, 0, 0, 0, 0)
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     400, 0, 1, 0, 0, 0, 0, 0, 0)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

def disable_safety_switch(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     400, 0, 0, 0, 0, 0, 0, 0, 0)
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)'''
    
def set_safty_sw_state(vehicle, sw_state):
    msg = vehicle.message_factory.set_mode_encode(0,mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY, not sw_state)
    vehicle.send_mavlink(msg)
# Enable the safety switch
'''enable_safety_switch(the_connection)
print("Safety switch enabled")
print("Waiting 5 seconds")
time.sleep(5)

# Disable the safety switch
disable_safety_switch(the_connection)
print("Safety switch disabled")
print("Waiting 5 seconds")
time.sleep(5)'''
while True:
    set_safty_sw_state(vehicle, sw_state=0)
    print("Safety switch enabled")
    time.sleep(5)
    print("waiting for 5 seconds to desable the safty switch")
    set_safty_sw_state(vehicle, sw_state=1)
    print("Safety switch disabled") 
    time.sleep(5)
    pass
