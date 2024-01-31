import time
from pymavlink import mavutil

# Replace 'udp:127.0.0.1:14550' with your connection string (e.g., serial port, UDP, etc.)
connection_string = 'udp:127.0.0.1:14550'

# Define the file path to save waypoints
file_path = 'path/to/waypoints.txt'

# Create a MAVLink connection
vehicle = mavutil.mavlink_connection(connection_string)

try:
    # Request waypoint information
    vehicle.mav.mission_request_list_send(
        vehicle.target_system,   # Target system ID
        vehicle.target_component,  # Target component ID
        0  # Sequence number of the first mission item
    )

    # Wait for the mission count message
    msg_count = vehicle.recv_match(type='MISSION_COUNT', blocking=True)
    waypoint_count = msg_count.count

    if waypoint_count == 0:
        print("No waypoints available.")
    else:
        # Open a text file to save waypoints
        with open(file_path, 'w') as file:
            # Loop through each waypoint
            for seq in range(waypoint_count):
                # Request waypoint data
                vehicle.mav.mission_request_send(
                    vehicle.target_system,    # Target system ID
                    vehicle.target_component,  # Target component ID
                    seq  # Sequence number of the requested mission item
                )

                # Wait for the waypoint message
                msg_waypoint = vehicle.recv_match(type='MISSION_ITEM', blocking=True)

                # Extract waypoint information
                latitude = msg_waypoint.x / 1e7  # Latitude in degrees
                longitude = msg_waypoint.y / 1e7  # Longitude in degrees
                altitude = msg_waypoint.z  # Altitude in meters

                # Write waypoint to the file
                file.write(f"Waypoint {seq + 1}: Lat={latitude}, Lon={longitude}, Alt={altitude}\n")

                time.sleep(1)  # Sleep for a second before the next iteration

    print(f"Waypoints saved to '{file_path}'.")

except KeyboardInterrupt:
    print("Reading waypoints interrupted by user.")

finally:
    # Close the connection when done
    vehicle.close()
