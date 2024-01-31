from pymavlink import mavutil
import time

# Specify the file path for saving the mission
MISSION_FILE_PATH = r'/C:\Users\ani93\OneDrive\Desktop\prec lan\mission.txt'

def connect_to_pixhawk(connection_string='tcp:localhost:5762'):
    """
    Connect to Pixhawk using the specified connection string.
    """
    return mavutil.mavlink_connection(connection_string)

def read_flight_plan(master):
    """
    Read the active flight plan from the Pixhawk using mavlink.
    """
    print("Reading flight plan from Pixhawk")

    # Request the current mission items
    master.waypoint_request_list_send()

    # Wait for the waypoints to be received
    waypoints = []
    start_time = time.time()
    timeout = 10  # seconds

    while True:
        msg = master.recv_match(type='MISSION_ITEM', blocking=False)
        if msg:
            waypoints.append(msg)

        if time.time() - start_time > timeout:
            print("Timeout: No more mission items received.")
            break

    print("Flight plan read successfully")
    return waypoints

def save_flight_plan_to_file(waypoints, filename=MISSION_FILE_PATH):
    """
    Save flight plan to a text file.
    """
    with open(filename, 'w') as file:
        for waypoint in waypoints:
            file.write(f"Waypoint {waypoint.seq}: Lat={waypoint.x}, Lon={waypoint.y}, Alt={waypoint.z}\n")

    print(f"Flight plan saved to {filename}")

# Example usage
if __name__ == "__main__":
    # Connect to Pixhawk
    pixhawk = connect_to_pixhawk()

    try:
        # Read the active flight plan from Pixhawk
        flight_plan = read_flight_plan(pixhawk)

        # Save the flight plan to the specified file path
        save_flight_plan_to_file(flight_plan)

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Close the Pixhawk connection when done
        pixhawk.close()
