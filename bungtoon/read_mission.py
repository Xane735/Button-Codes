from dronekit import connect, VehicleMode

def connect_to_pixhawk(connection_string='tcp:localhost:5762'):
    """
    Connect to Pixhawk using the specified connection string.
    """
    return connect(connection_string, wait_ready=True)

def read_mission(vehicle):
    """
    Read the mission from the Pixhawk using dronekit.
    """
    print("Reading mission from Pixhawk")

    # Get the mission items from the vehicle
    cmds = vehicle.commands

    # Get all mission items as a list
    missionlist = []
    for cmd in cmds:
        missionlist.append(cmd)

    print("Mission read successfully")
    return missionlist

# Example usage
if __name__ == "__main__":
    # Connect to Pixhawk
    pixhawk = connect_to_pixhawk()

    try:
        # Read the mission from Pixhawk
        mission_items = read_mission(pixhawk)

        # Display or process the mission items as needed
        for item in mission_items:
            print(f"Waypoint {item.seq}: Lat={item.x}, Lon={item.y}, Alt={item.z}")

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Close the Pixhawk connection when done
        pixhawk.close()
