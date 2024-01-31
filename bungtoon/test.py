import time
from dronekit import connect, VehicleMode, Command, mavutil
# Example usage with a connection string
connection_string = 'tcp:localhost:5762'  # Change to your connection string



def monitor_and_continue(connection_string):
    try:
        # Connect to the drone (update the connection string accordingly)
        the_connection = connect(connection_string)
        print("Connected")

        # Monitor for LAND message
        land_message_received = False
        while not land_message_received:
            print("Waiting for LAND message...")
            time.sleep(1)

            # Check if LAND message is received (replace 'LAND' with the actual message name)
            if 'LAND' in the_connection.messages:
                land_message_received = True

        print("LAND message received. Monitoring SIM_GROUND_HIT for 5 seconds...")

        # Monitor SIM_GROUND_HIT for 5 seconds
        end_time = time.time() + 5
        while time.time() < end_time:
            # Check if SIM_GROUND_HIT message is received (replace 'SIM_GROUND_HIT' with the actual message name)
            if 'SIM_GROUND_HIT' in the_connection.messages:
                print("SIM_GROUND_HIT message received.")
                break

            time.sleep(1)

        # Continue with the original sequence

        # Change mode to LOITER
        the_connection.mode = VehicleMode("LOITER")
        while the_connection.mode.name != "LOITER":
            print("Changing mode to LOITER...")
            time.sleep(1)

        print("Mode changed to LOITER")

        # Wait for 3 seconds
        time.sleep(3)

        # Arm the drone
        the_connection.armed = True

        # Wait for the arming process to complete
        while not the_connection.armed:
            print("Waiting for arming...")
            time.sleep(1)

        print("Drone armed!")

        # Change mode to AUTO
        the_connection.mode = VehicleMode("AUTO")
        while the_connection.mode.name != "AUTO":
            print("Changing mode to AUTO...")
            time.sleep(1)

        print("Mode changed to AUTO")

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Close the connection in a finally block to ensure it's closed even if an exception occurs
        the_connection.close()


monitor_and_continue(connection_string)
print("connection ")