import time
from dronekit import connect, VehicleMode

# Connect to the drone (update the connection string accordingly)
connection_string = 'tcp:localhost:5762'  # Change to your connection string
the_connection = connect(connection_string)
print("Connected")

# Change mode to loiter
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

# Change mode to auto
the_connection.mode = VehicleMode("AUTO")
while the_connection.mode.name != "AUTO":
    print("Changing mode to AUTO...")
    time.sleep(1)

print("Mode changed to AUTO")

# Close the connection
the_connection.close()
