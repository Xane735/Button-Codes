import time
from dronekit import connect, VehicleMode
# Connect to the the_connection (update the connection string accordingly)
connection_string = 'tcp:localhost:5762'  # Change to your connection string
the_connection = connect(connection_string)
print("connected")


# Print the_connection mode and armed status
print("the_connection mode:", the_connection.mode)
print("the_connection armed:", the_connection.armed)

# Arm the drone after a 5-second delay
time.sleep(5)
the_connection.armed = True

# Wait for the arming process to complete
while not the_connection.armed:
    print("Waiting for arming...")
    print("the_connection armed:", the_connection.armed)
    time.sleep(1)

print("Drone armed!")

# Disarm the drone after another 5-second delay
time.sleep(5)
the_connection.armed = False

# Wait for the disarming process to complete
while the_connection.armed:
    print("Waiting for disarming...")
    print("the_connection armed:", the_connection.armed)
    time.sleep(1)

print("Drone disarmed!")

# Close the connection
the_connection.close()
