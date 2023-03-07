import math
import socket
import time
import numpy as np
import matplotlib.pyplot as plt
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

positions = {}
rotations = {}

# Record Coordinates--------------------------------
Coordinate_Desired_X = []
Coordinate_Desired_Y = []
Coordinate_X = []
Coordinate_Y = []
# End here-------------------------


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz


# IP addresses
IP_ADDRESS = "192.168.0.204"
clientAddress = "192.168.0.23"
optitrackServerAddress = "192.168.0.4"
robot_id = 204

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')

# This will create a new NatNet client
streaming_client = NatNetClient()
streaming_client.set_client_address(clientAddress)
streaming_client.set_server_address(optitrackServerAddress)
streaming_client.set_use_multicast(True)
# Configure the streaming client to call our rigid body handler on the emulator to send data out.
streaming_client.rigid_body_listener = receive_rigid_body_frame

# Start up the streaming client now that the callbacks are set up.
# This will run perpetually, and operate on a separate thread.
is_running = streaming_client.run()

while True:
    if robot_id in positions:
        Initial_Position_X = positions[robot_id][0]
        Initial_Position_Y = positions[robot_id][1]
        break

t = 0

try:
    while True:
        if robot_id in positions:
            if t <= 25:
                # X and Y coordinates positions
                X_Position = positions[robot_id][0]
                Y_Position = positions[robot_id][1]
                Desired_Position_X = Initial_Position_X + t/10
                Desired_Position_Y = Initial_Position_Y + ((t/10)**2)*((t/10)-3)
                # Difference in X and Y coordinates
                Difference_X = Desired_Position_X - X_Position
                Difference_Y = Desired_Position_Y - Y_Position
                # Calculate the Distance
                Distance = np.sqrt(Difference_X ** 2 + Difference_Y ** 2)
                # Rotation
                Rotation = math.radians(rotations[robot_id])
                # Desired Rotation
                Alpha = np.arctan2(Difference_Y, Difference_X)
                # Difference in desired rotation and real rotation'
                Difference_Rotation = Alpha - Rotation
                Difference_Rotation = np.arctan2(np.sin(Difference_Rotation), np.cos(Difference_Rotation))
                # Linear speed and Angular speed transformation

                Coordinate_X.append(X_Position)
                Coordinate_Y.append(Y_Position)
                Coordinate_Desired_X.append(Desired_Position_X)
                Coordinate_Desired_Y.append(Desired_Position_Y)

                v = 500+1500*Distance
                omega = 400 * math.degrees(Difference_Rotation)

                u = np.array([v - omega, v + omega])
                u[u > 1500] = 1500
                u[u < -1500] = -1500
                # Send control input to the motors
                command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (u[0], u[0], u[1], u[1])
                s.send(command.encode('utf-8'))
                t = t + 0.1
                time.sleep(0.1)
            else:
                print(positions[robot_id])
                command = 'CMD_MOTOR#00#00#00#00\n'
                s.send(command.encode('utf-8'))
                # plot the desired coordinates
                ax = plt.gca()
                ax.set_aspect('equal', adjustable='box')
                plt.plot(Coordinate_Desired_X, Coordinate_Desired_Y)
                # plot the real coordinates
                plt.plot(Coordinate_X, Coordinate_Y)
                plt.xlabel('X')
                plt.ylabel('Y')
                plt.grid()
                plt.show()
                break


except KeyboardInterrupt:
    # STOP
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))
    streaming_client.shutdown()

# Terminate the Robot
command = 'CMD_MOTOR#00#00#00#00\n'
s.send(command.encode('utf-8'))
streaming_client.shutdown()
# Close the connection
s.shutdown(2)
s.close()
