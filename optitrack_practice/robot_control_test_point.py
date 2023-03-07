import math
import socket
import time
import numpy as np
import matplotlib.pyplot as plt
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

positions = {}
rotations = {}
Desired_Position = [1, 0]
# Desired_Position = [2.8, 2.0]

# Record Coordinates--------------------------------
Distance_Error = []
Time = []
Rotation_Error = []
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

t = 0
cnt = 0
try:
    while True:
        if robot_id in positions:
            # X and Y coordinates positions
            X_Position = positions[robot_id][0]
            Y_Position = positions[robot_id][1]
            cnt += 1
            if cnt % 5 == 0:
                print(X_Position,Y_Position)
                print(Distance)
                print(command)
            # Difference in X and Y coordinates
            Difference_X = Desired_Position[0] - X_Position
            Difference_Y = Desired_Position[1] - Y_Position
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

            Distance_Error.append(Distance)
            Time.append(t)
            t = t + 0.05
            Rotation_Error.append(Difference_Rotation)

            v = 400+1200*Distance
            omega = 300*math.degrees(Difference_Rotation)
            u = np.array([v - omega, v + omega])
            u[u > 1500] = 1500
            u[u < -1500] = -1500
            # Send control input to the motors
            command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (u[0], u[0], u[1], u[1])
            s.send(command.encode('utf-8'))

            # Wait for 1 second
            time.sleep(0.05)
            if Distance <= 0.1:
                print(positions[robot_id])
                command = 'CMD_MOTOR#00#00#00#00\n'
                s.send(command.encode('utf-8'))
                # plot the data
                plt.plot(Time, Distance_Error)
                plt.xlabel('Time')
                plt.ylabel('Distance Error')
                plt.show()
                plt.plot(Time, Rotation_Error)
                plt.xlabel('Time')
                plt.ylabel('Rotation Error')
                plt.show()
                break

except KeyboardInterrupt:
    # STOP
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))
    streaming_client.shutdown()

# Close the connection
s.shutdown(2)
s.close()
command = 'CMD_MOTOR#00#00#00#00\n'
s.send(command.encode('utf-8'))
streaming_client.shutdown()





