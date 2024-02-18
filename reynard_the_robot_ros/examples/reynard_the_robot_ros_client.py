#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import reynard_the_robot_ros_msgs.msg as reynard_msg
import reynard_the_robot_ros_msgs.srv as reynard_srv
import numpy as np

# Initialize a rospy node
rospy.init_node('reynard_the_robot_client', anonymous=True)

# Create service proxies, publishers, and subscribers
teleport_srv = rospy.ServiceProxy('/reynard/teleport', reynard_srv.Teleport)
say_pub = rospy.Publisher('/reynard/say', String, queue_size=10)
new_message_sub = rospy.Subscriber('/reynard/new_message', String, lambda data: print(f"New message: {data.data}"))
drive_robot_srv = rospy.ServiceProxy('/reynard/drive_robot', reynard_srv.Drive)
drive_arm_srv = rospy.ServiceProxy('/reynard/drive_arm', reynard_srv.Drive)
set_arm_position_srv = rospy.ServiceProxy('/reynard/set_arm_position', reynard_srv.SetPosition)
set_color_srv = rospy.ServiceProxy('/reynard/set_color', reynard_srv.SetColor)
get_color_srv = rospy.ServiceProxy('/reynard/get_color', reynard_srv.GetColor)

new_state_print_count = [0]

# Define a callback function for the new state subscriber
def new_state(data):
    if new_state_print_count[0] < 2:
        print(f"New state: {data}")
    new_state_print_count[0] += 1
new_state_pub = rospy.Subscriber('/reynard/state', reynard_msg.ReynardState, new_state)

teleport_srv.wait_for_service()

# Teleport the robot
teleport_req = reynard_srv.TeleportRequest(0.1, -0.2)
teleport_res = teleport_srv(teleport_req)
assert teleport_res.success, f"Teleport error: {teleport_res.status_message}"

# Drive the robot with no timeout
drive_res = drive_robot_srv(reynard_srv.DriveRequest((0.5,-0.2),-1,False))
assert drive_res.success, f"Drive error: {drive_res.status_message}"

# Wait for one second
rospy.sleep(1)

# Stop the robot
drive_res = drive_robot_srv(reynard_srv.DriveRequest((0,0),-1,False))
assert drive_res.success, f"Drive error: {drive_res.status_message}"

# Set the arm position
pos_res = set_arm_position_srv(reynard_srv.SetPositionRequest(np.deg2rad((100,-30,-70))))
assert pos_res.success, f"Set arm position error: {pos_res.status_message}"

# Drive the arm with no timeout
drive_res = drive_arm_srv(reynard_srv.DriveRequest(np.deg2rad((10, -30, -15)), 1.5, True))
assert drive_res.success, f"Drive error: {drive_res.status_message}"

# Set the color to red
color_res = set_color_srv(reynard_srv.SetColorRequest(255,0,0))
assert color_res.success, f"Set color error: {color_res.status_message}"

# Read the color
color_res = get_color_srv()
assert color_res.success, f"Get color error: {color_res.status_message}"
print(f"Color: {color_res.r}, {color_res.g}, {color_res.b}")

rospy.sleep(1)

# Reset the color
color_res = set_color_srv(reynard_srv.SetColorRequest(0.929,0.49,0.192))
assert color_res.success, f"Set color error: {color_res.status_message}"

# Say hello
say_pub.publish(String("Hello, World From ROS!"))
