#!/usr/bin/env python
import rospy
# Import the service message used by the service / gazebo / get_link_state
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest

# Initialise a ROS node with the name service_client
rospy.init_node("service_client")
# Wait for the service client / gazebo / get_link_state to be running --> How ?
rospy.wait_for_service("/gazebo/get_link_state")
# Create the connection to the service
get_pos_service = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
# Create an object of type GetLinkStateRequest
get_pos_object = GetLinkStateRequest()
# Specify the two arguments desired by GetLinkState
get_pos_object.link_name = "turtlebot3_burger::base_footprint"
get_pos_object.reference_frame = " "
# Send through the connection position information of the robot , and wait for the robot execute
result = get_pos_service(get_pos_object)
# Print the result given by the service called
print(result)