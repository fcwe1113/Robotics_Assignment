#!/usr/bin/env python3
import rospy
# Notice the difference with the server ?
from rss_assignment.srv import turtlebot_move_square, turtlebot_move_squareRequest

# The service client node
rospy.init_node("turtlebot_move_client")
# Wait for the service ’/ turtlebot_move_service ’ to run
# You need to start the service first
rospy.wait_for_service("/turtlebot_move_service")
# Connect to the service ’/ turtlebot_move_service ’
turtlebot_service_client = rospy.ServiceProxy("/turtlebot_move_service", turtlebot_move_square)

while not rospy.is_shutdown():
    inputt = input("please enter the square length: ")
    try:
        length = float(inputt)
    except ValueError:
        print("please enter a valid number")
        continue

    inputt = input("please enter the repetition: ")
    try:
        rep = int(inputt)
    except ValueError:
        print("please enter a valid integer")
        continue

    request = turtlebot_move_squareRequest()
    request.sideLength = length
    request.repetitions = rep
    if turtlebot_service_client(request):
        print("movement successful")
    else:
        print("movement failed")

# # Create a request instance
# turtlebot_request_instance = turtlebot_move_squareRequest()
# turtlebot_request_instance.duration = 20
# # Send the request to the server through connection built
# feedback = turtlebot_service_client(turtlebot_request_instance)
# # Show resutls after the service being called rospy . loginfo ( str (feedback ) )
# rospy.loginfo("End of service call")