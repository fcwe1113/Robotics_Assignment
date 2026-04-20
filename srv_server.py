#!/usr/bin/env python3
import math

import numpy as np
import rospy
# Notice the difference with the client ?
# from package_where_the_srv_is . srv
# srv_turtlebot_move . srv
# srv_turtlebot_moveResponse and srv_turtlebot_moveRequest are
# maintained by ROS
from rss_assignment.srv import turtlebot_move_square, turtlebot_move_squareResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def my_callback(request):
    rospy.loginfo("Turtlebot_move_service has been called")
    # print(f"length: {request.sideLength}\nrepitition: {request.repetitions}")
    try:

        for _ in range(request.repetitions):
            for _ in range(4):
                start_pos = position
                while math.sqrt((position[0] - start_pos[0]) ** 2 + (position[1] - start_pos[1]) ** 2) < request.sideLength:
                    vel.linear.x = 0.2
                    vel.angular.z = 0.0
                    pw_pub.publish(vel)
                target_angle = (orientation + 90) % 360
                while not (target_angle - 5 <= orientation <= target_angle + 5):
                    vel.linear.x = 0.0
                    vel.angular.z = 0.2
                    pw_pub.publish(vel)

        vel.linear.x = 0
        vel.angular.z = 0
        pw_pub.publish(vel)
        rate.sleep()

        return turtlebot_move_squareResponse(True)

    except:
        return turtlebot_move_squareResponse(False)

def odom_callback(msg):
    global position
    global orientation
    quat = msg.pose.pose.orientation
    position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    orientation = (np.arctan2(2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2 * (quat.y ** 2 + quat.z ** 2)) * 180 / math.pi + 360) % 360

rospy.init_node("turtlebot_move_server")
# This is the service called ’/ turtlebot_move_service ’
pw_sevice = rospy.Service("/turtlebot_move_service", turtlebot_move_square, my_callback)
pw_pub = rospy.Publisher("/cmd_vel" , Twist, queue_size=1)
rospy.Subscriber('/odom', Odometry, odom_callback) # check if functioning
vel = Twist()
# Make sure counting second by second
rate = rospy.Rate(1)
rospy.loginfo("Service/turtlebot_move_service is ready!")
# Maintian the service open
rospy.spin()