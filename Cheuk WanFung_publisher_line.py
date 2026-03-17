#!/usr/bin/env python3
import rospy
from rss_msgsrv_pkg.msg import date_cmd_vel
from geometry_msgs.msg import Twist
from datetime import datetime
from move_turtlebot import MoveTurtleBot

rospy.init_node("msg_pub")
now = datetime.now()
date_str = now.strftime("%m/%d/%y,%H:%M:%S")
pw_cmd_vel = date_cmd_vel()
pw_cmd_vel.pw_date = date_str
pw_cmd_vel.pw_cmd_vel.linear.x = 0.5
pw_cmd_vel.pw_cmd_vel.angular.z = 0.1
pub = rospy.Publisher("/pw_topic" , date_cmd_vel, queue_size=1)
rate = rospy.Rate(1)
moveturtlebot_object = MoveTurtleBot()

while not rospy.is_shutdown():
    moveturtlebot_object.move_turtlebot(1, 1, 0)
    pub.publish(pw_cmd_vel)
    rate.sleep()