#!/usr/bin/env python3
import rospy
from rss_assignment.msg import date_cmd_vel
from geometry_msgs.msg import Twist

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
twist = Twist()

def callback(msg):
    rospy.loginfo(msg.pw_date)
    rospy.loginfo(msg.pw_cmd_vel)
    twist.linear.x = msg.pw_cmd_vel.linear.x
    twist.angular.z = msg.pw_cmd_vel.angular.z
    pub.publish(twist)

rospy.init_node("msg_sub")
sub = rospy.Subscriber("/pw_topic", date_cmd_vel, callback)
rospy.spin()