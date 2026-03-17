#!/usr/bin/env python3
import rospy
from rss2_msgsrv_pkg.msg import date_cmd_vel

def callback(msg):
    rospy.loginfo(msg.pw_date)
    rospy.loginfo(msg.pw_cmd_vel)

rospy.init_node("msg_sub")
sub = rospy.Subscriber("/pw_topic", date_cmd_vel, callback)
rospy.spin()